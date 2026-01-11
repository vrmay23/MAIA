/*
 * Copyright 2026 Vinicius May
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/****************************************************************************
 * components/drivers/button/src/button.c
 *
 * Button driver implementation with state machine
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "button.h"
#include "maia_board.h"
#include <driver/gpio.h>
#include <esp_timer.h>
#include <esp_log.h>
#include <string.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TAG "[BUTTON]"

/* Thresholds from Kconfig */

#define DEBOUNCE_TIME_MS           CONFIG_MAIA_BUTTON_DEBOUNCE_MS
#define DOUBLE_CLICK_WINDOW_MS     CONFIG_MAIA_BUTTON_DOUBLE_CLICK_WINDOW_MS
#define LONG_PRESS_THRESHOLD_MS    CONFIG_MAIA_BUTTON_LONG_PRESS_MS
#define EXTRA_LONG_1_THRESHOLD_MS  CONFIG_MAIA_BUTTON_EXTRA_LONG_PRESS_1_MS
#define EXTRA_LONG_2_THRESHOLD_MS  CONFIG_MAIA_BUTTON_EXTRA_LONG_PRESS_2_MS

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef enum
{
  BUTTON_STATE_IDLE = 0,
  BUTTON_STATE_DEBOUNCING_PRESS,
  BUTTON_STATE_PRESSED,
  BUTTON_STATE_DEBOUNCING_RELEASE,
  BUTTON_STATE_WAIT_DOUBLE_CLICK,
} button_state_t;

typedef struct
{
  button_callback_t callback;
  esp_timer_handle_t debounce_timer;
  esp_timer_handle_t press_timer;
  esp_timer_handle_t double_timer;
  button_state_t state;
  int64_t press_time;
  bool initialized;
  uint8_t click_count;
} button_context_t;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static button_context_t g_button_ctx = {0};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void IRAM_ATTR button_isr_handler(void *arg);
static void button_debounce_timer_callback(void *arg);
static void button_press_timer_callback(void *arg);
static void button_double_click_timer_callback(void *arg);
static void button_process_press(void);
static void button_process_release(void);
static void button_notify_event(button_event_t event);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: button_notify_event
 *
 * Description:
 *   Notify registered callback of button event.
 *
 ****************************************************************************/

static void button_notify_event(button_event_t event)
{
  if (g_button_ctx.callback != NULL)
    {
      g_button_ctx.callback(event);
    }
}

/****************************************************************************
 * Name: button_isr_handler
 *
 * Description:
 *   GPIO interrupt handler for button press/release detection.
 *   Runs in ISR context - keep minimal!
 *
 ****************************************************************************/

static void IRAM_ATTR button_isr_handler(void *arg)
{
  /* Start debounce timer on ANY edge (press or release) */

  if (g_button_ctx.state == BUTTON_STATE_IDLE ||
      g_button_ctx.state == BUTTON_STATE_PRESSED ||
      g_button_ctx.state == BUTTON_STATE_WAIT_DOUBLE_CLICK)
    {
      esp_timer_start_once(g_button_ctx.debounce_timer,
                           DEBOUNCE_TIME_MS * 1000);

      /* Update state based on current GPIO level */

      int level = gpio_get_level(MAIA_GPIO_BUTTON);

      if (level == 0)  /* Active low - button pressed */
        {
          g_button_ctx.state = BUTTON_STATE_DEBOUNCING_PRESS;
        }
      else  /* Button released */
        {
          g_button_ctx.state = BUTTON_STATE_DEBOUNCING_RELEASE;
        }
    }
}

/****************************************************************************
 * Name: button_debounce_timer_callback
 *
 * Description:
 *   Debounce timer callback. Validates GPIO state after debounce period.
 *
 ****************************************************************************/

static void button_debounce_timer_callback(void *arg)
{
  int level = gpio_get_level(MAIA_GPIO_BUTTON);

  if (g_button_ctx.state == BUTTON_STATE_DEBOUNCING_PRESS)
    {
      if (level == 0)  /* Still pressed - valid press */
        {
          button_process_press();
        }
      else  /* Was noise - ignore */
        {
          g_button_ctx.state = BUTTON_STATE_IDLE;
        }
    }
  else if (g_button_ctx.state == BUTTON_STATE_DEBOUNCING_RELEASE)
    {
      if (level == 1)  /* Still released - valid release */
        {
          button_process_release();
        }
      else  /* Was noise - ignore */
        {
          g_button_ctx.state = BUTTON_STATE_PRESSED;
        }
    }
}

/****************************************************************************
 * Name: button_process_press
 *
 * Description:
 *   Process validated button press event.
 *
 ****************************************************************************/

static void button_process_press(void)
{
  g_button_ctx.press_time = esp_timer_get_time() / 1000;  /* ms */
  g_button_ctx.state = BUTTON_STATE_PRESSED;

  ESP_LOGI(TAG, "Button PRESSED");
  button_notify_event(BUTTON_EVENT_PRESSED);

  /* Start press timer for long press detection */

  esp_timer_start_once(g_button_ctx.press_timer,
                       LONG_PRESS_THRESHOLD_MS * 1000);
}

/****************************************************************************
 * Name: button_process_release
 *
 * Description:
 *   Process validated button release event.
 *
 ****************************************************************************/

static void button_process_release(void)
{
  /* Stop all timers */

  esp_timer_stop(g_button_ctx.press_timer);
  esp_timer_stop(g_button_ctx.double_timer);

  int64_t now = esp_timer_get_time() / 1000;
  int64_t held_time = now - g_button_ctx.press_time;

  ESP_LOGI(TAG, "Button RELEASED (held for %lld ms)", held_time);
  button_notify_event(BUTTON_EVENT_RELEASED);

  /* Determine what type of press it was */

  if (held_time >= EXTRA_LONG_2_THRESHOLD_MS)
    {
      /* Already notified EXTRA_LONG_PRESS_2 */

      g_button_ctx.click_count = 0;
      g_button_ctx.state = BUTTON_STATE_IDLE;
    }
  else if (held_time >= EXTRA_LONG_1_THRESHOLD_MS)
    {
      /* Already notified EXTRA_LONG_PRESS_1 */

      g_button_ctx.click_count = 0;
      g_button_ctx.state = BUTTON_STATE_IDLE;
    }
  else if (held_time >= LONG_PRESS_THRESHOLD_MS)
    {
      /* Already notified LONG_PRESS */

      g_button_ctx.click_count = 0;
      g_button_ctx.state = BUTTON_STATE_IDLE;
    }
  else
    {
      /* Short press - check for SINGLE/DOUBLE click */

      g_button_ctx.click_count++;

      if (g_button_ctx.click_count == 1)
        {
          /* Wait for possible second click */

          g_button_ctx.state = BUTTON_STATE_WAIT_DOUBLE_CLICK;
          esp_timer_start_once(g_button_ctx.double_timer,
                               DOUBLE_CLICK_WINDOW_MS * 1000);
        }
      else if (g_button_ctx.click_count == 2)
        {
          /* Double click detected */

          ESP_LOGI(TAG, "DOUBLE_CLICK");
          button_notify_event(BUTTON_EVENT_DOUBLE_CLICK);
          g_button_ctx.click_count = 0;
          g_button_ctx.state = BUTTON_STATE_IDLE;
        }
    }
}

/****************************************************************************
 * Name: button_press_timer_callback
 *
 * Description:
 *   Press timer callback. Detects long press events with cascading.
 *
 ****************************************************************************/

static void button_press_timer_callback(void *arg)
{
  if (g_button_ctx.state != BUTTON_STATE_PRESSED)
    {
      return;
    }

  int64_t now = esp_timer_get_time() / 1000;
  int64_t held_time = now - g_button_ctx.press_time;

  /* Cascading timer logic - check thresholds */

  if (held_time >= EXTRA_LONG_2_THRESHOLD_MS)
    {
      /* Extra long press 2 reached - stop timer */

      ESP_LOGI(TAG, "EXTRA_LONG_PRESS_2 detected (held %lld ms)",
               held_time);
      button_notify_event(BUTTON_EVENT_EXTRA_LONG_PRESS_2);
      esp_timer_stop(g_button_ctx.press_timer);
    }
  else if (held_time >= EXTRA_LONG_1_THRESHOLD_MS)
    {
      /* Extra long press 1 reached - schedule next check */

      ESP_LOGI(TAG, "EXTRA_LONG_PRESS_1 detected (held %lld ms)",
               held_time);
      button_notify_event(BUTTON_EVENT_EXTRA_LONG_PRESS_1);

      /* Reschedule for EXTRA_LONG_2 */

      int64_t next_delay = (EXTRA_LONG_2_THRESHOLD_MS - held_time) * 1000;
      esp_timer_start_once(g_button_ctx.press_timer, next_delay);
    }
  else if (held_time >= LONG_PRESS_THRESHOLD_MS)
    {
      /* Long press reached - schedule next check */

      ESP_LOGI(TAG, "LONG_PRESS detected (held %lld ms)", held_time);
      button_notify_event(BUTTON_EVENT_LONG_PRESS);

      /* Reschedule for EXTRA_LONG_1 */

      int64_t next_delay = (EXTRA_LONG_1_THRESHOLD_MS - held_time) * 1000;
      esp_timer_start_once(g_button_ctx.press_timer, next_delay);
    }
}

/****************************************************************************
 * Name: button_double_click_timer_callback
 *
 * Description:
 *   Double click timer callback. Fires if second click didn't arrive.
 *
 ****************************************************************************/

static void button_double_click_timer_callback(void *arg)
{
  if (g_button_ctx.state == BUTTON_STATE_WAIT_DOUBLE_CLICK)
    {
      /* Timeout - was single click */

      ESP_LOGI(TAG, "SINGLE_CLICK");
      button_notify_event(BUTTON_EVENT_SINGLE_CLICK);
      g_button_ctx.click_count = 0;
      g_button_ctx.state = BUTTON_STATE_IDLE;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: button_init
 *
 * Description:
 *   Initialize button driver with callback.
 *
 ****************************************************************************/

esp_err_t button_init(button_callback_t callback)
{
  esp_err_t ret;

  if (g_button_ctx.initialized)
    {
      ESP_LOGW(TAG, "Button already initialized");
      return ESP_ERR_INVALID_STATE;
    }

  if (callback == NULL)
    {
      ESP_LOGE(TAG, "Callback cannot be NULL");
      return ESP_ERR_INVALID_ARG;
    }

  ESP_LOGI(TAG, "Initializing button driver");

  /* Initialize context */

  memset(&g_button_ctx, 0, sizeof(button_context_t));
  g_button_ctx.callback = callback;
  g_button_ctx.state = BUTTON_STATE_IDLE;

  /* Create debounce timer */

  esp_timer_create_args_t debounce_args = {
    .callback = button_debounce_timer_callback,
    .name = "button_debounce"
  };

  ret = esp_timer_create(&debounce_args, &g_button_ctx.debounce_timer);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to create debounce timer");
      return ret;
    }

  /* Create press timer (for long press detection) */

  esp_timer_create_args_t press_args = {
    .callback = button_press_timer_callback,
    .name = "button_press"
  };

  ret = esp_timer_create(&press_args, &g_button_ctx.press_timer);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to create press timer");
      esp_timer_delete(g_button_ctx.debounce_timer);
      return ret;
    }

  /* Create double click timer */

  esp_timer_create_args_t double_args = {
    .callback = button_double_click_timer_callback,
    .name = "button_double"
  };

  ret = esp_timer_create(&double_args, &g_button_ctx.double_timer);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to create double click timer");
      esp_timer_delete(g_button_ctx.debounce_timer);
      esp_timer_delete(g_button_ctx.press_timer);
      return ret;
    }

  /* Add GPIO ISR handler */

  ret = gpio_isr_handler_add(MAIA_GPIO_BUTTON, button_isr_handler, NULL);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to add GPIO ISR handler");
      esp_timer_delete(g_button_ctx.debounce_timer);
      esp_timer_delete(g_button_ctx.press_timer);
      esp_timer_delete(g_button_ctx.double_timer);
      return ret;
    }

  /* Enable interrupts on both edges */

  ret = gpio_set_intr_type(MAIA_GPIO_BUTTON, GPIO_INTR_ANYEDGE);
  if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to set interrupt type");
      gpio_isr_handler_remove(MAIA_GPIO_BUTTON);
      esp_timer_delete(g_button_ctx.debounce_timer);
      esp_timer_delete(g_button_ctx.press_timer);
      esp_timer_delete(g_button_ctx.double_timer);
      return ret;
    }

  g_button_ctx.initialized = true;

  ESP_LOGI(TAG, "Button driver initialized successfully");

  return ESP_OK;
}

/****************************************************************************
 * Name: button_deinit
 *
 * Description:
 *   Deinitialize button driver.
 *
 ****************************************************************************/

esp_err_t button_deinit(void)
{
  if (!g_button_ctx.initialized)
    {
      return ESP_ERR_INVALID_STATE;
    }

  /* Disable interrupts */

  gpio_set_intr_type(MAIA_GPIO_BUTTON, GPIO_INTR_DISABLE);
  gpio_isr_handler_remove(MAIA_GPIO_BUTTON);

  /* Delete timers */

  if (g_button_ctx.debounce_timer != NULL)
    {
      esp_timer_stop(g_button_ctx.debounce_timer);
      esp_timer_delete(g_button_ctx.debounce_timer);
    }

  if (g_button_ctx.press_timer != NULL)
    {
      esp_timer_stop(g_button_ctx.press_timer);
      esp_timer_delete(g_button_ctx.press_timer);
    }

  if (g_button_ctx.double_timer != NULL)
    {
      esp_timer_stop(g_button_ctx.double_timer);
      esp_timer_delete(g_button_ctx.double_timer);
    }

  /* Clear context */

  memset(&g_button_ctx, 0, sizeof(button_context_t));

  ESP_LOGI(TAG, "Button driver deinitialized");

  return ESP_OK;
}

/****************************************************************************
 * Name: button_get_state
 *
 * Description:
 *   Get current physical button state.
 *
 ****************************************************************************/

bool button_get_state(void)
{
  return (gpio_get_level(MAIA_GPIO_BUTTON) == 0);  /* Active low */
}