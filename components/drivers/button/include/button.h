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
 * components/drivers/button/include/button.h
 *
 * Button driver with debounce, event detection, and callbacks
 *
 ****************************************************************************/

#ifndef __COMPONENTS_DRIVERS_BUTTON_INCLUDE_BUTTON_H
#define __COMPONENTS_DRIVERS_BUTTON_INCLUDE_BUTTON_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <esp_err.h>
#include <stdbool.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef enum
{
  BUTTON_EVENT_PRESSED = 0,           /* Button physically pressed */
  BUTTON_EVENT_RELEASED,              /* Button physically released */
  BUTTON_EVENT_SINGLE_CLICK,          /* Short press + release */
  BUTTON_EVENT_DOUBLE_CLICK,          /* Two short presses */
  BUTTON_EVENT_LONG_PRESS,            /* Held for configured duration */
  BUTTON_EVENT_EXTRA_LONG_PRESS_1,    /* Held for extra duration 1 */
  BUTTON_EVENT_EXTRA_LONG_PRESS_2,    /* Held for extra duration 2 */
} button_event_t;

typedef void (*button_callback_t)(button_event_t event);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: button_init
 *
 * Description:
 *   Initialize the button driver with callback for events.
 *
 * Input Parameters:
 *   callback - Function to call when button events occur
 *
 * Returned Value:
 *   ESP_OK on success; ESP_FAIL on failure
 *
 ****************************************************************************/

esp_err_t button_init(button_callback_t callback);

/****************************************************************************
 * Name: button_deinit
 *
 * Description:
 *   Deinitialize the button driver and release resources.
 *
 * Returned Value:
 *   ESP_OK on success; ESP_FAIL on failure
 *
 ****************************************************************************/

esp_err_t button_deinit(void);

/****************************************************************************
 * Name: button_get_state
 *
 * Description:
 *   Get current button physical state (pressed or not).
 *
 * Returned Value:
 *   true if button is currently pressed; false otherwise
 *
 ****************************************************************************/

bool button_get_state(void);

#endif /* __COMPONENTS_DRIVERS_BUTTON_INCLUDE_BUTTON_H */