# MAIA
ADAS for blind dogs!

## Third-Party Dependencies

This project uses the VL53L5CX Ultra Lite Driver (ULD) from
STMicroelectronics, licensed under BSD-3-Clause.

The ULD is NOT included in this repository. It is downloaded
automatically at build time via the ESP-IDF Component Manager
using the manifest at `components/drivers/vl53l5cx/idf_component.yml`.

- Source: https://github.com/STMicroelectronics/stm32-vl53l5cx
- License: BSD-3-Clause, (c) STMicroelectronics

### Integration approach

The ST ULD provides the sensor core logic (`vl53l5cx_api.c`)
and expects the user to implement a platform layer (`platform.h`)
with I2C and delay callbacks.

MAIA provides its own `platform.h` and `vl53l5cx_platform.c`
using the ESP-IDF v6.0 new I2C master API. The ST `porting/`
directory is intentionally excluded from the build — only
`modules/vl53l5cx_api.c` is compiled from the ULD.

This keeps the repository 100% Apache 2.0 while the ST ULD
is consumed as an external dependency at build time.

## Building
idf.py build

The Component Manager downloads the ST ULD automatically.
No manual steps required.

## ⚠️ DISCLAIMER

THIS IS AN EXPERIMENTAL OPEN-SOURCE PROJECT FOR EDUCATIONAL
AND RESEARCH PURPOSES ONLY.

No Warranty of Operation: This ADAS system for blind dogs is
provided on an "AS IS" BASIS, without warranties or conditions
of any kind, either express or implied. ToF sensors are subject
to environmental interference, signal noise, blind spots, and
false negatives/positives.

Assumption of Risk: The use of this hardware and firmware on
animals is at the sole discretion and risk of the user/builder.
The author(s) shall not be held liable for any collisions,
injuries, property damage, or any other incidents resulting
from the use or failure of this device.

Not a Medical or Safety Device: This project is NOT a certified
medical or assistive device. It is not a substitute for human
supervision, professional veterinary advice, or proper
orientation training for visually impaired animals.

Hardware Safety: The builder is responsible for ensuring
electrical safety (battery insulation, thermal management) and
mechanical integrity (secure attachment to the animal) to
prevent risks of shock, fire, or physical harm.

<img width="430" height="761" alt="image" src="https://github.com/user-attachments/assets/a9813dc2-92f9-4854-a0d0-9907cdfece1f" />
