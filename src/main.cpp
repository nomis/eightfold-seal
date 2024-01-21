/*
 * eightfold-seal - ESP32 Zigbee door alarm
 * Copyright 2024  Simon Arlott
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "octavo/main.h"

#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <driver/gpio.h>

#include "octavo/device.h"
#include "octavo/door.h"
#include "octavo/log.h"
#include "octavo/ui.h"

using namespace octavo;

static_assert(octavo::Device::NUM_EP_PER_DEVICE + MAX_DOORS * octavo::Door::NUM_EP_PER_DOOR <= ZB_MAX_EP_NUMBER,
	"You'll need to ask Espressif to let you use more endpoints");

extern "C" void app_main() {
	esp_err_t err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		err = nvs_flash_init();
	}
	ESP_ERROR_CHECK(err);

	auto &logging = *new Logging{};

	ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL2));

	auto &ui = *new UserInterface{logging, GPIO_NUM_4, true};
	auto &device = *new Device{ui};

	/*                                Switch       Active Low
	 *                                -----------  -----------------
	 */
	if (MAX_DOORS >= 1) (new Door{1, GPIO_NUM_3,  SWITCH_ACTIVE_LOW })->attach(device);
	if (MAX_DOORS >= 2) (new Door{2, GPIO_NUM_2,  SWITCH_ACTIVE_LOW })->attach(device);
	if (MAX_DOORS >= 3) (new Door{3, GPIO_NUM_11, SWITCH_ACTIVE_LOW })->attach(device);
	if (MAX_DOORS >= 4) (new Door{4, GPIO_NUM_10, SWITCH_ACTIVE_LOW })->attach(device);
	device.start();

	ui.attach(device);
	ui.start();

	TaskStatus_t status;

	vTaskGetInfo(nullptr, &status, pdTRUE, eRunning);
	ESP_LOGD(TAG, "Free stack: %lu", status.usStackHighWaterMark);
}
