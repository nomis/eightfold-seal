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

#include "octavo/buzzer.h"

#include <esp_err.h>
#include <driver/gpio.h>

namespace octavo {

Buzzer::Buzzer(gpio_num_t pin, bool active_low) : pin_(pin),
		active_low_(active_low) {
	gpio_config_t config{};

	config.pin_bit_mask = 1ULL << pin_;
	config.mode = GPIO_MODE_OUTPUT;
	config.pull_up_en = GPIO_PULLUP_DISABLE;
	config.pull_down_en = GPIO_PULLDOWN_DISABLE;
	config.intr_type = GPIO_INTR_DISABLE;

	off();
	ESP_ERROR_CHECK(gpio_config(&config));
}

void Buzzer::on() {
	ESP_ERROR_CHECK(gpio_set_level(pin_, active()));
}

void Buzzer::off() {
	ESP_ERROR_CHECK(gpio_set_level(pin_, inactive()));
}

} // namespace octavo
