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

#pragma once

#include <driver/gpio.h>

namespace octavo {

class Buzzer {
public:
	Buzzer(gpio_num_t pin, bool active_low);
	~Buzzer() = delete;

	void on();
	void off();

private:
	inline int active() const { return active_low_ ? 0 : 1; }
	inline int inactive() const { return active_low_ ? 1 : 0; }

	const gpio_num_t pin_;
	const bool active_low_;
};

} // namespace octavo
