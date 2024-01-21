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

#include <cstddef>
#include <sdkconfig.h>


namespace octavo {

static constexpr const char *TAG = "octavo";
static constexpr const size_t MAX_DOORS = CONFIG_OCTAVO_MAX_DOORS;

#ifndef CONFIG_OCTAVO_SWITCH_ACTIVE_LOW
#define CONFIG_OCTAVO_SWITCH_ACTIVE_LOW 0
#endif
static constexpr const bool SWITCH_ACTIVE_LOW = CONFIG_OCTAVO_SWITCH_ACTIVE_LOW;

#ifndef CONFIG_OCTAVO_BUZZER_ACTIVE_LOW
#define CONFIG_OCTAVO_BUZZER_ACTIVE_LOW 0
#endif
static constexpr const bool BUZZER_ACTIVE_LOW = CONFIG_OCTAVO_BUZZER_ACTIVE_LOW;

} // namespace octavo
