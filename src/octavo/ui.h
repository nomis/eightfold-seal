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
#include <led_strip.h>
#include <sdkconfig.h>

#include <bitset>
#include <chrono>
#include <mutex>
#include <unordered_map>
#include <vector>

#include "debounce.h"
#include "thread.h"

namespace octavo {

class Buzzer;
class Device;
class Logging;

namespace door {

struct Alarm;

} // namespace door

namespace ui {

struct RGBColour {
	uint8_t red;
	uint8_t green;
	uint8_t blue;
} __attribute__((packed));

namespace colour {

static constexpr const RGBColour OFF = {0, 0, 0};
static constexpr const RGBColour RED = {255, 0, 0};
static constexpr const RGBColour ORANGE = {255, 96, 0};
static constexpr const RGBColour YELLOW = {255, 255, 0};
static constexpr const RGBColour GREEN = {0, 255, 0};
static constexpr const RGBColour CYAN = {0, 255, 255};
static constexpr const RGBColour BLUE = {0, 0, 255};
static constexpr const RGBColour MAGENTA = {255, 0, 255};
static constexpr const RGBColour WHITE = {255, 255, 255};

} // namespace colour

struct LEDState {
	RGBColour colour;
	unsigned long duration_ms;

	uint64_t remaining_us{duration_ms * 1000UL};
};

struct LEDSequence {
	unsigned long duration_ms;
	std::vector<LEDState> states;

	uint64_t remaining_us{duration_ms * 1000UL};
};

/* Ordered by priority (high to low) */
enum class Event {
	DOOR_ALARM2,
	DOOR_ALARM1,
	NETWORK_UNCONFIGURED_FAILED,
	NETWORK_CONFIGURED_FAILED,
	NETWORK_ERROR,
	NETWORK_CONFIGURED_CONNECTING,
	NETWORK_CONFIGURED_DISCONNECTED,
	NETWORK_UNCONFIGURED_CONNECTING,
	NETWORK_UNCONFIGURED_DISCONNECTED,
	OTA_UPDATE_ERROR,
	IDENTIFY,
	DOOR_OPENED,
	DOOR_CLOSED,
	OTA_UPDATE_OK,
	CORE_DUMP_PRESENT,
	NETWORK_CONNECT,
	NETWORK_CONNECTED,
	IDLE,
};

enum class NetworkState {
	DISCONNECTED,
	CONNECTING,
	CONNECTED,
	FAILED,
};

} // namespace ui

class UserInterface: public WakeupThread {
public:
	UserInterface(Logging &logging, Buzzer &buzzer, gpio_num_t network_join_pin, bool active_low);
	~UserInterface() = delete;

	// cppcheck-suppress duplInheritedMember
	static constexpr const char *TAG = "octavo.UI";

	void attach(Device &device);
	void start();

	void network_state(bool configured, ui::NetworkState state);
	void network_error();
	void identify(uint16_t seconds);
	void door_opened();
	void door_closed();
	void alarm(const door::Alarm &status);
	void ota_update(bool ok);
	void core_dump(bool present);

private:
	void buzzer_test();

	static constexpr const unsigned long DEBOUNCE_PRESS_US = std::chrono::microseconds(std::chrono::milliseconds(100)).count();
	static constexpr const unsigned long DEBOUNCE_RELEASE_US = std::chrono::microseconds(std::chrono::seconds(1)).count();
	static constexpr const uint8_t LED_LEVEL = CONFIG_OCTAVO_UI_LED_BRIGHTNESS;
	static constexpr const uint64_t BUZZER_DURATION_US = std::chrono::microseconds(std::chrono::milliseconds(250)).count();
	static constexpr const uint64_t BUZZER_MIN_INTERVAL_US = BUZZER_DURATION_US;
	static constexpr const uint64_t BUZZER_MAX_INTERVAL_US = std::chrono::microseconds(std::chrono::seconds(5)).count();
	static constexpr const uint64_t BUZZER_RAPID_DURATION_US = std::chrono::microseconds(std::chrono::seconds(2)).count();;
	static const std::unordered_map<ui::Event,ui::LEDSequence> led_sequences_;

	unsigned long run_tasks() override;
	void uart_handler();

	void crash();
	void print_memory();
	void print_tasks();

	void start_event(ui::Event event);
	void restart_event(ui::Event event);
	bool event_active(ui::Event event);
	void stop_event(ui::Event event);
	void stop_events(std::initializer_list<ui::Event> events);
	void set_led(ui::RGBColour colour);
	unsigned long update_buzzer();
	unsigned long update_led();

	Logging &logging_;
	Buzzer &buzzer_;
	Debounce button_debounce_;
	led_strip_handle_t led_strip_{nullptr};
	Device *device_{nullptr};

	uint64_t render_time_us_{0};
	ui::Event render_event_{ui::Event::IDLE};

	std::mutex mutex_;
	std::bitset<static_cast<unsigned long>(ui::Event::IDLE) + 1> active_events_;
	std::unordered_map<ui::Event,ui::LEDSequence> active_sequence_;
	uint64_t buzzer_start_time_us_{0};
	uint64_t buzzer_stop_time_us_{0};
	bool buzzer_test_{false};
	uint64_t alarm_level1_time_us_{0};
	uint64_t alarm_level2_time_us_{0};
};

} // namespace octavo
