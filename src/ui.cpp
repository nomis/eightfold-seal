/*
 * eightfold-seal - ESP32 Zigbee door alarm
 * Copyright 2024-2025  Simon Arlott
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

#include "octavo/ui.h"

#include <esp_err.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_partition.h>
#include <esp_timer.h>
#include <driver/gpio.h>
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <hal/uart_ll.h>
#include <led_strip.h>
#include <soc/uart_pins.h>
#include <string.h>
extern "C" {
#include <zboss_api.h>
}

#include <bitset>
#include <memory>
#include <thread>
#include <unordered_map>

#include "octavo/buzzer.h"
#include "octavo/debounce.h"
#include "octavo/device.h"
#include "octavo/door.h"
#include "octavo/log.h"
#include "octavo/zigbee.h"

namespace octavo {

using namespace ui;
using namespace ui::colour;

const std::unordered_map<Event,LEDSequence> UserInterface::led_sequences_{
	{ Event::IDLE,                                 {    0, { { OFF, 0 }                      } } },
	{ Event::NETWORK_CONNECT,                      { 8000, { { GREEN, 5000 }, { OFF, 0 }     } } },
	{ Event::NETWORK_CONNECTED,                    {    0, { { GREEN, 250 }, { OFF, 2750 }   } } },
	{ Event::CORE_DUMP_PRESENT,                    {    0, { { WHITE, 200 }, { RED, 200 },
	                                                         { ORANGE, 200 }, { YELLOW, 200 },
	                                                         { GREEN, 200 }, { CYAN, 200 },
	                                                         { BLUE, 200 }, { MAGENTA, 200 }
	                                                                                         } } },
	{ Event::OTA_UPDATE_OK,                        {  500, { { CYAN, 0 }                     } } },
	{ Event::DOOR_OPENED,                          { 2000, { { ORANGE, 0 }                   } } },
	{ Event::DOOR_CLOSED,                          { 2000, { { BLUE, 0 }                     } } },
	{ Event::IDENTIFY,                             { 3000, { { MAGENTA, 0 }                  } } },
	{ Event::OTA_UPDATE_ERROR,                     { 3000, { { RED, 200 }, { OFF, 200 }      } } },
	{ Event::NETWORK_UNCONFIGURED_DISCONNECTED,    {    0, { { WHITE, 0 }                    } } },
	{ Event::NETWORK_UNCONFIGURED_CONNECTING,      {    0, { { WHITE, 250 }, { OFF, 250 }    } } },
	{ Event::NETWORK_CONFIGURED_DISCONNECTED,      {    0, { { YELLOW, 0 }                   } } },
	{ Event::NETWORK_CONFIGURED_CONNECTING,        {    0, { { YELLOW, 250 }, { OFF, 250 }   } } },
	{ Event::NETWORK_ERROR,                        { 1000, { { RED, 250 }, { OFF, 250 }      } } },
	{ Event::NETWORK_CONFIGURED_FAILED,            {    0, { { RED, 0 }                      } } },
	{ Event::NETWORK_UNCONFIGURED_FAILED,          {    0, { { RED, 500 }, { OFF, 500 }      } } },
	{ Event::DOOR_ALARM1,                          {    0, { { ORANGE, 500 }, { OFF, 500 }   } } },
	{ Event::DOOR_ALARM2,                          {    0, { { ORANGE, 250 }, { WHITE, 250 } } } },
};

} // namespace octavo

namespace octavo {

namespace colour = ui::colour;
using ui::Event;
using ui::NetworkState;
using ui::RGBColour;

UserInterface::UserInterface(Logging &logging, Buzzer &buzzer,
		gpio_num_t network_join_pin, bool active_low)
		: WakeupThread("UI", false), logging_(logging), buzzer_(buzzer),
		button_debounce_(network_join_pin, active_low, DEBOUNCE_PRESS_US,
			DEBOUNCE_RELEASE_US) {
	led_strip_config_t led_strip_config{};
	led_strip_rmt_config_t rmt_config{};

	led_strip_config.max_leds = 1;
	led_strip_config.strip_gpio_num = GPIO_NUM_8;
	led_strip_config.led_model = LED_MODEL_WS2812;
	led_strip_config.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB;
	rmt_config.resolution_hz = 10 * 1000 * 1000;

	ESP_ERROR_CHECK(led_strip_new_rmt_device(&led_strip_config, &rmt_config, &led_strip_));
	set_led(colour::OFF);

	ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, U0TXD_GPIO_NUM, U0RXD_GPIO_NUM,
		UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

	ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, UART_PIN_NO_CHANGE, GPIO_NUM_15,
		UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

	uart_config_t uart_config{};
	uart_config.baud_rate = 115200;
	uart_config.data_bits = UART_DATA_8_BITS;
	uart_config.parity = UART_PARITY_DISABLE;
	uart_config.stop_bits = UART_STOP_BITS_1;
	uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;

	ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
	ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, SOC_UART_FIFO_LEN + 1,
		0, 0, nullptr, ESP_INTR_FLAG_LEVEL1));

	ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
	ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, SOC_UART_FIFO_LEN + 1,
		0, 0, nullptr, ESP_INTR_FLAG_LEVEL1));

	uart_intr_config_t uart_int_config{};
	uart_int_config.intr_enable_mask = UART_INTR_RXFIFO_FULL;
	uart_int_config.rxfifo_full_thresh = 1;

	ESP_ERROR_CHECK(uart_intr_config(UART_NUM_0, &uart_int_config));
	ESP_ERROR_CHECK(uart_enable_rx_intr(UART_NUM_0));

	ESP_ERROR_CHECK(uart_intr_config(UART_NUM_1, &uart_int_config));
	ESP_ERROR_CHECK(uart_enable_rx_intr(UART_NUM_1));
}

void UserInterface::set_led(RGBColour colour) {
	ESP_ERROR_CHECK(led_strip_set_pixel(led_strip_, 0,
		colour.red * LED_LEVEL / 255,
		colour.green * LED_LEVEL / 255,
		colour.blue * LED_LEVEL / 255));
	ESP_ERROR_CHECK(led_strip_refresh(led_strip_));
}

void UserInterface::attach(Device &device) {
	device_ = &device;
}

void UserInterface::start() {
	std::thread t;

	button_debounce_.start(*this);

	make_thread(t, "ui_main", 4096, 10, &UserInterface::run_loop, this);
	t.detach();

	make_thread(t, "ui_uart0", 6144, 1, &UserInterface::uart_handler, this, UART_NUM_0);
	t.detach();

	make_thread(t, "ui_uart1", 6144, 1, &UserInterface::uart_handler, this, UART_NUM_1);
	t.detach();
}

unsigned long UserInterface::run_tasks() {
	DebounceResult debounce = button_debounce_.run();

	if (debounce.changed
			&& button_debounce_.value()
			&& !button_debounce_.first()) {
		Device *device = device_;

		ESP_LOGI(TAG, "Network join/leave button pressed");

		if (device) {
			device->join_or_leave_network();
		}
	}

	return std::min(std::min(debounce.wait_ms, update_buzzer()), update_led());
}

void UserInterface::uart_handler(uart_port_t port) {
	char buf[1];

	while (true) {
		if (uart_read_bytes(port, buf, sizeof(buf), portMAX_DELAY) == 1) {
			Device *device = device_;

			if (buf[0] == '0') {
				logging_.set_app_level(ESP_LOG_NONE);
				logging_.set_sys_level(ESP_LOG_NONE);
			} else if (buf[0] >= '1' && buf[0] <= '5') {
				logging_.set_app_level(static_cast<esp_log_level_t>(buf[0] - '1' + 1));
			} else if (buf[0] >= '6' && buf[0] <= '9') {
				logging_.set_sys_level(static_cast<esp_log_level_t>(buf[0] - '6' + 1));
			} else if (buf[0] == 'A') {
				zb_assert(__FILE__, __LINE__);
			} else if (device && buf[0] == 'b') {
				device->print_bindings();
			} else if (buf[0] == 'C') {
				crash();
			} else if (device && buf[0] == 'd') {
				device->print_core_dump(false);
			} else if (device && buf[0] == 'D') {
				vTaskPrioritySet(nullptr, 20);
				device->print_core_dump(true);
				vTaskPrioritySet(nullptr, 1);
			} else if (device && buf[0] == 'E') {
				device->erase_core_dump();
			} else if (device && buf[0] == 'j') {
				device->join_network();
			} else if (device && buf[0] == 'l') {
				device->leave_network();
			} else if (buf[0] == 'm') {
				print_memory();
			} else if (device && buf[0] == 'n') {
				device->print_neighbours();
			} else if (buf[0] == 'R') {
				esp_restart();
			} else if (buf[0] == 't') {
				print_tasks();
			} else if (buf[0] == 'z') {
				buzzer_test();
			}
		}
	}
}

void UserInterface::crash() {
	uint32_t now_us = esp_timer_get_time();
	uint32_t *x = nullptr;
	ESP_LOGE(TAG, "Crash at 0x%08" PRIx32, now_us);
	// cppcheck-suppress nullPointer
	*x = now_us;
}

void UserInterface::print_memory() {
	size_t total_bytes = heap_caps_get_total_size(MALLOC_CAP_8BIT);
	size_t free_bytes = heap_caps_get_free_size(MALLOC_CAP_8BIT);
	size_t min_free_bytes = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
	size_t largest_free_block_bytes = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);

	ESP_LOGI(TAG, "Total memory: %6zu", total_bytes);
	ESP_LOGI(TAG, "Used memory:  %6zu", total_bytes - free_bytes);
	ESP_LOGI(TAG, "Free memory:  %6zu", free_bytes);
	ESP_LOGI(TAG, "              %6zu (minimum)", min_free_bytes);
	ESP_LOGI(TAG, "              %6zu (largest block)", largest_free_block_bytes);
}

void UserInterface::print_tasks() {
	std::vector<char> buffer(1024);

	vTaskList(buffer.data());
	ESP_LOGI(TAG, "Tasks:\r\n%-*s\tState\tPrio\tStack\tID\r\n%s",
		configMAX_TASK_NAME_LEN - 1, "Name", buffer.data());

	vTaskGetRunTimeStats(buffer.data());
	ESP_LOGI(TAG, "Stats:\r\n%-*s\tRunning\t\tCPU%%\r\n%s",
		configMAX_TASK_NAME_LEN - 1, "Name", buffer.data());
}

void UserInterface::start_event(Event event) {
	unsigned long value = static_cast<unsigned long>(event);

	active_events_.set(value);
	active_sequence_.insert_or_assign(event, led_sequences_.at(event));
}

void UserInterface::restart_event(Event event) {
	stop_event(event);
	start_event(event);
}

bool UserInterface::event_active(Event event) {
	unsigned long value = static_cast<unsigned long>(event);

	return active_events_.test(value);
}

void UserInterface::stop_event(Event event) {
	if (event_active(event)) {
		unsigned long value = static_cast<unsigned long>(event);

		active_events_.reset(value);
		active_sequence_.erase(event);

		if (render_time_us_ && render_event_ == event) {
			render_time_us_ = 0;
		}
	}
}

inline void UserInterface::stop_events(std::initializer_list<Event> events) {
	for (Event event : events)
		stop_event(event);
}

unsigned long UserInterface::update_buzzer() {
	std::unique_lock lock{mutex_};
	uint64_t now_us = esp_timer_get_time();

	if (event_active(Event::DOOR_ALARM2)) {
		if (!buzzer_start_time_us_) {
			buzzer_start_time_us_ = now_us;
			buzzer_stop_time_us_ = 0;
			buzzer_test_ = false;
			buzzer_.on();
		}

		return ULONG_MAX;
	} else if (event_active(Event::DOOR_ALARM1)) {
		if (buzzer_start_time_us_) {
			if (!buzzer_stop_time_us_) {
				buzzer_stop_time_us_ = now_us + BUZZER_DURATION_US;
			}

			if (now_us >= buzzer_stop_time_us_) {
				buzzer_start_time_us_ = 0;
				buzzer_stop_time_us_ = now_us;
				buzzer_test_ = false;
				buzzer_.off();
			}
		}

		if (!buzzer_start_time_us_) {
			uint64_t interval_us;

			if (buzzer_stop_time_us_) {
				uint64_t last_stop_time_us = std::max(buzzer_stop_time_us_, alarm_level1_time_us_);
				uint64_t level1_duration_us = alarm_level2_time_us_ - alarm_level1_time_us_;

				if (level1_duration_us && last_stop_time_us < alarm_level2_time_us_) {
					constexpr uint64_t interval_range_us = (BUZZER_MAX_INTERVAL_US - BUZZER_MIN_INTERVAL_US);
					uint64_t level1_remaining_us = alarm_level2_time_us_ - last_stop_time_us;

					interval_us = BUZZER_MIN_INTERVAL_US
						+ (interval_range_us * level1_remaining_us) / level1_duration_us;
				} else {
					interval_us = BUZZER_MIN_INTERVAL_US;
				}
			} else {
				interval_us = 0;
			}

			uint64_t next_time_us_ = buzzer_stop_time_us_ + interval_us;

			if (now_us >= next_time_us_) {
				buzzer_start_time_us_ = now_us;
				buzzer_stop_time_us_ = now_us + BUZZER_DURATION_US;
				buzzer_.on();
			} else {
				return std::min(static_cast<unsigned long>((next_time_us_ - now_us) / 1000UL), ULONG_MAX - 1);
			}
		}

		return std::min(static_cast<unsigned long>((buzzer_stop_time_us_ - now_us) / 1000UL), ULONG_MAX - 1);
	} else {
		if (buzzer_start_time_us_ && !buzzer_test_) {
			buzzer_stop_time_us_ = now_us;
		}

		if (buzzer_stop_time_us_) {
			if (now_us >= buzzer_stop_time_us_) {
				buzzer_start_time_us_ = 0;
				buzzer_stop_time_us_ = 0;
				buzzer_test_ = false;
				buzzer_.off();
			} else {
				return std::min(static_cast<unsigned long>((buzzer_stop_time_us_ - now_us) / 1000UL), ULONG_MAX - 1);
			}
		}

		return ULONG_MAX;
	}
}

unsigned long UserInterface::update_led() {
	std::unique_lock lock{mutex_};
	uint64_t now_us = esp_timer_get_time();

	if (render_time_us_ && event_active(render_event_)) {
		uint64_t elapsed_us = now_us - render_time_us_;
		auto &sequence = active_sequence_[render_event_];

		if (sequence.states[0].duration_ms) {
			if (elapsed_us >= sequence.states[0].remaining_us) {
				sequence.states[0].remaining_us = sequence.states[0].duration_ms * 1000UL;
				auto state_copy = sequence.states[0];
				sequence.states.erase(sequence.states.begin());
				sequence.states.emplace_back(std::move(state_copy));
			} else {
				sequence.states[0].remaining_us -= elapsed_us;
			}
		}

		if (sequence.duration_ms) {
			if (elapsed_us >= sequence.remaining_us) {
				stop_event(render_event_);
			} else {
				sequence.remaining_us -= elapsed_us;
			}
		}
	}

	unsigned long wait_ms;
	RGBColour colour;
	Event event;
	unsigned long value = ffsl(active_events_.to_ulong());

	if (value) {
		event = static_cast<Event>(value - 1);
	} else {
		event = Event::IDLE;
		start_event(event);
	}

	const auto &sequence = active_sequence_[event];

	if (sequence.states[0].duration_ms) {
		wait_ms = std::min(static_cast<unsigned long>(sequence.states[0].remaining_us / 1000UL), ULONG_MAX - 1);
	} else {
		wait_ms = ULONG_MAX;
	}

	if (sequence.duration_ms) {
		wait_ms = std::min(wait_ms, std::min(static_cast<unsigned long>(sequence.remaining_us / 1000UL), ULONG_MAX - 1));
	} else {
		wait_ms = std::min(wait_ms, ULONG_MAX);
	}

	colour = sequence.states[0].colour;

	render_time_us_ = esp_timer_get_time();
	render_event_ = event;
	lock.unlock();

	set_led(colour);

	return wait_ms;
}

void UserInterface::network_state(bool configured, NetworkState state) {
	std::lock_guard lock{mutex_};
	auto event = Event::IDLE;

	switch (state) {
	case NetworkState::DISCONNECTED:
		event = configured ? Event::NETWORK_CONFIGURED_DISCONNECTED
			: Event::NETWORK_UNCONFIGURED_DISCONNECTED;
		break;

	case NetworkState::CONNECTING:
		event = configured ? Event::NETWORK_CONFIGURED_CONNECTING
			: Event::NETWORK_UNCONFIGURED_CONNECTING;
		break;

	case NetworkState::CONNECTED:
		event = Event::NETWORK_CONNECTED;
		break;

	case NetworkState::FAILED:
		event = configured ? Event::NETWORK_CONFIGURED_FAILED
			: Event::NETWORK_UNCONFIGURED_FAILED;
		stop_event(Event::NETWORK_ERROR);
		break;
	}

	if (state == NetworkState::CONNECTED) {
		if (!event_active(event)) {
			restart_event(Event::NETWORK_CONNECT);
		}
	} else {
		stop_event(Event::NETWORK_CONNECT);
	}

	stop_events({
		Event::NETWORK_CONFIGURED_CONNECTING,
		Event::NETWORK_CONFIGURED_DISCONNECTED,
		Event::NETWORK_CONFIGURED_FAILED,
		Event::NETWORK_CONNECTED,
		Event::NETWORK_UNCONFIGURED_CONNECTING,
		Event::NETWORK_UNCONFIGURED_DISCONNECTED,
		Event::NETWORK_UNCONFIGURED_FAILED,
	});

	restart_event(event);
	wake_up();
}

void UserInterface::network_error() {
	std::lock_guard lock{mutex_};

	restart_event(Event::NETWORK_ERROR);
	wake_up();
}

void UserInterface::identify(uint16_t seconds) {
	ESP_LOGI(TAG, "Identify for %us", seconds);

	std::lock_guard lock{mutex_};

	stop_event(Event::IDENTIFY);
	if (seconds) {
		start_event(Event::IDENTIFY);
	}
	wake_up();
}

void UserInterface::ota_update(bool ok) {
	std::lock_guard lock{mutex_};

	restart_event(ok ? Event::OTA_UPDATE_OK : Event::OTA_UPDATE_ERROR);
	wake_up();
}

void UserInterface::door_opened() {
	std::lock_guard lock{mutex_};

	restart_event(Event::DOOR_OPENED);
	wake_up();
}

void UserInterface::door_closed() {
	std::lock_guard lock{mutex_};

	restart_event(Event::DOOR_CLOSED);
	wake_up();
}

void UserInterface::alarm(const door::Alarm &status) {
	std::lock_guard lock{mutex_};

	if (status.level != 1) {
		stop_event(Event::DOOR_ALARM1);
		alarm_level1_time_us_ = 0;
		alarm_level2_time_us_ = 0;
	}

	if (status.level < 2) {
		stop_event(Event::DOOR_ALARM2);
	}

	if (status.level >= 2) {
		if (!event_active(Event::DOOR_ALARM2)) {
			start_event(Event::DOOR_ALARM2);
		}
	} else if (status.level >= 1) {
		alarm_level1_time_us_ = status.open_time_us + status.level1_time_us;
		alarm_level2_time_us_ = alarm_level1_time_us_;
		if (status.level2_time_us > BUZZER_RAPID_DURATION_US) {
			alarm_level2_time_us_ += status.level2_time_us - BUZZER_RAPID_DURATION_US;
		}
		if (!event_active(Event::DOOR_ALARM1)) {
			start_event(Event::DOOR_ALARM1);
		}
	}
	wake_up();
}

void UserInterface::core_dump(bool present) {
	std::lock_guard lock{mutex_};

	stop_event(Event::CORE_DUMP_PRESENT);
	if (present) {
		start_event(Event::CORE_DUMP_PRESENT);
	}
	wake_up();
}

void UserInterface::buzzer_test() {
	std::lock_guard lock{mutex_};
	uint64_t now_us = esp_timer_get_time();

	buzzer_start_time_us_ = now_us;
	buzzer_stop_time_us_ = now_us + BUZZER_DURATION_US;
	buzzer_test_ = true;
	buzzer_.on();
	wake_up();
}

} // namespace octavo
