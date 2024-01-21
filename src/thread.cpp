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

#include "octavo/thread.h"

#include <esp_err.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <algorithm>

namespace octavo {

WakeupThread::WakeupThread(const char *name) : name_(name),
		semaphore_(xSemaphoreCreateBinary()) {
	if (!semaphore_) {
		ESP_LOGE(TAG, "Semaphore create for %s failed", name_);
		esp_restart();
	}

	esp_timer_create_args_t timer_config{};
	timer_config.callback = wake_up_timer;
	timer_config.arg = this;
	timer_config.dispatch_method = ESP_TIMER_TASK;
	timer_config.name = name_;

	ESP_ERROR_CHECK(esp_timer_create(&timer_config, &timer_));
}

void WakeupThread::run_loop() {
	while (true) {
		unsigned long wait_ms = std::max(1UL, run_tasks());

		if (wait_ms == ULONG_MAX) {
			esp_timer_stop(timer_);
		} else {
			if (esp_timer_restart(timer_, wait_ms * 1000U) != ESP_OK) {
				ESP_ERROR_CHECK(esp_timer_start_once(timer_, wait_ms * 1000U));
			}
		}

		xSemaphoreTake(semaphore_, portMAX_DELAY);
	}

	ESP_LOGE(TAG, "%s loop stopped", name_);
	esp_restart();
}

void WakeupThread::wake_up() {
	xSemaphoreGive(semaphore_);
}

void WakeupThread::wake_up_isr() {
	BaseType_t xHigherPriorityTaskWoken{pdFALSE};

	xSemaphoreGiveFromISR(semaphore_, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void WakeupThread::wake_up_timer(void *arg) {
	WakeupThread *wt = static_cast<WakeupThread*>(arg);

	if (wt) {
		xSemaphoreGive(wt->semaphore_);
	}
}

} // namespace octavo
