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

#include "octavo/door.h"

#include <esp_err.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_zigbee_cluster.h>
#include <esp_zigbee_type.h>
#include <nvs.h>
#include <nvs_handle.hpp>
#include <driver/gpio.h>
#include <ha/esp_zigbee_ha_standard.h>

#include <cinttypes>
#include <cmath>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "octavo/debounce.h"
#include "octavo/device.h"
#include "octavo/ui.h"
#include "octavo/util.h"

using namespace std::chrono_literals;

namespace octavo {

std::unique_ptr<nvs::NVSHandle> Door::nvs_;

Door::Door(uint8_t index, gpio_num_t switch_pin, bool switch_active_low)
		: index_(index), switch_debounce_(switch_pin, switch_active_low, DEBOUNCE_US),
		switch_active_(switch_debounce_.value()),
		alarm_enable_(enable_nvs()),
		alarm_time1_us_(alarm_time1_us_nvs()),
		alarm_time2_us_(alarm_time2_us_nvs()),
		door_status_cl_(*new door::DoorStatusCluster{*this}),
		alarm_status_cl_(*new door::AlarmStatusCluster{*this}),
		alarm_enable_cl_(*new door::AlarmEnableCluster{*this}),
		alarm_cancel_cl_(*new door::AlarmCancelCluster{*this}),
		alarm_time1_cl_(*new door::AlarmTime1Cluster{*this}),
		alarm_time2_cl_(*new door::AlarmTime2Cluster{*this}) {
	ESP_LOGD(TAG, "Door %u switch is %d", index_, switch_active_);
}

bool Door::open_nvs() {
	if (!nvs_) {
		nvs_ = nvs::open_nvs_handle("door", NVS_READWRITE, nullptr);
	}

	return !!nvs_;
}

bool Door::enable_nvs() {
	if (open_nvs()) {
		uint8_t value = 1;

		if (nvs_->get_item(("e" + std::to_string(index_)).c_str(), value) == ESP_OK) {
			return value != 0;
		}
	}

	return true;
}

void Door::enable_nvs(bool state) {
	if (open_nvs()) {
		uint8_t value = state ? 1 : 0;

		nvs_->set_item(("e" + std::to_string(index_)).c_str(), value);
		nvs_->commit();
	}
}

uint64_t Door::alarm_time1_us_nvs() {
	if (open_nvs()) {
		uint64_t value = 1;

		if (nvs_->get_item(("a" + std::to_string(index_)).c_str(), value) == ESP_OK) {
			return constrain_alarm_time_us(value);
		}
	}

	return std::chrono::microseconds{std::chrono::seconds{CONFIG_OCTAVO_DEFAULT_ALARM_TIME1_S}}.count();
}

void Door::alarm_time1_us_nvs(uint64_t value) {
	if (open_nvs()) {
		nvs_->set_item(("a" + std::to_string(index_)).c_str(), value);
		nvs_->commit();
	}
}

uint64_t Door::alarm_time2_us_nvs() {
	if (open_nvs()) {
		uint64_t value = 1;

		if (nvs_->get_item(("b" + std::to_string(index_)).c_str(), value) == ESP_OK) {
			return constrain_alarm_time_us(value);
		}
	}

	return std::chrono::microseconds{std::chrono::seconds{CONFIG_OCTAVO_DEFAULT_ALARM_TIME2_S}}.count();
}

void Door::alarm_time2_us_nvs(uint64_t value) {
	if (open_nvs()) {
		nvs_->set_item(("b" + std::to_string(index_)).c_str(), value);
		nvs_->commit();
	}
}

void Door::attach(Device &device) {
	device_ = &device;
	device.add(*this, {
		*new ZigbeeEndpoint{
			static_cast<ep_id_t>(DOOR_STATUS_BASE_EP_ID + index_),
			ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
			{
				door_status_cl_,
			}},
		*new ZigbeeEndpoint{
			static_cast<ep_id_t>(ALARM_STATUS_BASE_EP_ID + index_),
			ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
			{
				alarm_status_cl_,
			}},
		*new ZigbeeEndpoint{
			static_cast<ep_id_t>(ALARM_ENABLE_BASE_EP_ID + index_),
			ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
			{
				alarm_enable_cl_,
			}},
		*new ZigbeeEndpoint{
			static_cast<ep_id_t>(ALARM_TIME1_BASE_EP_ID + index_),
			ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
			{
				alarm_time1_cl_,
			}},
		*new ZigbeeEndpoint{
			static_cast<ep_id_t>(ALARM_TIME2_BASE_EP_ID + index_),
			ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
			{
				alarm_time2_cl_,
			}},
		*new ZigbeeEndpoint{
			static_cast<ep_id_t>(ALARM_CANCEL_BASE_EP_ID + index_),
			ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
			{
				alarm_cancel_cl_,
			}},
	});

	switch_debounce_.start(device);
}

unsigned long Door::run() {
	DebounceResult debounce = switch_debounce_.run();

	if (debounce.changed) {
		if (switch_debounce_.first()) {
			std::lock_guard lock{mutex_};
			bool state = switch_debounce_.value();

			switch_active_ = state;
			switch_change_us_ = esp_timer_get_time();
			if (!state) {
				alarm_cancel_ = false;
			}
			request_refresh();
		} else {
			open(switch_debounce_.value());
		}
	}

	return debounce.wait_ms;
}

bool Door::open() const {
	std::lock_guard lock{mutex_};
	return switch_active_;
}

void Door::open(bool state) {
	std::lock_guard lock{mutex_};

	ESP_LOGD(TAG, "Door %u switch %d -> %d", index_, switch_active_, state);

	if (switch_active_ != state) {
		uint64_t now_us = esp_timer_get_time();

		ESP_LOGD(TAG, "Door %u switch was %s for %s",
			index_, switch_active_ ? "open" : "closed",
			duration_us_to_string(now_us - switch_change_us_).c_str());

		switch_active_ = state;
		switch_change_us_ = now_us;
	}

	if (alarm_cancel_ && !state) {
		ESP_LOGD(TAG, "Door %u alarm cancel %d -> %d (door closed)",
			index_, alarm_cancel_, false);
		alarm_cancel_ = false;
	}

	update_state();

	if (state) {
		device_->ui().door_opened();
	} else {
		device_->ui().door_closed();
	}
}

uint8_t Door::alarm_level() const {
	std::lock_guard lock{mutex_};
	return alarm_level_;
}

bool Door::alarm_enable() const {
	std::lock_guard lock{mutex_};
	return alarm_enable_;
}

void Door::alarm_enable(bool state) {
	std::lock_guard lock{mutex_};

	ESP_LOGD(TAG, "Door %u set alarm enable %d -> %d",
		index_, alarm_enable_, state);
	enable_nvs(state);
	alarm_enable_ = state;
	update_state();
}

bool Door::alarm_cancel() const {
	std::lock_guard lock{mutex_};
	return alarm_cancel_;
}

void Door::alarm_cancel(bool state) {
	std::lock_guard lock{mutex_};

	ESP_LOGD(TAG, "Door %u set alarm cancel %d -> %d",
		index_, alarm_cancel_, state);
	alarm_cancel_ = state;

	if (alarm_cancel_ && !switch_active_) {
		ESP_LOGD(TAG, "Door %u alarm cancel %d -> %d (door not open)",
			index_, alarm_cancel_, false);
		alarm_cancel_ = false;
	}
	update_state();
}

uint64_t Door::alarm_time1_us() const {
	std::lock_guard lock{mutex_};
	return alarm_time1_us_;
}

void Door::alarm_time1_us(uint64_t value) {
	std::lock_guard lock{mutex_};

	value = constrain_alarm_time_us(value);

	ESP_LOGD(TAG, "Door %u set alarm time 1 %" PRIu64 " -> %" PRIu64,
		index_, alarm_time1_us_, value);
	alarm_time1_us_nvs(value);
	alarm_time1_us_ = value;
	update_state();
}

uint64_t Door::alarm_time2_us() const {
	std::lock_guard lock{mutex_};
	return alarm_time2_us_;
}

void Door::alarm_time2_us(uint64_t value) {
	std::lock_guard lock{mutex_};

	value = constrain_alarm_time_us(value);

	ESP_LOGD(TAG, "Door %u set alarm time 2 %" PRIu64 " -> %" PRIu64,
		index_, alarm_time2_us_, value);
	alarm_time2_us_nvs(value);
	alarm_time2_us_ = value;
	update_state();
}

void Door::update_state() {
	/* TODO */
	request_refresh();
}

void Door::request_refresh() {
	device_->request_refresh(*this);
}

void Door::refresh() {
	door_status_cl_.refresh();
	alarm_status_cl_.refresh();
	alarm_time1_cl_.refresh();
	alarm_time2_cl_.refresh();
	alarm_cancel_cl_.refresh();
}

namespace door {

BooleanCluster::BooleanCluster(Door &door, const char *name,
		uint16_t cluster_id, uint16_t attr_id) : ZigbeeCluster(cluster_id,
			ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, {attr_id}), door_(door),
			name_(name), attr_id_(attr_id) {

}

BooleanCluster::BooleanCluster(Door &door, const char *name,
		uint16_t cluster_id, uint16_t attr_id, const char *label,
		uint16_t app_usage) : ZigbeeCluster(cluster_id,
			ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, {attr_id}), door_(door),
			name_(name), label_(label), attr_id_(attr_id), app_type_(
	  			  (  0x03 << 24)  /* Group: Binary Input            */
				| (  0x00 << 16)  /* Type:  Application Domain HVAC */
				|  app_usage      /* Index                          */
			) {
}

void BooleanCluster::configure_switch_cluster_list(esp_zb_cluster_list_t &cluster_list) {
	esp_zb_on_off_cluster_cfg_t switch_cfg{};
	switch_cfg.on_off = (state_ = refresh_value() ? 1 : 0);

	ESP_ERROR_CHECK(esp_zb_cluster_list_add_on_off_cluster(&cluster_list,
		esp_zb_on_off_cluster_create(&switch_cfg), role()));
}

void BooleanCluster::configure_binary_input_cluster_list(esp_zb_cluster_list_t &cluster_list) {
	esp_zb_attribute_list_t *input_cluster = esp_zb_binary_input_cluster_create(nullptr);

	state_ = refresh_value() ? 1 : 0;

	ESP_ERROR_CHECK(esp_zb_binary_input_cluster_add_attr(input_cluster,
			ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID, &state_));

	ESP_ERROR_CHECK(esp_zb_binary_input_cluster_add_attr(input_cluster,
			ESP_ZB_ZCL_ATTR_BINARY_INPUT_APPLICATION_TYPE_ID, &app_type_));

	ESP_ERROR_CHECK(esp_zb_binary_input_cluster_add_attr(input_cluster,
			ESP_ZB_ZCL_ATTR_BINARY_INPUT_DESCRIPTION_ID,
			ZigbeeString(std::string{label_} + " " + std::to_string(door_.index())).data()));

	ESP_ERROR_CHECK(esp_zb_cluster_list_add_binary_input_cluster(&cluster_list,
		input_cluster, role()));
}

void BooleanCluster::refresh() {
	uint8_t new_state = refresh_value() ? 1 : 0;

	if (new_state != state_) {
		state_ = new_state;
		ESP_LOGD(TAG, "Door %u report %s %u", door_.index(), name_, state_);

		update_attr_value(attr_id_, &state_);
	}
}

esp_err_t BooleanCluster::set_attr_value(uint16_t attr_id,
		const esp_zb_zcl_attribute_data_t *data) {
	if (attr_id == attr_id_) {
		if (data->type == ESP_ZB_ZCL_ATTR_TYPE_BOOL
				&& data->size == sizeof(uint8_t)) {
			state_ = *(uint8_t *)data->value;
			updated_value(state_);
			return ESP_OK;
		}
	}
	return ESP_ERR_INVALID_ARG;
}

DoorStatusCluster::DoorStatusCluster(Door &door)
		: BooleanCluster(door, "door state",
			ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT,
			ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID,
			"Door", 0x0026 /* Index: Door Status BI */) {
}

void DoorStatusCluster::configure_cluster_list(esp_zb_cluster_list_t &cluster_list) {
	configure_binary_input_cluster_list(cluster_list);
}

bool DoorStatusCluster::refresh_value() {
	return door_.open();
}

void DoorStatusCluster::updated_value(bool state) {
	door_.refresh();
}

AlarmStatusCluster::AlarmStatusCluster(Door &door)
		: BooleanCluster(door, "alarm state",
			ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT,
			ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID,
			"Alarm", 0x0019 /* Index: Cooling Alarm BI */) {
}

void AlarmStatusCluster::configure_cluster_list(esp_zb_cluster_list_t &cluster_list) {
	configure_binary_input_cluster_list(cluster_list);
}

bool AlarmStatusCluster::refresh_value() {
	return door_.alarm_level() > 0;
}

void AlarmStatusCluster::updated_value(bool state) {
	door_.refresh();
}

AlarmEnableCluster::AlarmEnableCluster(Door &door)
		: BooleanCluster(door, "alarm enable",
			ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID) {
}

void AlarmEnableCluster::configure_cluster_list(esp_zb_cluster_list_t &cluster_list) {
	configure_switch_cluster_list(cluster_list);
}

bool AlarmEnableCluster::refresh_value() {
	return door_.alarm_enable();
}

void AlarmEnableCluster::updated_value(bool state) {
	door_.alarm_enable(state);
}

AlarmCancelCluster::AlarmCancelCluster(Door &door)
		: BooleanCluster(door, "alarm cancel",
			ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID) {
}

void AlarmCancelCluster::configure_cluster_list(esp_zb_cluster_list_t &cluster_list) {
	configure_switch_cluster_list(cluster_list);
}

bool AlarmCancelCluster::refresh_value() {
	return door_.alarm_cancel();
}

void AlarmCancelCluster::updated_value(bool state) {
	door_.alarm_cancel(state);
}

AnalogCluster::AnalogCluster(Door &door, const char *name,
		uint16_t cluster_id, uint16_t attr_id, const char *label_prefix,
		const char *label_suffix, uint32_t app_type, uint16_t eng_units,
		float min_value, float max_value, float resolution)
			: ZigbeeCluster(cluster_id, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
			{attr_id}), door_(door), name_(name), label_prefix_(label_prefix),
			label_suffix_(label_suffix), attr_id_(attr_id), app_type_(app_type),
			eng_units_(eng_units), min_value_(min_value),
			max_value_(max_value), resolution_(resolution) {
}

void AnalogCluster::configure_analog_output_cluster_list(esp_zb_cluster_list_t &cluster_list) {
	esp_zb_attribute_list_t *output_cluster = esp_zb_analog_output_cluster_create(nullptr);

	state_ = refresh_value();

	ESP_ERROR_CHECK(esp_zb_cluster_update_attr(output_cluster,
			ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID, &state_));

	ESP_ERROR_CHECK(esp_zb_analog_output_cluster_add_attr(output_cluster,
			ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_MIN_PRESENT_VALUE_ID, &min_value_));

	ESP_ERROR_CHECK(esp_zb_analog_output_cluster_add_attr(output_cluster,
			ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_MAX_PRESENT_VALUE_ID, &max_value_));

	ESP_ERROR_CHECK(esp_zb_analog_output_cluster_add_attr(output_cluster,
			ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_RESOLUTION_ID, &resolution_));

	ESP_ERROR_CHECK(esp_zb_analog_output_cluster_add_attr(output_cluster,
			ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_APPLICATION_TYPE_ID, &app_type_));

	ESP_ERROR_CHECK(esp_zb_analog_output_cluster_add_attr(output_cluster,
			ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_ENGINEERING_UNITS_ID, &eng_units_));

	ESP_ERROR_CHECK(esp_zb_analog_output_cluster_add_attr(output_cluster,
			ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_DESCRIPTION_ID,
			ZigbeeString(std::string{label_prefix_} + " "
				+ std::to_string(door_.index()) + " " + label_suffix_).data()));

	ESP_ERROR_CHECK(esp_zb_cluster_list_add_analog_output_cluster(&cluster_list,
		output_cluster, role()));
}

void AnalogCluster::refresh() {
	uint64_t new_value = refresh_value();

	if (new_value != state_) {
		state_ = new_value;
		ESP_LOGD(TAG, "Door %u report %s %.6f", door_.index(), name_, state_);

		update_attr_value(attr_id_, &state_);
	}
}

esp_err_t AnalogCluster::set_attr_value(uint16_t attr_id,
		const esp_zb_zcl_attribute_data_t *data) {
	if (attr_id == attr_id_) {
		if (data->type == ESP_ZB_ZCL_ATTR_TYPE_SINGLE
				&& data->size == sizeof(float)) {
			state_ = *(float *)data->value;
			updated_value(state_);
			return ESP_OK;
		}
	}
	return ESP_ERR_INVALID_ARG;
}

AlarmTime1Cluster::AlarmTime1Cluster(Door &door)
		: AnalogCluster(door, "alarm time1",
			ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT,
			ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID,
			"Alarm", "Time 1",
			  (  0x01 << 24)  /* Group: Analog Output    */
			| (  0x0E << 16)  /* Type:  Time in Seconds */
			|  0x0000         /* Index: Relative time   */,
			       73         /* Time - Days */,
			Door::MIN_ALARM_TIME_S.count(), Door::MAX_ALARM_TIME_S.count(), 1) {
}

void AlarmTime1Cluster::configure_cluster_list(esp_zb_cluster_list_t &cluster_list) {
	configure_analog_output_cluster_list(cluster_list);
}

float AlarmTime1Cluster::refresh_value() {
	return static_cast<float>(door_.alarm_time1_us()) / std::chrono::microseconds(1s).count();
}

void AlarmTime1Cluster::updated_value(float state) {
	door_.alarm_time1_us(std::nearbyintf(state * std::chrono::microseconds(1s).count()));
}

AlarmTime2Cluster::AlarmTime2Cluster(Door &door)
		: AnalogCluster(door, "alarm time2",
			ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT,
			ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID,
			"Alarm", "Time 2",
			  (  0x01 << 24)  /* Group: Analog Output    */
			| (  0x0E << 16)  /* Type:  Time in Seconds */
			|  0x0000         /* Index: Relative time   */,
			       73         /* Time - Days */,
			Door::MIN_ALARM_TIME_S.count(), Door::MAX_ALARM_TIME_S.count(), 1) {
}

void AlarmTime2Cluster::configure_cluster_list(esp_zb_cluster_list_t &cluster_list) {
	configure_analog_output_cluster_list(cluster_list);
}

float AlarmTime2Cluster::refresh_value() {
	return static_cast<float>(door_.alarm_time2_us()) / std::chrono::microseconds(1s).count();
}

void AlarmTime2Cluster::updated_value(float state) {
	door_.alarm_time2_us(std::nearbyintf(state * std::chrono::microseconds(1s).count()));
}

} // namespace door

} // namespace octavo
