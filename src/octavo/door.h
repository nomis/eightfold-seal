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

#include <esp_zigbee_cluster.h>
#include <esp_zigbee_type.h>
#include <nvs.h>
#include <nvs_handle.hpp>
#include <driver/gpio.h>

#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "debounce.h"
#include "device.h"
#include "zigbee.h"

namespace octavo {

class Door;

namespace door {

class BooleanCluster: public ZigbeeCluster {
protected:
	BooleanCluster(Door &door, const char *name, uint16_t cluster_id,
		uint16_t attr_id);
	BooleanCluster(Door &door, const char *name, uint16_t cluster_id,
		uint16_t attr_id, const char *label, uint16_t app_usage);
	~BooleanCluster() = default;

public:
	void refresh();
	esp_err_t set_attr_value(uint16_t attr_id,
		const esp_zb_zcl_attribute_data_t *value) override;

protected:
	static constexpr const char *TAG = "octavo.Door";

	void configure_switch_cluster_list(esp_zb_cluster_list_t &cluster_list);
	void configure_binary_input_cluster_list(esp_zb_cluster_list_t &cluster_list);
	virtual bool refresh_value() = 0;
	virtual void updated_value(bool value) = 0;

	Door &door_;

private:
	const char *name_;
	const char *label_{"Unknown"};
	const uint16_t attr_id_;
	uint32_t app_type_{0};
	uint8_t state_;
};

class DoorStatusCluster: public BooleanCluster {
public:
	explicit DoorStatusCluster(Door &door);
	~DoorStatusCluster() = delete;

	void configure_cluster_list(esp_zb_cluster_list_t &cluster_list) override;

protected:
	bool refresh_value() override;
	void updated_value(bool value) override;
};

class AlarmStatusCluster: public BooleanCluster {
public:
	explicit AlarmStatusCluster(Door &door);
	~AlarmStatusCluster() = delete;

	void configure_cluster_list(esp_zb_cluster_list_t &cluster_list) override;

protected:
	bool refresh_value() override;
	void updated_value(bool value) override;
};

class AlarmEnableCluster: public BooleanCluster {
public:
	explicit AlarmEnableCluster(Door &door);
	~AlarmEnableCluster() = delete;

	void configure_cluster_list(esp_zb_cluster_list_t &cluster_list) override;

protected:
	bool refresh_value() override;
	void updated_value(bool value) override;
};

} // namespace door

class Door {
public:
	Door(uint8_t index, gpio_num_t switch_pin, bool switch_active_low);
	~Door() = delete;

	static constexpr const char *TAG = "octavo.Door";
	static constexpr const size_t NUM_EP_PER_DOOR = 5;

	inline uint8_t index() const { return index_; }

	void attach(Device &device);

	unsigned long run();

	bool open() const;
	uint8_t alarm_level() const;
	bool alarm_enable() const;

	void alarm_enable(bool state);

	void request_refresh();
	void refresh();

private:
	static constexpr const ep_id_t DOOR_STATUS_BASE_EP_ID = 10;
	static constexpr const ep_id_t ALARM_STATUS_BASE_EP_ID = 20;
	static constexpr const ep_id_t ALARM_ENABLE_BASE_EP_ID = 30;
	static constexpr const ep_id_t ALARM_TIME1_BASE_EP_ID = 40;
	static constexpr const ep_id_t ALARM_TIME2_BASE_EP_ID = 50;
	static constexpr const unsigned long DEBOUNCE_US = 20 * 1000;
	static std::unique_ptr<nvs::NVSHandle> nvs_;

	bool open_nvs();
	bool enable_nvs();
	void enable_nvs(bool state);

	void open(bool state);

	void update_state();

	const uint8_t index_;
	Debounce switch_debounce_;

	mutable std::mutex mutex_;
	bool switch_active_;
	uint64_t switch_change_us_{0};
	uint8_t alarm_level_{0};
	bool alarm_enable_{true};

	door::DoorStatusCluster &door_status_cl_;
	door::AlarmStatusCluster &alarm_status_cl_;
	door::AlarmEnableCluster &alarm_enable_cl_;

	Device *device_{nullptr};
};

} // namespace octavo
