#!/bin/bash
if [ -z "$2" ] || [ -z "$3" ] || [ -z "$4" ]; then
	echo "Usage: $0 <old suffix|-> <new id> <new name> <door> [door] [door] [door]"
	echo
	echo "Example: $0 uuid_uk_eightfold_seal eightfold_seal_1 \"Eightfold Seal 1\" 1 2"
	exit 1
fi
OLD="$1"
NEW="$2"
NAME="$3"
[ "$OLD" = "-" ] && OLD=""
shift 3
DOORS=("$@")

function generate_file() {
	echo "{"

	binary_sensor=1
	switch=1
	number=1

	i=0
	while [ $i -lt ${#DOORS[@]} ]; do
		n=$(($i + 1))

		id="_${binary_sensor}"
		[ $binary_sensor -eq 1 ] && id=""
		old="binary_sensor.${OLD}_binaryinput${id}"
		new="binary_sensor.${NEW}_door_${n}"
		[ -n "$OLD" ] || old="$new"
		echo "  \"${old}\": [\"${new}\", \"${NAME} Door ${DOORS[$i]}\"],"
		binary_sensor=$(($binary_sensor + 1))

		id="_${switch}"
		[ $switch -eq 1 ] && id=""
		old="switch.${OLD}_switch${id}"
		new="switch.${NEW}_alarm_enable_${n}"
		[ -n "$OLD" ] || old="$new"
		echo "  \"${old}\": [\"${new}\", \"${NAME} Alarm ${DOORS[$i]} Enable\"],"
		switch=$(($switch + 1))

		id="_${number}"
		old="number.${OLD}_number_alarm${id}_time_1"
		new="number.${NEW}_alarm_${n}_time_1"
		[ -n "$OLD" ] || old="$new"
		echo "  \"${old}\": [\"${new}\", \"${NAME} Alarm ${DOORS[$i]} Time 1\"],"
		old="number.${OLD}_number_alarm${id}_time_2"
		new="number.${NEW}_alarm_${n}_time_2"
		[ -n "$OLD" ] || old="$new"
		echo "  \"${old}\": [\"${new}\", \"${NAME} Alarm ${DOORS[$i]} Time 2\"],"
		number=$(($number + 1))

		i=$n
	done

	i=0
	while [ $i -lt ${#DOORS[@]} ]; do
		n=$(($i + 1))

		id="_${binary_sensor}"
		[ $binary_sensor -eq 1 ] && id=""
		old="binary_sensor.${OLD}_binaryinput${id}"
		new="binary_sensor.${NEW}_alarm_${n}"
		[ -n "$OLD" ] || old="$new"
		echo "  \"${old}\": [\"${new}\", \"${NAME} Alarm ${DOORS[$i]}\"],"
		binary_sensor=$(($binary_sensor + 1))

		id="_${switch}"
		[ $switch -eq 1 ] && id=""
		old="switch.${OLD}_switch${id}"
		new="switch.${NEW}_alarm_cancel_${n}"
		[ -n "$OLD" ] || old="$new"
		echo "  \"${old}\": [\"${new}\", \"${NAME} Alarm ${DOORS[$i]} Cancel\"],"
		switch=$(($switch + 1))

		i=$n
	done

	old="button.${OLD}_identify"
	new="button.${NEW}_identify"
	[ -n "$OLD" ] || old="$new"
	echo "  \"${old}\": [\"${new}\", \"${NAME} Identify\"],"

	old="sensor.${OLD}_analoginput"
	new="sensor.${NEW}_uptime"
	[ -n "$OLD" ] || old="$new"
	echo "  \"${old}\": [\"${new}\", \"${NAME} Uptime (days)\"],"

	old="sensor.${OLD}_analoginput_2"
	new="sensor.${NEW}_connected_time"
	[ -n "$OLD" ] || old="$new"
	echo "  \"${old}\": [\"${new}\", \"${NAME} Connected time (days)\"],"

	old="sensor.${OLD}_analoginput_3"
	new="sensor.${NEW}_uplink_address"
	[ -n "$OLD" ] || old="$new"
	echo "  \"${old}\": [\"${new}\", \"${NAME} Uplink address\"],"

	old="sensor.${OLD}_analoginput_4"
	new="sensor.${NEW}_uplink_rssi"
	[ -n "$OLD" ] || old="$new"
	echo "  \"${old}\": [\"${new}\", \"${NAME} Uplink RSSI\"],"

	old="sensor.${OLD}_lqi"
	new="sensor.${NEW}_lqi"
	[ -n "$OLD" ] || old="$new"
	echo "  \"${old}\": [\"${new}\", \"${NAME} LQI\"],"

	old="sensor.${OLD}_rssi"
	new="sensor.${NEW}_rssi"
	[ -n "$OLD" ] || old="$new"
	echo "  \"${old}\": [\"${new}\", \"${NAME} RSSI\"]"

	echo "}"
}

pipenv run ./homeassistant-entity-renamer.py --file <(generate_file)
