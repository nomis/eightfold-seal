Eightfold Seal
==============

ESP32 Zigbee door open alarm.

Supports up to 4 doors and one buzzer.

Requires an ESP32-H2 or ESP32-C6.

    The walls were covered with occult symbols, and most of the floor was taken
    up by the Eightfold Seal of Stasis, generally agreed in magical circles to
    have all the stopping power of a well-aimed halfbrick.

    The only furnishing in the room was a lectern of dark wood, carved into the
    shape of a bird -- well, to be frank, into the shape of a winged thing it is
    probably best not to examine too closely -- and on the lectern, fastened to
    it by a heavy chain covered in padlocks, was a book.

    ...

    The air in the room was now sparkling with tiny flashes as dust motes
    incinerated in the flow of raw magic. The Seal of Stasis was beginning to
    blister and curl up at the edges. The book in question was called the Octavo
    and, quite obviously, it was no ordinary book.

    -- `Terry Pratchett <https://en.wikipedia.org/wiki/Terry_Pratchett>`_
    (`The Light Fantastic, 1986 <https://en.wikipedia.org/wiki/The_Light_Fantastic>`_)


Usage
-----

Use a push button on GPIO4 to GND (or use the UART command) to join/leave the
Zigbee network. Joining a new network is not performed automatically. Leaving
the network currently requires a restart.

Each door has a switch GPIO and presents the following Zigbee on/off clusters:

* Door Status (Binary Input)
* Alarm Status (Binary Input)
* Alarm Enable Switch
* Alarm Time 1 (Analog Output)
* Alarm Time 2 (Analog Output)
* Alarm Cancel Switch

The buzzer GPIO will be activated when either door is left open for longer than
the configured alarm times. After **Alarm Time 1** a warning beep will sound,
increasing in frequency until the **Alarm Time 2** expires when it will be
continuously active.

The **Alarm Cancel Switch** is used to dismiss the alarm until the door is
closed without needing to disable the alarm completely.

Status
~~~~~~

The following Zigbee analog input clusters report the status of the device:

* Uptime (days)
* Connected time (days)
* Uplink address
* Uplink RSSI (dB)

When a core dump is present the **Uptime (days)** cluster will report a fault
in the status flag with a reliability attribute value of "unreliable other".

LED Events
~~~~~~~~~~

.. list-table::
   :widths: 50 50
   :header-rows: 1

   * - Colours
     - Description
   * - White
     - Disconnected (network not configured)
   * - White (blinking)
     - Connecting (network not configured)
   * - Yellow
     - Disconnected (network configured)
   * - Yellow (blinking)
     - Connecting (network configured)
   * - Green (constant then blinking every 3 seconds)
     - Network connected
   * - Red (blinking 2 times for 1 second)
     - Network error
   * - Red
     - Network failed (network configured)
   * - Red (blinking)
     - Network failed (network not configured)
   * - Orange (for 2 seconds)
     - Door opened
   * - Blue (for 2 seconds)
     - Door closed
   * - Orange (blinking every second)
     - Door alarm (level 1)
   * - Orange/White (alternate blinking 2 times every second)
     - Door alarm (level 2)
   * - Magenta
     - Identify request received
   * - Cyan
     - OTA update in progress
   * - Red (blinking 8 times in 3 seconds)
     - OTA update error
   * - Rainbow (cycling)
     - Core dump present

UART Commands
~~~~~~~~~~~~~

.. list-table::
   :widths: 15 85
   :header-rows: 1

   * - Keys
     - Description
   * - ``0``
     - Disable logging (persistent)
   * - ``1``\ ..\ ``5``
     - Set application log level to ERROR..VERBOSE (persistent)
   * - ``6``\ ..\ ``9``
     - Set system log level to ERROR..DEBUG (persistent)
   * - ``b``
     - Print cluster binding table
   * - ``j``
     - Join Zigbee network (no effect if already joined/joining)
   * - ``l``
     - Leave Zigbee network (no effect if already left)
   * - ``m``
     - Print memory information
   * - ``n``
     - Print Zigbee neighbours
   * - ``t``
     - Print task list and stats
   * - ``R``
     - Restart
   * - ``C``
     - Crash (used for testing to generate a core dump)
   * - ``d``
     - Print brief core dump summary
   * - ``D``
     - Print whole core dump
   * - ``E``
     - Erase saved core dump

Build
-----

This project can be built with the `ESP-IDF build system
<https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html>`_.

Configure::

    idf.py set-target esp32c6
    idf.py menuconfig

Under "Component config" you'll find "Eightfold Seal" where you can configure
the number of doors supported and whether switches/buzzers are active low or not.

The GPIO configuration assumes you're using an `ESP32-C6-DevKitC-1
<https://docs.espressif.com/projects/espressif-esp-dev-kits/en/latest/esp32c6/esp32-c6-devkitc-1/>`_.

Build::

    idf.py build

Flash::

    idf.py flash


Help
----

What order are all the entities shown in Home Assistant?
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The Zigbee specifications are thousands of pages long and it supports 240
endpoints per device but there's no attribute to describe on/off clusters if
you have more than one of the same type!

Using `this version of homeassistant-entity-renamer
<https://github.com/nomis/homeassistant-entity-renamer>`_ that can update
the friendly names (so that they're not all "Binaryinput" and "Switch") and the
`hass-rename-entities.sh script <hass-rename-entities.sh>`_ you can rename
all of the entities automatically.

The control cluster endpoints are in the following order:

.. list-table::
   :widths: 20 10 70
   :header-rows: 1

   * - Type
     - Endpoint
     - Name
   * - Number
     - 41
     - Alarm 1 Time 1
   * - Number
     - 51
     - Alarm 1 Time 2
   * - ⋮
     - ⋮
     - ⋮
   * - Number
     - 4n
     - Alarm N Time 1
   * - Number
     - 5n
     - Alarm N Time 2
   * - Switch
     - 31
     - Alarm Enable 1
   * - ⋮
     - ⋮
     - ⋮
   * - Switch
     - 3n
     - Alarm Enable N
   * - Switch
     - 61
     - Alarm Cancel 1
   * - ⋮
     - ⋮
     - ⋮
   * - Switch
     - 6n
     - Alarm Cancel N

The sensor cluster endpoints are in the following order:

.. list-table::
   :widths: 20 10 70
   :header-rows: 1

   * - Type
     - Endpoint
     - Name
   * - Analoginput
     - 1
     - Uptime (days)
   * - Analoginput
     - 210
     - Connected time (days)
   * - Analoginput
     - 211
     - Uplink address
   * - Analoginput
     - 212
     - Uplink RSSI (dB)
   * - Binaryinput
     - 11
     - Door Status 1
   * - ⋮
     - ⋮
     - ⋮
   * - Binaryinput
     - 1n
     - Door Status N
   * - Binaryinput
     - 21
     - Alarm Status 1
   * - ⋮
     - ⋮
     - ⋮
   * - Binaryinput
     - 2n
     - Alarm Status N
