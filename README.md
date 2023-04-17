# PowMr Inverter serial communication protocol 

Would you like to monitor and control your PowMr solar hybrid inverter via MQTT?

Here is the reverse engineering of the communication protocol. The infomation might be incomplete, feel free to contribute. All communications has been sniffed out of the original Wifi dongle.

Applicable to POWMR POW-HVM3.2H-24V inverter series.
- user manual: https://powmr.com/index.php?dispatch=attachments.getfile&attachment_id=368

The device communicates with RS232 modbus protocol at 2400 baud (1 stop bit, no parity control).
PowMr device slave id is 5.

RJ45 connector is located on the left side of the inverter has the following pinout:

- pin 8 (Brown)          Ground 
- pin 4 (Blue)           +12V
- pin 2 (Orange)         RX (data to inverter)
- pin 1 (White Orange)   TX (data from inverter)


Modbus Holding registers to control inverter:
- 5017 charger source priority (range 0-3, settings menu 16)
- 5018 output source priority (range 0-2, settings menu 1)
- 5024 utility charge current (one of 2, 10, 20, 30, 40, 50, 60, settings menu 11)
- 5002 buzzer alarm (range 0-1, settings menu 18)
- 5007 beeps when primary source interrupted (range 0-1, settings menu 22)
- 5009 transfer to bypass on overload (0-1, settings menu 23)
- 5022 max total charge current (range 10-80, settings menu 2)
- 5025 comeback utility mode voltage (SBU) (0.5 volts step, settings menu 12) 
- 5026 comeback battery mode voltage (SBU) (0.5 volts step, settings menu 13)


Original WiFi dongle sends two requests every few seconds to monitor inverter state. 
- read 45 registers (func 3) from slave 5 starting from address 4501 (Decimal) (raw: 05031195002d9143)
- read 16 registers (func 3) from slave 5 starting from address 4546 (Decimal) (raw: 050311c20010e142)

Please refer wifi bridge code for state registers description.

