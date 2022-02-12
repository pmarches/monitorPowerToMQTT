What is this?
==

This project reads the victron energy MPPT bluetooth packets and sends them to a MQTT broker. It also reads the mastervolt data from the masterbus and forwards it to the same MQTT broker.

This is the first open source attempt at documenting the VE.Smart bluetooth protocol. If enough interest, we should make it a stand alone library.

Installation
==
This project runs on ESP32 and is built with ESP-IDF. The installation steps are the standard ESP-IDF steps.

	idf.py menuconfig
	idf.py flash monitor


TODO
==
- Get the AES key from the MPPT devices instead the klunky app hack. We will need to connect to the MPPT, send the PIN, issue some commands to get the AES key. Is that even possible?
- Reduce power usage because we seem to draw too much for the masterbus
  - Reduce BLE power usage esp_bredr_tx_power_set
  - Go to sleep when solar panels are off
  - Turn off the micro controller LED to save power
  
- Add support for mDNS lookup of the MQTT broker
- Augment the MPPT data with stats such as max yield, min/max battery voltage
- Make the values more human readable instead of the basic registers? Maybe put the registers in one topic, and the human readable in other topics?
- Add OTA updates.Thru MQTT or direct socket connection?
- Add remote logging

Data augmentation
==
- Min/Max/Median/Avg
- Network 
  - yield (min/max/median) over multiple days
  - Lifetime yield
- How much time was spent in negative yield? (Too much load)
- Daily battery consumption
- Daily solar consumption (yield)
- At what time 
  - did the solar start producing?
  - was the production greater or equal to the consumption?
  - did the solar stop producing?
- History
- Does one MPPT produce less than it should, indicating a problem?

Some of these require long term data storage. Is this project the right layer?

MQTT Schema
===========

    /venetworking/MPPT_ID/
        0100  HEX Model type, capabilities and so on. 62A0==150|60, ...
        0102  HEX The firmware version in HEX
        2000  Only set on the mater mppt, 10 indicates master status?
        2001  maybe target charge voltage. Does not change quickly, only on master device. Or Maybe max batt voltage? Seen 1419 1420 1424
        2007  Yield Network watts produced? Only seen on master
        2008  Yield MPPT today
        200b  seen 1219 1229 1242 1246, only on master. Looks slow monotonic. Maybe day number?
        200c  Values seen: 3, 5 only on master (Maybe bulk/Float/Abs mode?)
        2027  Network total power (CentiWatt)
        ed8c  MPPT output amps on battery side (CentiAmps)
        edbc  MPPT output (CentiWatt)
        edef  Always seen 12. Maybe the 12v settings?
        ec3e  probably true/false. Seen 1. (Not charger enabled flag, not load output)
        
    /masterbus/0x31297/
        0x00  DC Shunt Percent charge
        0x01  DC Shunt voltage
        0x02  DC Shunt Amps flow
        0x03  DC Shunt Consumed amps

    /masterbus/0x2F412/
        0x0B  Inverter 120v amp flow
        0x14  Inverter state

Issue
---
When Wifi connection is lost, it somehow affects the BLE scanning. If we try to re-start the scanning before the Wifi is connected, the wifi is unable to connect.
Maybe send all events back to the main app for handling? Implement a FMS? Turns, out Wifi and BLE timeshare the same RF system. So either BLE is receiving, or Wifi is TXing. 
Possible solutions: 
1- Tune the co-existance parameters. Ensure the BLE scanning gets restarted on a regular basis?
2- Use the veDirect HEX protocol with the serial multiplexer. Am worried about galvanic isolation. Also I got noise problems, as shown with failing checksums.
3- Disable Software Wifi/BLE coexistance and implement hardware control. We would listen on BLE for a while, accumulate results, switch to Wifi (using GPIO pins?) send results. 
  `