What is this?
==

This project reads the victron energy MPPT bluetooth packets and sends them to a MQTT broker. 

This is the first open source attempt at documenting the VE.Smart bluetooth protocol. If enough interest, we should make it a stand alone library.

Installation
==
This project runs on ESP32 and is built with ESP-IDF. The installation steps are the standard ESP-IDF steps.

	idf.py menuconfig
	idf.py flash monitor


TODO
==
- Get the AES key from the MPPT devices instead the klunky app hack. We will need to connect to the MPPT, send the PIN, issue some commands to get the AES key. Is that even possible?
- Go to sleep when solar panels are off
- Add support for mDNS lookup of the MQTT broker
- Augment the MPPT data with stats such as max yield, min/max battery voltage
- Make the values more human readable instead of the basic registers? Maybe put the registers in one topic, and the human readable in other topics?


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
  - 


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
