What is this?
==

This project reads the victron energy MPPT bluetooth packets and sends them to a MQTT broker. 

This is the first open source attempt at documenting the VE.Smart bluetooth protocol. If enough interest, we should make it a stand alone library.

Installation
==
This project runs on ESP32 and is built with ESP-IDF. The installation steps are

	idf.py menuconfig
	idf.py flash monitor should


TODO
==
- MQTT topic should contain the mppt serial number, not the mac address
- Get the AES key from the MPPT devices. We will need to connect to the MPPT, send the PIN, issue some commands to get the AES key. Is that even possible?
- Go to sleep when solar panels are off

MQTT Schema
===========

/venetworking/MPPT_ID/
    0100  HEX Model type, capabilities and so on. 62A0==150|60, ...
    0102  HEX The firmware version in HEX
    2000  Only set on the mater mppt, 10 indicates master status?
    2001  maybe target charge voltage. Does not change quickly, only on master device. Or Maybe max batt voltage? Seen 1419 1420
    2007  Yield Network watts produced? Only seen on master
    2008  Yield MPPT today
    200b  some battery voltage, seen 1219 1229 1242, only on master
    200c  Values seen: 3, 5 only on master (Maybe bulk/Float/Abs mode?)
    2027  Network total power (CentiWatt)
    ed8c  MPPT output amps on battery side (CentiAmps)
    edbc  MPPT output (CentiWatt)
    edef  Always seen 12. Maybe the 12v settings?
    ec3e  probably true/false. Seen 1. (Not charger enabled flag, not load output)

