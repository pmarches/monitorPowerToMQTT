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
- Reduce power usage because we seem to sometime draw too much for the masterbus
  - Reduce BLE power usage esp_bredr_tx_power_set
  - Turn off the micro controller LED to save power
  - Go to sleep when solar panels are off
- Add support for mDNS lookup of the MQTT broker
- Augment the MPPT data with stats such as max yield, min/max battery voltage
- Add remote logging
- Make the values more human readable instead of the basic registers? Maybe put the registers in one topic, and the human readable in other topics?
- Convert cache into a MPPT model.

Data augmentation
==
- Min/Max/Median/Avg
- Network 
  - yield (min/max/median) over multiple days
  - Lifetime yield
- How much time was spent in negative amp flow? (Too much load)
- Daily battery consumption
- Daily solar yield
- At what time 
  - did the solar start producing?
  - was the production greater or equal to the consumption?
  - did the solar stop producing?
- History
- Does one MPPT produce less than it should, indicating a problem?

Some of these require long term data storage. Is this project the right layer?

MQTT Schema
===========

These HEX values correspond to the vedirect HEX protocol!

    /venetworking/MPPT_ID/
        0100  HEX Model type, capabilities and so on. 62A0==150|60, ...
        0102  HEX The firmware version in HEX
        2000  Only set on the mater mppt, 10 indicates master status?
        2001  The networked target voltage. Does not change quickly, only on master device. Or Maybe max batt voltage? Seen 1419 1420 1424
        2007  Only seen on master. Monotonic, increments every few seconds (not regular). Stops increasing when the sun goes down, but still increases by 5049 when at dusk, when the panels do not produce power. Increases more when in full sunlight. Maybe yield today or lifetime for the network?
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
        
    /vedirect/MPPT/SERIAL/0xREGID

    /vedirect/GROUP/GROUPID/
        0x2027  Total DC input power (Panel)
        Network status register
     
Graphing
===
1- Use Watts instead of Amps
2- Solar graphs:
  - Network Power
  - Network Yield (to date)
  - Each MPPT Power and yield? Cummulative curve?

3- Battery graph
  - Power flow, power consumed
  - Battery voltage (x2 axis)
4- Midnight to midnight
5- Show target charging voltages (absorbtion+float)


Questions I want answers!
--
    Solar
    At what time did the batteries start charging?
    What was the maximum charging power to the batteries?
    How is each MPPT performing relative to it’s history? Are they degrading?
    How much power did we draw overnight?
    How much dumpload power did we harness?
    What was the lowest voltage that the batteries have reached?
    At what time did the batteries reach full?
    How much yield per MPPT, total network. Distribution amongst MPPTs
    How much time did we spend in negative power during the day?
    At what time did we start and stop producing solar?
    What is the power deviation between Port arch and std arch? Yield also?
    
    
    NMEA
    Max wind gusts today, last hour
    Extract KML file for the whole season
    Where did we spend most of our time?
    When did we loose GPS connection?
    
    
    
    
    Meshtastic
    Max range attained ever
    Where is the dinghy?
    Where is everyone? Magnetic Bearing+distance
    Max somwewhat reliable range
