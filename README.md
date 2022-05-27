# TheFloat
This is where we plan the design of theFloat. The objectives of this project is as follows:
* Design and build a bouy that can be dropped off at sea, and tracked wirelessly.
* It must emit transmissions that contains its location information at all times of the day.
* Transmissions should be received from wherever the bouy is (global coverage).
* It must have the ability to carry on transmitting indefinately(battery recharge capability).
* It must be robust enough to endure harsh sea conditions for at least 1 year.


The bouy shall be deployed at sea and will commence its journey, driven solely by the wind and the waves. Ideally, it should travel for months, and give observers great enjoyment watching the theFloat drifting across the seas. It wll be interesting to know where the sea currents take it. Middle of the Atlantic? To the North Pole? Ideally must not get stuck on some beach.

# Early ideas
The bouy will probably look like Bob Sutton ZL1RS's bouy, which was launched near Australia ([Project Link](https://www.qsl.net/zl1rs/oceanfloater.html)). The key difference is the theFloat will have rechargable batteries, with some current source for recharge(likely solar).

The [Zachtech WSPR-TX LP1 Transmitter](https://www.zachtek.com/1011) looks like a good candidate for a tracker device. It has GPS for timing, radio based around a Silicon Labs Si5351 clock generator, and uses a Arduino ATmega328p microcontroller. The [firmware](https://github.com/HarrydeBug/1011-WSPR-TX_LP1) is released under the GPL v3 licence.

Zachtech trackers and QRP Labs trackers are not very power efficient, especially in sleep mode. They were not really designed to operation on battery power. A narrowed down hardware spec is as follows:

* Single 18650 battery should be able to power it all night
* It should charge up sufficiently during the day
* Power consumption while not transmitting should be in the ~10 uA range. 
* Enough flash storage. The code on the Zachtech WSPR tracker takes up 95% of the available space on the ATmega328P processor. No room for more code features

It sounds like an opportunity to design an IOT WSPR tracker - powered a single battery, powered by the sun, and consumes minimum energy when not transmitting.

The bill of materials could look like this:
* Processor - STM32 or SAMD or ESP32.
* PMIC - needs to charge the battery from solar - needs solar harvesting capablity
* Silabs Si5351 frequency generator for radio
* RF amplifier to increase power output up to 200 mW
* PCB to mount all components. Ideally one sided PCB, small and compact(5 cm x 5 cm)
* Software can be taken from some of the High Alititude Ballon trackers - maybe add in a watchdog for good measure.




