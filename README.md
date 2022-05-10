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

The [Zachtech WSPR-TX LP1 Transmitter](https://www.zachtek.com/1011) looks like a good candidate for a tracker device. It has GPS for timing, radio based around a Silicon Labs Si5351 clock generator, and uses a Arduino ATmega328p microcontroller. The (firmware)[https://github.com/HarrydeBug/1011-WSPR-TX_LP1] is released under the GPL v3 licence.




