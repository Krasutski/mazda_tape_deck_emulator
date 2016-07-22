#Mazda tape deck emulator

Tape Deck Emulator for Mazda 6 up to 2006 year (with MD/TAPE button). After connect this device to tape connector audio system will swich device (via TAPE/MD button) to "Tape mode" and you can use tape audio signals for playing external audio sources.
This emulator well suited for implement AUX functionality (of course instead of tape).

##Software

Code based on protocol provided from [http:\\\\nikosapi.org](http://nikosapi.org/w/index.php/Mazda_Entertainment_System_-_Bus_Protocol).

##Hardware

Implementation the tape emulator based on [Arduino micro pro 5V](https://www.sparkfun.com/products/11098) hardware. This device is popular and is available in almost any electrical store also easy to programming.

The device used 5V for powered and easy way to get it this use readymade board with LM7805 voltage regulator(analogue L7805, 78L05, REG1117-5, AMS1117-5 ) or simular boards also you can use board from "cigarette lighter to USB" device. I am using part of the __"MB102 Breadboard Power Supply Module 3.3V 5V"__ board, just google it.

Connecting diagram:

![Mazda tape deck emulator schematics](https://github.com/Krasutski/mazda_tape_deck_emulator/blob/master/doc/mazda_tape_emulator_connecting.png)
