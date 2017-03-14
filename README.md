#Mazda tape deck emulator

Tape Deck Emulator for Mazda 6 up to 2006 year (with MD/TAPE button). After connect this device to tape connector audio system will swich device (via TAPE/MD button) to "Tape mode" and you can use tape audio signals for playing external audio sources.
This emulator well suited for implement AUX functionality (of course instead of tape). Also emulator allow control plaing if you will use 4-pin audio jact.

##Software
Project implemented in Arduino environment and provide as "Arduino sketch". 

Description on protocol found here [http:\\\\nikosapi.org](http://nikosapi.org/w/index.php/Mazda_Entertainment_System_-_Bus_Protocol). Thanks to Nikosapi!


##Hardware

Implementation the tape emulator based on the [Arduino micro pro 5V](https://www.sparkfun.com/products/11098) hardware. This device is popular and available in any electrical store also easy to programming.

Devide use one pin for conneting to audio system data bus, this pin used as I\O and reconfigured during send command.

Second pin use for control playing on phone (like single button on headset). This functionality allow control playing (start\stop and switch next track). If you use Jack3.5 whithout the MIC pin juct don't connect this pin.

The device used 5V for powered. Unfortunately audio system doesn't provide 5V. Easy way to get it this use readymade board with LM7805 voltage regulator(analogue L7805, 78L05, REG1117-5, AMS1117-5 ) or simular boards. Also you can use board from "car cigarette lighter to USB" device. I am using 5v part of the __"MB102 Breadboard Power Supply Module 3.3V 5V"__ board, just google it.

Device allow use 4-pin audio jack in this case you get play control funtionality. Also you can connect microphone to MIC pin and organize Handsfree in your car.

Connecting diagram:

![Mazda tape deck emulator schematics](https://github.com/Krasutski/mazda_tape_deck_emulator/blob/master/doc/mazda_tape_emulator_jack_3_5_connecting.png)

###Compatibility:
Solution shold be work on all devices with button "TAPE\MD".  
Tested:
 * Mazda 6 2005 3.0L


  
  
  
If you have any question let me know.


Best Regards,  
Denis Krasutski (BLR)  
dkrasutski@gmail.com

