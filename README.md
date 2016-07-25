# SousVide_arduino

Using PID algorithm to keep a water bath at a constant temperature.
Required for Sous Vide, can also be useful for mashing when homebrewing beer.
This sketch is based on the tutorial from adafruit: 
https://learn.adafruit.com/sous-vide-powered-by-arduino-the-sous-viduino
 
The main differences are that this implementation does not use a shield. 
I use a 1.3" 128x64 OLED screen (SPI), and the buttons are currently connected on a breadboard.
But most of the code is based on the sketch in the adafruit tutorial.


The main sketch is in oled_SousVide_PID.
But since this was my first real arduino project, two additional small sketches used for learning the basics are included.
ButtonPress was simply used to learn how to use buttons with the arduino (using the internal pullup resistor). 
And TemperatureReadingDallas was for learning to use the temperature sensor, ds18b20.
