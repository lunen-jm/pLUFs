# pLUFs

The Physical LUFS meter will be a project that allows users to have realtime analysis of the LUFs level of the audio being transmitted by their PC.

The datasheets for the projects are located in the `datasheets` folder. Note that information surrounding the circular display can be found here: 

* Getting started: https://wiki.seeedstudio.com/get_start_round_display/
* Hardware: https://wiki.seeedstudio.com/seeedstudio_round_display_usage/

Information regarding the stepper gauge can be found here:

* https://www.adafruit.com/product/2424 

Information regarding stepper motor driver can be found here:

* https://www.adafruit.com/product/2448 
* https://learn.adafruit.com/adafruit-tb6612-h-bridge-dc-stepper-motor-driver-breakout/using-stepper-motors

The kiCad project for the PCB is located in the `pcbMain` folder. The PCB will be used in the display device, and will connect the Seeed XIAO ESP32C3 to the circular display, a Li-ION battery, and the gauge. It will also have a few tactile buttons.