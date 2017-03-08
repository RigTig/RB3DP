# RB3DP
RigTig's Big 3D Printer (RB3DP) is a machine for placing material in 3 dimensions. The effector (printing head) hangs from three strings. The length of each string is adjusted by a small stepper motor, so the effector moves in 3D space.

Many types of effectors are possible. Several have been tried with limited success. This project is about getting useful effectors in the scale of metres rather than millimetres as well as refining the 3D positioning of a string-based 3D printer.

For an initial use case, see https://hackaday.io/project/13442-3d-print-emergency-accommodation where locally sourced dirt is the primary material, plus a binder placed using RB3DP. The effector simply dribbles the diluted binder into the dirt to solidify dirt into walls and roof.

## Key features

Print volume is big: 1 to 100+ cubic metres.

Cost is small: less than US$100.

Minimum movement of effector is currently set at 2mm.

Complete printer can be transported by one person in a backback or bucket.

## Big
The RB3DP can print bigger objects than most other 3D printers. One unusual characteristic is that the print base is triangular. The test prototype was somewhat less than a cubic metre. The second installation was about 5 cubic metres. The design is flexible about the size; it just needs enough string. Not yet tested, but 10 to 20 metres in each direction seems workable and probably even more. Each installation does need to be measured to calibrate the printer.

## Inexpensive
The basic RB3DP without an effector costs about $20 (USD) in electronic parts. The knobby string is about $20 for a 100 metre roll, so the cost depends on how big you want to make the RB3DP. The rest of the components are either 3D printed or are scrounged locally to the build area. For example, you need a wall, fence, tree or post to put each motor at the same height above the ground as the other 2 motors. 

## Accuracy
In 10 metres, 2mm is 0.02%, which is more than adequate for buildings.

## Small and portable
The first version fitted in a bucket, including the cabling. The latest version uses its own wifi network, so all the parts fit into a 2 litre food container. Even if you add 3 collapsible tent poles, strings and tent pegs, the whole thing is not too onerous to carry. You still need a netbook or laptop to interact with the printer, but future implementations can do without a computer interface.

# Components
* wifi stepper: Uses ESP12 to accept commands via a wifi connection to move 28BYJ-48 stepper motor using ULN2003 driver. Code is written in punyforth (a forth-like interpreter).
* GCODE interpreter: Uses the STM8 part of ESP14, with code is written in STM8EF (a forth interpreter).
* Comms hub: uses the ESP8266 part of ESP14, with code written in punyforth. Accepts GCODE via wifi and sends to STM8, plus handles the distribution of commands to wifi steppers, and any replies.
* an effector: not yet decided what type to design. Using a wifi stepper as the basis seems obvious, with the only possible restriction being that ESP12 has only 1 ADC port, so if an effector needs more than one analogue measurement, then need to consider other options (perhaps ESP14).

## WiFi Stepper
Rather than connect with an ethernet cable, how about sending the control for a stepper motor using WiFi? Still have to supply power to each stepper, but that is much simpler than distributing control through wires. This becomes useful when the distance between stepper motors can be variable.

More description is available from https://hackaday.io/project/18533-esp8266-stepper-driver .

## GCODE interpreter
The original design of RB3DP used an Arduino Nano directly connected to 4 28BYJ-48 stepper motors (see slightly buggy code in obsolete_arduino). Problem was cabling for communication to the steppers. So, then the idea was to use ESP8266 to get wifi communications. Unfortunately, NodeMCU uses up a lot of the ESP8266 resources, and code size is too big to actually run in what's left (see Lua code in obsolete-lua). So, next idea was to use an Arduino for GCODE interpreter and connect it to an ESP12 to get the wifi capability. Before getting started, an ESP14 came into contention (being STM8 with serially attached ESP8266), but was the STM8 MCU up to the task of doing GCODE interpretation? There were more than one person who said that STM8 could not possibly do the job. However, GCODE is simple in its design, so why not give it a go. STM8EF to the rescue (thank you, Thomas). 

See bp.f for current STM8EF code for GCODE interpreter, and needs maths.f for a few mathematical functions. Note that serial input is from the ESP8266 and serial output goes to ESP8266. More details in the Wiki (when it gets done).

## Comms hub
The ESP8266 part of ESP14 needs to handle receiving GCODE via wifi and sending to STM8, plus receiving and processing commands from STM8. If the commands require action by wifi stepper and/or effector, then the comms hub is responsible for sending such commands and receiving any acknowledgements.

See bp-ESP.forth for current punyforth code.

## Effector
To be decided, though it has to be soon.
