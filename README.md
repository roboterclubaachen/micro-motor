# µMotor: a motor controller for BLDC and DC motors up to 250W

µMotor (or micro-motor) is a motor controller PCB and software made to control any BLDC and DC motor up to 250W.

The micro-motor is supplied with power and CAN bus via a single connector and can control the motor independently.
In addition, a limit switch can be read out and the temperature of the motor can be monitored.

On the microcontroller, freely configurable PID controllers and, if necessary, "motor-with limit switch" components, which can initialize and move the actuator in a parameterizable manner, should form the external interface.

The software is done with [modm.io](https://modm.io/).

For more information read the following blog posts about the project (in German):
* [µMotor: Idee](http://www.roboterclub.rwth-aachen.de/blog/2018/micro-motor-motorcontroller.html)
* [µMotor: Inbetriebnahme und Debugging](http://www.roboterclub.rwth-aachen.de/blog/2018/micro-motor-debugging-inbetriebnahme.html)
* [µMotor 2 - Teil 1: Konzept](http://www.roboterclub.rwth-aachen.de/blog/2019/micro-motor-2-teil-1.html)
* [µMotor 2 - Teil 2: Strommessung](http://www.roboterclub.rwth-aachen.de/blog/2019/micro-motor-2-teil-2.html)
* [µMotor 2 - Teil 3: Microcontroller & Stromversorgung](http://www.roboterclub.rwth-aachen.de/blog/2019/micro-motor-2-teil-3.html)


## Technical data
* 3 phases (DC and BLDC motors)
* Approx. 10A per phase
* 20V operating voltage (battery) -> 40V or 60V design
* Terminals for AB encoder
* optional [magnetic encoder](https://github.com/roboterclubaachen/magnet-motor-encoder) on the PCB
* Terminals for hall sensor
* Temperature sensor (contactless soldered on the PCB or I²C)
* Current measurement
* Cycle-by-cycle current limiting
* LEDs for software status & encoder status
* Configurable ID (From STM32 hardware ID)
* Easy to solder and assemble
* Dimensions: Round, base-Ø 20mm ("Namiki"-motors from China: Ø 22mm)


## Status

PCBs *micro-motor-v1 (rev1)* and *macro-motor* are not supported anymore.

The second version *micro-motor-v2* is basically functional, software improvements are in progress.


## Folder structure

#### `kicad/pcb_name/`
* KiCad files of the PCBs

#### `STM32CubeMX/`
* Pinout configuration of the STM32 microcontrollers done with [STs CubeMX software](https://www.st.com/en/development-tools/stm32cubemx.html)

#### `src/test_*/`
* Test software. Useful during PCB assembly

#### `src/app/`
* Main Software

## License

Software is licensed under [GPLv3](LICENSE).
Hardware (PCB files, ...) is licensed under [CERN OHL v.1.2](LICENSE.hardware).
