# µMotor: a motor controller for BLDC and DC motors up to 50W

µMotor (or micro-motor) is a motor controller PCB and software made to control any BLDC and DC motor up to 250W.

The micro-motor is supplied with power and CAN bus via a single connector and can control the motor independently.
In addition, a limit switch can be read out and the temperature of the motor can be monitored.

On the microcontroller, freely configurable PID controllers and, if necessary, "motor-with limit switch" components, which can initialize and move the actuator in a parameterizable manner, should form the external interface.

The software is done with [modm.io](https://modm.io/).
[XPCC]() is used to communicate over the CAN bus.

For more information read [this blog post](http://www.roboterclub.rwth-aachen.de/blog/2018/micro-motor-motorcontroller.html).


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

The first prototype (PCB rev1) is assembled and basically works.
A lot of test for different hardware functionalities (Microcontroller booting, PWM generation, CAN, ...) are available in `src/test_*`.

Improvements for PCB rev2 are documented in [Issue #1](#1).


## Folder structure

#### `kicad/µmotor-*`
* KiCad files of the µMotor PCB incl. Gerber fabrication data.

#### `kicad/drv832x_testboard/`
* KiCad files for a DRV8323 test board. WIP!

#### `STM32CubeMX/`
* Pinout configuration of the STM32L433CU microcontroller done with [STs CubeMX software](https://www.st.com/en/development-tools/stm32cubemx.html).

#### `src/test_*/`
* Test software. Useful during PCB assembly.

#### `src/micro-motor`
* Software. TODO.

## License

Software is licensed under [GPLv3](LICENSE).
Hardware (PCB files, ...) is licensed under [CERN OHL v.1.2](LICENSE.hardware).
