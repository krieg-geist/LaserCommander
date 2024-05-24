# LaserCommander

This is a heavily modified version of DeltaFlo's amazing LaserProjector project `https://github.com/DeltaFlo/LaserProjector`

It consists of an Arduino-based system designed to control a laser, a serial interface to recieve commands and a python script
to send serial commands to the laser. 

## RP2040 stuff
I fried my last MCP4822 so I modified this to work using RP2040 pwm pins. They go into the amplifier section to boost the signal up to +/-10v. Galvo seems to work pretty good, though you'll want to put a lowpass filter on each of the outputs. I'll re-add the proper DAC stuff at some point. I have it clocked at 240mhz, though I can't tell if that makes a difference

## New Features

- **Serial comms**: Operate basic laser functionality via serial. See python script for implementation example
- **RGB**: Set the laser to any RGB color, with support for RGB and potentially PWM color blending. Seems to work alright with PWM
