# rpi-bushdl

_Raspberry Pi's Bus Handlers Layer_

This repository contains a generic Bus handler's layer for Rapsberry Pis written in C++ 17.
These handlers were tested in the HERCCULES ballooning project.

The current implementation gives support for IIC buses 0, 1, 3, and 4.
The provided operations execute in mutual exclusion via mutexes, one per bus.
This way high-level applications using multi-threading can access to the IIC buses simultaneously.

## Project's structure

- [**`include`**](include): contains the public header files from this layer, i.e.: the API.
- [**`src`**](src): contains the headers implementations (.c) and private headers.