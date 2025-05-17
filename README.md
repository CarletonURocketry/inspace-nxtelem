# CU InSpace Telemetry System

This repository contains the application code for the CU InSpace telemetry system running on NuttX.

This system is built on top of many different device drivers which have been contributed to the NuttX kernel, but this
repository only contains application level code.

## Setup

Before using this application you'll need to set up and configure NuttX for a target. The NuttX simulator (the sim:nsh
configuration in particular) is a great first target to get this application running with. You can learn how to set up NuttX 
[here](https://nuttx.apache.org/docs/latest/index.html) and learn how to configure the simulator at 
[this page](https://nuttx.apache.org/docs/latest/guides/simulator.html)

Start by cloning this repository in the NuttX apps folder or add a symlink to it in the apps folder. Then, in
NuttX's menuconfig for applications, enable the Inspace Telemetry application. The code for this application is included under
`telemetry/`. After enabling, you can select to build tests as a seperate application, included under the `tests/` folder.
Some important configuration options for the main telemetry application include setting the callsign used on the radio
(a legal requirement if using a radio on an amateur frequency), setting the location of data logging, setting non-volatile
memory to store important flight state information in, or turning on debug mode to enable printed output (very important if running for development).

You can also enable an application to mount an sd card (useful on our flight hardware), included under `sdcard-mounter/`,
or set up fake sensor data, included under `/mocking` and discussed [below](#faking-sensor-data), which is useful when running
on the NuttX simulator

## Tests

Tests are included in an seperate application. The Inspace Telemetry application must be enabled in configuration first, after which the test 
application can be enabled and run like any other NuttX application. These tests use the Unity Test test framework for C. Information about the
test framework can be found [here](https://github.com/ThrowTheSwitch/Unity). Note that test assertions list the wrong file of origin, due to 
Unity being used in a way it isn't designed for. The line numbers for the assertions are still accurate.

## Faking Sensor Data

Running the program on the NuttX simulator is a great way to test the system. NuttX provides a way to mock sensor data as if it were
coming from real sensors and easy ways to add new fake sensor data.

Some mocked sensors and test data are already included under `/mocking`, and can be enabled as follows. First, you'll need genromfs
(which you'll already have if building for the NuttX simulator, and can find instructions for getting in the simulator documentation).
Then, enable the mocking program in the same area as the telemetry application. Running the program will mount the test data included 
under `mocking/test-data` and will register NuttX drivers to read from these files and create sensor data.

To add new faked sensors or fake sensor data, follow the example of the test data under `mocking/test-data` and add new fakesensor registrations
like those already included in `mocking/mocking_main.c`
