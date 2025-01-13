# CU InSpace Telemetry System

This repository contains the application code for the CU InSpace telemetry system running on NuttX.

This system is built on top of many different device drivers which have been contributed to the NuttX kernel, but this
repository only contains application level code.

# Faking Sensor Data

You can fake sensor data to mock our sensors and do testing on the NuttX Simulator

Setup:
  1. Create a csv file with your data
    - Your csv file needs to start with `interval:<time in ms>`, then the next line is
      the headers, which should be named the same way as the sensor data in `nuttx/sensors/uorb.h`
    - Each line in the csv is a measurement, seperated in time by the amount of time defined 
      by the interval you just set. Seperate columns with commas (Excel likes spaces by default)
  2. Configure NuttX
    - Enable GNSSUTILS_MINMEA_LIB, SENSORS_GNSS (those two are dependencies for fakesensor)
    - Enable SENSORS_FAKESENSOR
    - Enable ROMFS support
  3. Configure the mocking program

Running:
  1. Run the `mocking` program. It will mount a new ROMFS at /usr/local/share and
     enable fakesensors that will publish the data in your csv
  2. Run telemetry_main. When the program opens the sensors, they will activate and
     begin reading from the csv files (creating sensor measurements at the defined intervals)
