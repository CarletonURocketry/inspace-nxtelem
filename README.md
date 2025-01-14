# CU InSpace Telemetry System

This repository contains the application code for the CU InSpace telemetry system running on NuttX.

This system is built on top of many different device drivers which have been contributed to the NuttX kernel, but this
repository only contains application level code.

# Faking Sensor Data

You can fake sensor data to mock our sensors and do testing on the NuttX Simulator

Setup:
  1. Create a csv file with your data
    - Your csv file needs to start with `interval:<time in ms>`, then the next line is
      the headers, which are ignored (check out fakesensor_uorb.c to find what order you need)
    - Each line in the csv is a measurement, seperated in time by the amount of time defined
      by the interval you just set. Seperate columns with commas (Excel likes spaces by default)
  2. Configure NuttX
    - Enable GNSSUTILS_MINMEA_LIB, SENSORS_GNSS
    - Enable SENSORS_FAKESENSOR
    - Enable ROMFS support
  3. Configure SIM_FAKE_BARO
  4. You can either use the program included under `mocking` to include your csv in a read-only
     file system, or you can place your program somewhere in the NuttX repo - the sim:nsh config
     will put a link to the `nuttx` folder at `/data` by default

Running:
  1. Run the program included under `mocking`, which will mount a new drive with your test files
  2. Run telemetry_main. When the program opens the sensors, they will activate and
     begin reading from the csv files (creating sensor measurements at the defined intervals)
