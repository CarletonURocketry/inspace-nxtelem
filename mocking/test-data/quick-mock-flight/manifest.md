# Quick Mock Flight Data

This data was generated to provide sample flight data that can exercise the full state machine of the telemetry application in around 30 seconds. The apogee of this flight is around 5000m.

There are five stages to this flight data. The flight is only slightly realistic, to make sure it stayed somewhat short while including the important parts of a flight.
1. Landed before flight - five seconds with zero altitude and 9.81m/s acceleration
2. Liftoff - four seconds accelerating at 300m/s^2 from rest
3. Coasting - eight seconds slowing down, accelerating at 100m/s^2
4. Descent - around thriteen seconds descending at a constant speed of -390m/s. This was set extremely high so that the flight wouldn't last a long time
5. Landed after flight - around ten seconds landed after the flight with zero altitude and 9.81m/s acceleration
