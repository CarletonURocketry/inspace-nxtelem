#ifndef _FLIGHT_REPRODUCTION_H_
#define _FLIGHT_REPRODUCTION_H_

/* Data from flight on 2024-06-24
 *
 * Useful Intervals
 * 0-2.5 seconds - Initial burn stage (615m/s, 210m/s^2 max)
 * 2.5-40 seconds - Coasting stage (10,000m max)
 * 40-50 seconds - Starting descent (-25m/s max)
 * 50 seoncds - Parachute activation
 * 50-1330 seconds - Descent stage (-6m/s)
 * 1330-1341 seconds - Landing stage
 * */
double reproduce_flight_altitude(double t);

double reproduce_flight_velocity(double t);

double reproduce_flight_acceleration(double t);

#endif /* _FLIGHT_REPRODUCTION_H_ */
