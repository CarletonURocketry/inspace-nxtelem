"""Examples of different types of filtering or fusion on real flight data

This file demonstrates various filtering techniques such as median filtering, linear regressions,
and Kalman filters. It's intention is to give a visual way to compare the filter's performance
and decide on reasonable parameters when implemented in C
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import argparse


class MedianFilter:
    def __init__(self, window_size=3, wrappee=None):
        self.window_size = window_size
        self.times = []
        self.alts = []
        self.wrappee = wrappee

    def add_sample(self, t, y):
        self.times.append(t)
        self.alts.append(y)
        if len(self.times) > self.window_size:
            self.times.pop(0)
            self.alts.pop(0)

        # This is a left handed median, but might make more sense to be symetric
        median = np.median(self.alts)

        if (self.wrappee is not None):
            median_altitude, velocity, acceleration = self.wrappee.add_sample(t, median)
        else:
            median_altitude, velocity, acceleration = median, 0.0, 0.0

        return median_altitude, velocity, acceleration  # No velocity or acceleration in median filter


class MovingAverage:
    def __init__(self, window_size=3, wrappee=None):
        self.window_size = window_size
        self.times = []
        self.alts = []
        self.wrappee = wrappee

    def add_sample(self, t, y):
        self.times.append(t)
        self.alts.append(y)
        if len(self.times) > self.window_size:
            self.times.pop(0)
            self.alts.pop(0)

        mean = np.mean(self.alts)

        if (self.wrappee is not None):
            mean_altitude, velocity, acceleration = self.wrappee.add_sample(t, mean)
        else:
            mean_altitude, velocity, acceleration = mean, 0.0, 0.0

        return mean_altitude, velocity, acceleration  # No velocity or acceleration in moving average


class RollingRegression:
    def __init__(self, window_size=15):
        self.window_size = window_size
        self.times = []
        self.alts = []

    def add_sample(self, t, y):
        self.times.append(t)
        self.alts.append(y)
        if len(self.times) > self.window_size:
            self.times.pop(0)
            self.alts.pop(0)

        x = np.array(self.times)
        y = np.array(self.alts)

        Sy = np.sum(y)
        Sx = np.sum(x)
        Sxy = np.sum(x * y)
        Sxx = np.sum(x ** 2)

        if len(x) < 2 or Sxx == 0:
            return np.mean(y), 0.0, 0.0

        alpha = ((Sy * Sxx) - (Sx * Sxy)) / (len(x) * Sxx - Sx ** 2)
        beta = ((len(x) * Sxy) - (Sx * Sy)) / (len(x) * Sxx - Sx ** 2) 
        #altitude = alpha + beta * x[-1]
        altitude = np.mean(y)
        return altitude, beta, 0.0  # No acceleration from linear model


class KalmanFilter2D:
    def __init__(self, process_var=1e-2, measurement_var=1.0):
        self.x = None
        self.P = np.eye(2) * 1000
        self.F = np.eye(2)
        self.H = np.array([[1, 0]])
        self.Q = np.eye(2) * process_var
        self.R = np.array([[measurement_var]])
        self.last_time = None

    def add_sample(self, t, z):
        z = np.array([z])
        if self.x is None:
            self.x = np.array([z[0], 0.0])
            self.last_time = t
            return self.x[0], self.x[1], 0.0

        dt = t - self.last_time
        self.last_time = t

        self.F[0, 1] = dt
        # Predict step
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

        # Update step
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K.flatten() * y
        self.P = (np.eye(2) - K @ self.H) @ self.P
        return self.x[0], self.x[1], 0.0  # No acceleration in 1D model


class KalmanFilter3D:
    def __init__(self, process_var=1e-2, measurement_var=1.0):
        self.x = None
        self.P = np.eye(3) * 1000
        self.F = np.eye(3)
        self.H = np.array([[1, 0, 0]])
        self.Q = np.eye(3) * process_var
        self.R = np.array([[measurement_var]])
        self.last_time = None

    def add_sample(self, t, z):
        z = np.array([z])
        if self.x is None:
            self.x = np.array([z[0], 0.0, 0.0])
            self.last_time = t
            return self.x[0], self.x[1], self.x[2]

        dt = t - self.last_time
        self.last_time = t

        self.F[0, 1] = dt
        self.F[0, 2] = 0.5 * dt ** 2
        self.F[1, 2] = dt

        # Predict step
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

        # Update step
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K.flatten() * y
        self.P = (np.eye(3) - K @ self.H) @ self.P
        return self.x[0], self.x[1], self.x[2]  # Full state estimate
    

def generate_predictions(model, times, altitudes):
    alt_preds = []
    vel_preds = []
    acc_preds = []
    for t, a in zip(times, altitudes):
        alt, vel, acc = model.add_sample(t, a)
        alt_preds.append(alt)
        vel_preds.append(vel)
        acc_preds.append(acc)
    return np.array(alt_preds), np.array(vel_preds), np.array(acc_preds)


def plot_predictions(method_name, times, true_altitudes, pred_altitudes, pred_velocities):
    fig, axs = plt.subplots(2, 1, figsize=(12, 10), sharex=True)

    axs[0].plot(times, true_altitudes, label='True Altitude')
    axs[0].plot(times, pred_altitudes, '--', label=f'{method_name} Altitude')
    axs[0].set_ylabel('Altitude (m)')
    axs[0].set_title('Altitude Comparison')
    axs[0].legend()
    axs[0].grid(True)

    axs[1].plot(times, pred_velocities, label=f'{method_name} Velocity')
    axs[1].set_ylabel('Velocity (m/s)')
    axs[1].set_xlabel('Time (s)')
    axs[1].set_title('Velocity Estimate')
    axs[1].grid(True)
    axs[1].legend()


def main(csv_file):
    df = pd.read_csv(csv_file)
    times = df['time'].to_numpy()
    altitudes = df['altitude'].to_numpy()

    alt_preds, vel_preds, acc_preds = generate_predictions(RollingRegression(15), times, altitudes)
    plot_predictions("Linear Regression, window 15", times, altitudes, alt_preds, vel_preds)

    alt_preds, vel_preds, acc_preds = generate_predictions(RollingRegression(30), times, altitudes)
    plot_predictions("Linear Regression, window 30", times, altitudes, alt_preds, vel_preds)

    alt_preds, vel_preds, acc_preds = generate_predictions(MedianFilter(5, RollingRegression(15)), times, altitudes)
    plot_predictions("Median Filter, window 5 -> Linear Regression, window 15", times, altitudes, alt_preds, vel_preds)

    alt_preds, vel_preds, acc_preds = generate_predictions(MovingAverage(10, RollingRegression(15)), times, altitudes)
    plot_predictions("Moving Average, window 10 -> Linear Regression, window 15", times, altitudes, alt_preds, vel_preds)

    alt_preds, vel_preds, acc_preds = generate_predictions(KalmanFilter2D(), times, altitudes)
    plot_predictions("Kalman Constant Velocity", times, altitudes, alt_preds, vel_preds)

    alt_preds, vel_preds, acc_preds = generate_predictions(KalmanFilter3D(), times, altitudes)
    plot_predictions("Kalman Constant Acceleration", times, altitudes, alt_preds, vel_preds)

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Compare filter models.')
    parser.add_argument('csv_file', help='CSV file with time and altitude columns')
    args = parser.parse_args()
    main(args.csv_file)
