#!/usr/bin/python3
# BSD 3-Clause License
#
# Copyright (c) 2025 - <YEARS>, Sun Yat-sen University.
# All rights reserved.
#
# Authors:
# Hanamy: rongerch@outlook.com
#
# Paper:
# Aerodynamic Prior-free Trajectory Generation and Tracking Control for a Tail-sitter UAV.

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import minimize

# Data provided
X = np.array([0.0, 0.33, np.pi])  # X values in radians
y = np.array([0.0, 0.146277, 0.17]) # y = 2sin(x) + 1 + noise

# --- Model 1: Using the first two data points with constraint a1 > 0 ---
X1 = X[:2]
y1 = y[:2]
X1_features = np.c_[np.sin(X1), np.ones((2, 1))]

# Objective function (squared error)
def objective_function1(weights):
    return np.sum((X1_features @ weights - y1)**2)

# Constraint: a1 > 0
constraint1 = {'type': 'ineq', 'fun': lambda weights: weights[0]}

# Initial guess for weights (important for constrained optimization)
initial_weights1 = np.array([1.0, 1.0])  # Start with positive a1

# Perform constrained optimization
result1 = minimize(objective_function1, initial_weights1, constraints=constraint1)
weights1 = result1.x
a1 = weights1[0]
b1 = weights1[1]


# --- Model 2: Using the last two data points with constraint a2 > 0 ---
X2 = X[1:]
y2 = y[1:]
X2_features = np.c_[np.sin(X2), np.ones((2, 1))]

# Objective function (squared error)
def objective_function2(weights):
    return np.sum((X2_features @ weights - y2)**2)

# Constraint: a2 > 0
constraint2 = {'type': 'ineq', 'fun': lambda weights: weights[0]}

# Initial guess for weights
initial_weights2 = np.array([1.0, 1.0])

# Perform constrained optimization
result2 = minimize(objective_function2, initial_weights2, constraints=constraint2)
weights2 = result2.x
a2 = weights2[0]
b2 = weights2[1]



# --- Plotting ---
x_plot = np.linspace(0, 2*np.pi, 100)
y1_plot = a1 * np.sin(x_plot) + b1
y2_plot = a2 * np.sin(x_plot) + b2


plt.scatter(X[:2], y[:2], label="Data for Model 1", color='blue')
plt.plot(x_plot, y1_plot, label="Model 1", color='blue')

plt.scatter(X[1:], y[1:], label="Data for Model 2", color='red')
plt.plot(x_plot, y2_plot, label="Model 2", color='red')

plt.xlabel("X (radians)")
plt.ylabel("y")
plt.title("Two Constrained Linear Regression Models")
plt.legend()
plt.grid(True)
plt.show()
