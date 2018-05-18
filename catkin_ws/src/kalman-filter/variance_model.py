#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np


def scale_conversion(value, min_1=0.0, max_1=200.0, min_2=-3.0, max_2=3.0):
    new_value = ((value-min_1) * (max_2-min_2)) / (max_1-min_1) + min_2
    return new_value

def sigmoid_function(value, offset=0.1, scale_factor=100):
    sigmoid = (np.tanh(value)+1) * scale_factor
    sigmoid += offset
    return sigmoid

def get_var_from_distance(value):
    # tanh function values that will approximate to -1 and 1
    tanh_min = -3.0
    tanh_max = 3.0
    # Minimum and maximum distances, beyond which the error will be constant.
    dist_min = 0.0
    dist_max = 100.0
    # Conversion from the distance space to the tanh function space
    norm_dist = scale_conversion(value, dist_min, dist_max, tanh_min, tanh_max)
    # Apply a sigmoid function for getting the variance.
    variance = sigmoid_function(norm_dist)
    return variance


if __name__ == "__main__":
    test_input = list(range(200))
    test_output = []
    for value in test_input:
        test_output.append(get_var_from_distance(value))
    plt.plot(test_input, test_output)
    plt.show()
