#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import csv

# Load CSV file with a variable number of columns
def load_csv(filename):
    # Use numpy to load data, skip the first row if it contains headers
    try:
        data = np.loadtxt(filename, delimiter=';', skiprows=0)
    except Exception as e:
        print("Error loading file:", e)
        return None
    return data

# Plot each column in the CSV file
def plot_data(data):
    num_columns = data.shape[1]  # Get number of columns in the data

    plt.figure(figsize=(10, 6))

    # for i in range(num_columns):
    #     plt.plot(data[:, i], label=f'Column {i + 1}')
    # plt.plot(data[:,0], data[:,1], label=f'AD {1 + 1}')
    plt.plot(data[:,1], label=f'AD {1}')
    plt.plot(data[:,2], label=f'AD {2}')

    plt.xlabel('Row Index')
    plt.ylabel('Values')
    plt.title('CSV Data Visualization')
    plt.legend()
    plt.show()

# Main function to load and plot data from a CSV file
def main():
    filename = 'an.csv'  # Update with your CSV file path
    data = load_csv(filename)
    if data is not None:
        plot_data(data)

if __name__ == "__main__":
    main()
