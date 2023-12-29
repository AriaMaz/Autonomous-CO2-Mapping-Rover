import csv
import seaborn as sb
from matplotlib import pyplot as plt
import numpy as np
import math
import os

def place_value(carbon_map, x, y, carbon):
    """
    Place a carbon value at the specified coordinates in the map.

    Args:
    carbon_map: numpy array representing the map.
    x, y: coordinates on the map.
    carbon: carbon value to be placed.
    """
    carbon_map[y][x] = (carbon, True)

def fill_gaps(carbon_map):
    """
    Fill gaps in the carbon map using weighted averages of nearby points.

    Args:
    carbon_map: numpy array representing the map with some values set.
    """
    num_rows, num_cols = carbon_map.shape
    true_indices = [(i, j) for i, row in enumerate(carbon_map)
                    for j, item in enumerate(row) if item[1]]
    
    for i in range(num_rows):
        for j in range(num_cols):
            if not carbon_map[i][j][1]:
                distances = np.abs(true_indices - np.array((i, j)))
                weights = 1 / np.sum(distances, axis=1)
                weighted_average = np.average(
                    [carbon_map[tuple(idx)][0] for idx in true_indices], weights=weights)
                carbon_map[i][j] = (weighted_average, False)

def save_map(carbon_map, file_path):
    """
    Save the carbon map as a heatmap image.

    Args:
    carbon_map: numpy array representing the carbon map.
    file_path: path where the heatmap image will be saved.
    """
    ax = sb.heatmap(carbon_map['carbon_concentration'], vmin=600, vmax=1200,
                    cmap='RdYlGn_r', cbar_kws={'pad': 0.1, "shrink": 1}, square=True)
    ax.invert_yaxis()
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    ax.set_xlabel('')
    ax.set_ylabel('')
    ax.tick_params(axis='both', which='both', length=0)
    plt.savefig(file_path, dpi=300)

# Set script directory and change working directory to it
script_dir = os.path.dirname(os.path.abspath(__file__))
os.chdir(script_dir)

# File containing CO2 concentration data
filename = "C02Concentrations.csv"

# Map dimensions and scale
dimension = 201  # Adjust as necessary
scale = 6  # Controls the read distance from origin

# Initialize an empty carbon map
carbon_map = np.zeros((dimension, dimension), dtype=[
                      ('carbon_concentration', 'f4'), ('is_value_set', '?')])

# Process CSV file to populate the carbon map
with open(filename) as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    for line in csv_reader:
        a, b, z = map(float, line)
        # Adjust input by scale and shift so (0,0) is centered
        x = int(a * scale) + math.floor(dimension / 2)
        y = int(b * scale) + math.floor(dimension / 2)
        place_value(carbon_map, x, y, z)

# Fill gaps in the map and save it as a heatmap
fill_gaps(carbon_map)
save_map(carbon_map, "C02HeatMapEg.png")
