from matplotlib import pyplot as plt

import csv


x = []
y = []

with open("./ev3_plotter_brick/possible_locations.txt", "r", encoding="utf-8") as file:
    data = csv.reader(file)
    
    for row in data:
        x.append(float(row[0]))
        y.append(float(row[1]))


plt.scatter(x, y)
plt.show()