# script for map filtering (median filter)
# put your map in the map.py script
from map import *
import numpy as np
from copy import deepcopy
from itertools import product

for i in range(len(exMap)):
    if exMap[i] == 100:
        exMap[i] = "■"
    else:
        exMap[i] = " "

for point in range(len(exMap)):
    if exMap[point] == -1:
        exMap[point] = 0

matrix = np.resize((np.asarray(exMap)), (230, 269))

matrix = matrix[:228]
mtx = []
for l in range(len(matrix)):
    mtx.append(matrix[l][:267])

mtx = np.asarray(mtx)


def filter(mtx, size):
    filtered = deepcopy(mtx)
    h = list(product(range(-int((size - 1) / 2), int((size - 1) / 2) + 1), repeat=2))
    print(h)
    for y in range(int((size - 1) / 2), len(mtx) - int((size - 1) / 2)):
        for x in range(int((size - 1) / 2), len(mtx[y]) - int((size - 1) / 2)):
            ec = 0
            for c in range(size**2):
                if mtx[y + h[c][0]][x + h[c][1]] == "■":
                    ec += 1
            if ec > (size**2 - 1) / 2:
                filtered[y][x] = "■"
            else:
                filtered[y][x] = " "
    return filtered


for l in mtx:
    print(*l)

for l in filter(filter(mtx, 7), 3):
    print(*l)
