# script for finding racks on the map using the Hough transform
import numpy as np


def hough_transform(image, theta_res=1, rho_res=1):
    height, width = image.shape

    diag_len = int(np.ceil(np.sqrt(height * height + width * width)))

    thetas = np.deg2rad(np.arange(-90, 90, theta_res))

    rhos = np.arange(-diag_len, diag_len, rho_res)

    accumulator = np.zeros((len(rhos), len(thetas)))

    for y in range(height):
        for x in range(width):
            if image[y, x] != 0:
                for theta_idx, theta in enumerate(thetas):
                    rho = x * np.cos(theta) + y * np.sin(theta)

                    rho_idx = np.argmin(np.abs(rhos - rho))

                    accumulator[rho_idx, theta_idx] += 1

    return accumulator, rhos, thetas


image = np.array(
    [
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    ]
)


def findRects(image, threshold, sideMin, sideMax):
    accumulator, rhos, thetas = hough_transform(image)
    height, width = image.shape
    rectangles = []
    ys, xs = np.where(accumulator >= threshold)

    for y, x in zip(ys, xs):
        for h in range(sideMin, sideMax + 1):
            for w in range(sideMin, sideMax + 1):
                rho = rhos[y]
                theta = thetas[x]
                cos_theta = np.cos(theta)
                sin_theta = np.sin(theta)
                x1 = int(rho * cos_theta - w * sin_theta)
                y1 = int(rho * sin_theta + h * cos_theta)
                x2 = int(rho * cos_theta + w * sin_theta)
                y2 = int(rho * sin_theta - h * cos_theta)
                if (
                    0 <= x1 < width
                    and 0 <= y1 < height
                    and 0 <= x2 < width
                    and 0 <= y2 < height
                    and image[y1, x1] != 0
                    and image[y2, x2] != 0
                ):
                    if not [(x1, y1), (x2, y2)] in rectangles:
                        rectangles.append([(x1, y1), (x2, y2)])

    return rectangles


rects = findRects(image, 3, 1, 11)
racks = []

for rect in rects:
    sizes = [5, 6]
    wallX = abs(rect[0][0] - rect[1][0]) + 1
    wallY = abs(rect[0][1] - rect[1][1]) + 1
    if wallX in sizes:
        sizes.remove(wallX)
        if wallY in sizes:
            racks.append(rect)

print(racks)
print(rects)
