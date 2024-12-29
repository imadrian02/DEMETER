import numpy as np
import cv2 as cv
import imutils
import math

img = cv.imread('img_1.png')
img = imutils.resize(img, 640)

# grab the dimensions of the image
(h, w, _) = img.shape

# set up the x and y maps as float32
map_x = np.zeros((h, w), np.float32)
map_y = np.zeros((h, w), np.float32)

scale_x = 1
scale_y = 1
center_x = w / 2
center_y = h / 2
radius = w / 2
# amount = -0.75   # negative values produce pincushion
amount = 0.1  # positive values produce barrel

# create map with the barrel pincushion distortion formula
for y in range(h):
    delta_y = scale_y * (y - center_y)
    for x in range(w):
        # determine if pixel is within an ellipse
        delta_x = scale_x * (x - center_x)
        distance = delta_x * delta_x + delta_y * delta_y
        if distance >= (radius * radius):
            map_x[y, x] = x
            map_y[y, x] = y
        else:
            factor = 1.0
            if distance > 0.0:
                factor = math.pow(math.sin(math.pi * math.sqrt(distance) / radius / 2), amount)
            map_x[y, x] = factor * delta_x / scale_x + center_x
            map_y[y, x] = factor * delta_y / scale_y + center_y

# do the remap
dst = cv.remap(img, map_x, map_y, cv.INTER_LINEAR)

# save the result
# cv.imwrite('lena_pincushion.jpg',dst)
# cv.imwrite('img.png', dst)

# show the result
cv.imshow('src', img)
cv.imshow('dst', dst)

cv.waitKey(0)
cv.destroyAllWindows()