import numpy
import scipy
from scipy import ndimage

im = scipy.misc.imread('bike.jpg')
im = im.astype('int32')
dy = ndimage.sobel(im, 0)  # vertical derivative
dx = ndimage.sobel(im, 1)  # horizontal derivative
mag = numpy.hypot(dx, dy)  # magnitude
mag *= 255.0 / numpy.max(mag)  # normalize (Q&D)
scipy.misc.imsave('sobel.jpg', mag)

