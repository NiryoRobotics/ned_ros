#!/usr/bin/env python
import math


def euclidean_dist_2_pts(p1, p2):
    """
    Return euclidean distance between 2 points
    :param p1: tuple(X,Y) of the first point's coordinates
    :param p2: tuple(X,Y) of the second point's coordinates
    :return: distance in the same metrics as the points
    """
    x1, y1 = p1
    x2, y2 = p2
    return math.sqrt((float(x1) - float(x2))**2 + (float(y1) - float(y2))**2)
