import pandas as pd
import numpy as np
from math import *

from shapely import Point

def get_tangent_angle(drone_position, airport_center, radius):
    dx = airport_center.x - drone_position.x
    dy = airport_center.y - drone_position.y
    d = sqrt(dx**2 + dy**2)

    if(d > radius):
        theta = atan2(dy, dx)
        beta = asin(radius/d)

        return degrees(theta - beta), degrees(theta + beta)