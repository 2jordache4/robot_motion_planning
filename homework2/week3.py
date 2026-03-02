#!/usr/bin/env python3

import matplotlib.pyplot as plt
from me570_geometry import Polygon, rot2d
from me570_robot import polygons
import numpy as np

link2 = polygons[1]
link2_transformed = Polygon(
    rot2d(np.pi / 4) @ link2.vertices + np.array([[3], [1]]))

link2.plot()
link2_transformed.plot()
plt.show()
# 
