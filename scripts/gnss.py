import numpy as np
from pyproj import Transformer

"""
Abstraction to deal with GNSS based navigation
"""


def project_position(lat, lng, reference_frame="EPSG:6052"):
    projection = Transformer.from_crs("EPSG:4326", "EPSG:6052")
    y_gnss, x_gnss = projection.transform(lat, lng)
    return np.array([x_gnss, y_gnss])