"""_summary_
Generic processing tools for pointclouds
"""


import numpy as np
import open3d as o3d
import cv2 as cv
from scipy.stats import linregress


def t_filter(pcd, channel, min=-np.inf, max=np.inf):
    if not pcd.point.positions.numpy().shape[0]:
        return pcd
    
    val = t_get_channel(pcd, channel)
    mask = (val >= min) & (val <= max)
    
    return pcd.select_by_mask(mask)


def t_percentile_filter(pcd, channel, lower=0, upper=100):
    vals = t_get_channel(pcd, channel)
    sorted_idx = np.argsort(vals)

    lb, ub = int(lower*sorted_idx.size/100), int(upper*sorted_idx.size/100)
    mask = sorted_idx[lb:ub]

    return pcd.select_by_index(mask)


def linear_correction(x, y):
    lin_reg = linregress(x, y)

    y_pred = lin_reg.intercept + lin_reg.slope*x

    y_corrected = y - y_pred
    y_corrected = y.max()*(y_corrected - y_corrected.min()) / (y_corrected.max() - y_corrected.min())
    return y_corrected


def t_normalize_intensity_from_topography(pcd):
    pcd_cpy = pcd.clone()
    z = pcd_cpy.point.positions.numpy()[:, 2]
    i_true = pcd_cpy.point.intensities.numpy().flatten()

    i_corrected = linear_correction(z, i_true)

    pcd_cpy.point.intensities = o3d.core.Tensor(i_corrected.reshape((-1, 1)), dtype=o3d.core.Dtype.UInt16)
    return pcd_cpy


def t_normalize_intensity_from_lidar_depth(pcd):
    pcd_cpy = pcd.clone()
    d = pcd.point.distance.numpy().flatten()
    i_true = pcd.point.intensities.numpy().flatten()

    i_corrected = linear_correction(d, i_true)

    pcd_cpy.point.intensities = o3d.core.Tensor(i_corrected.reshape((-1, 1)), dtype=o3d.core.Dtype.UInt16)
    return pcd_cpy

def t_channel_to_color(pcd, channel, colormap, p_lower=0, p_upper=100):
    vals = t_get_channel(pcd, channel)
    normalized = np.ones(vals.size, dtype=np.uint8)*126 # Default color in case vals are all the same value
    if vals.max() - vals.min():
        normalized = np.clip(255 * (vals - np.percentile(vals, p_lower)) /
                          (np.percentile(vals, p_upper) - np.percentile(vals, p_lower)), 
                          a_min=0, a_max=255).astype(np.uint8)

    c = cv.cvtColor(cv.applyColorMap(normalized, colormap), cv.COLOR_BGR2RGB).squeeze() / 255
    return o3d.utility.Vector3dVector(c)


def t_get_channel(pcd, channel):
    if channel == "topo":
        return pcd.point.positions.numpy()[:, 2]
    elif channel == "color_norm":
        return np.mean(pcd.point.colors.numpy().squeeze(), axis=1)
    return pcd.point[channel].numpy()




"""
Downsample point clouds based on planar position. Average all attributes within a voxel of radius 'size'.
"""
def t_grid_down_sample(pcd, voxel_size=0.1):
    pcd = pcd.clone()
    topo = t_get_channel(pcd, "topo").copy()

    # Set topo to 0
    pcd.point.positions[:, 2] = np.zeros_like(topo)
    pcd.point.topo = topo.reshape((-1, 1))
    
    pcd_down = pcd.voxel_down_sample(voxel_size)
    
    pcd_down.point.positions[:, 2] = pcd_down.point.topo.flatten()
    return pcd_down
    