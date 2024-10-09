
import os

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as Rot
from scipy.interpolate import interp1d

import matplotlib.pyplot as plt


OUTPUT_DIR = "/home/oskar/Desktop/journal1/plots/"

SHIP_PATH = "/home/oskar/smooth_sailing/data/long3/ship.csv"
LEVER_ARM = np.array([0, 10, -17])
SIGMA_L = np.array([2, 2, 2])

SOURCES = {
    "zero-mean": "/home/oskar/smooth_sailing/data/long3/exp/test_zero/navigation",
    "levered": "/home/oskar/smooth_sailing/data/long3/exp/test_nolidar/navigation",
    "levered-lidar": "/home/oskar/smooth_sailing/data/long3/exp/test_lidar/navigation",
    "eskf": "/home/oskar/icewatch-analysis/GoNorth24_Right/runs/long3/test/navigation"
}

SHIP_PATH = "/home/oskar/smooth_sailing/data/long3/ship.csv"

def predict_position_from_ship_data(ship_data, lever_arm):
    # Extract pose of ship
    rpy = ship_data[["roll", "pitch", "heading"]].to_numpy()
    
    # Estimate lever arm offset from ship orientation
    xyz_offset = Rot.from_euler('xyz', rpy, degrees=True).as_matrix() @ lever_arm
    xyz = ship_data[["x", "y", "z"]].to_numpy() + xyz_offset
    
    xyz_sigma = np.sqrt((np.eye(3) - Rot.from_euler('xyz', rpy, degrees=True).as_matrix())**2 @ (SIGMA_L**2))

    return xyz, xyz_sigma



def compare_altitude(ship_data, nav_data, meas_data, labels):
    colors = ['r', 'g', 'orange', 'pink', 'b']
    
    xyz_pred, xyz_pred_sigma = predict_position_from_ship_data(ship_data, LEVER_ARM)
    z_pred = xyz_pred[:, 2]
    z_pred_sigma = xyz_pred_sigma[:, 2]
    
    
    # Create subplots for different graphs
    fig, axs = plt.subplots(2, 2, figsize=(12, 8), sharex=True)
    ax0, ax1, ax2, ax3 = axs.flatten()  # Axes are (absolute altitude, relative altitude, relative measurements, relative error)
        
    # Absolute altitude
    ax0.plot(ship_data["ts"].values, z_pred, label="Ship prediction", c=colors[0])
    
    for i, nav in enumerate(nav_data):
        ax0.plot(nav["ts"].values, nav["z"].values, label=labels[i], c=colors[i+1])
    #plt.legend()
    
    # Relative altitude
    z_pred_norm = z_pred - z_pred.mean()
    ax1.plot(ship_data["ts"].values, z_pred_norm, label="Ship prediction", c=colors[0])
    ax1.fill_between(ship_data["ts"], z_pred_norm-3*z_pred_sigma, z_pred_norm+3*z_pred_sigma, alpha=0.5, color=colors[0])
    
    for i, nav in enumerate(nav_data):
        ax1.plot(nav["ts"].values, nav["z"].values - nav["z"].values.mean(), label=labels[i], c=colors[i+1])
    ax1.legend()
    
    # Relative measurements
    ax2.plot(ship_data["ts"].values, z_pred_norm, label="Ship prediction", c=colors[0])
    
    for i, meas in enumerate(meas_data):
        ax2.scatter(meas["ts"].values, meas["altitude"].values.mean()-meas["altitude"].values, marker='x', c=colors[i+1])
    
    # Relative errors
    ax3.fill_between(ship_data["ts"], -3*z_pred_sigma, +3*z_pred_sigma, alpha=0.1, color=colors[0])
    
    for i, nav in enumerate(nav_data):
        error = z_pred_norm - (nav["z"].values - nav["z"].values.mean())
        ax3.plot(nav["ts"].values, error, label=labels[i], c=colors[i+1])

    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, "altitude_compared.png"))
    
    

    # Now we can do some stats
    z0, rmse, corr, inliers = [], [], [], []
    for i, nav in enumerate(nav_data):
        z = nav["z"].values
        z_mean = z.mean()
        z_norm = z - z_mean
        
        z0.append(z_mean)
        rmse.append(np.sqrt(np.mean((z_norm - z_pred_norm)**2)))
        corr.append(np.corrcoef(z_norm, z_pred_norm)[0, 1])
        
        inlier_count = np.sum(abs(z_norm - z_pred_norm) < 3*z_pred_sigma)
        inliers.append(inlier_count / z_norm.size)
    
    stats = np.vstack((labels, z0, rmse, corr, inliers)).T
    
    df = pd.DataFrame(stats, columns=["label", "z0", "RMSE", "Correlation", "Inliers"])
    df.to_csv(os.path.join(OUTPUT_DIR, "stats.csv"), index=False)

    print(df.head())
    
    plt.show()

    
    
    


def align_data(ship_data, nav_data, meas_data):
    # Find the time window which is valid for all datasets
    t_ship = ship_data["ts"].values
    
    t0 = t_ship.min()
    t1 = t_ship.max()
    
    for nav in nav_data:
        ts = nav["ts"].values
        if ts.min() > t0:
            t0 = ts.min()
        if ts.max() < t1:
            t1 = ts.max()
            
    for meas in meas_data:
        ts = meas["ts"].values
        if ts.min() > t0:
            t0 = ts.min()
        if ts.max() < t1:
            t1 = ts.max()
    
    # Extract only valid ship data
    ship_idx = (t_ship > t0) & (t_ship < t1)
    ship_data = ship_data[ship_idx]
    
    # Interpolate navigation to the same time stamps as valid ship data
    for i, nav in enumerate(nav_data):
        int = interp1d(nav["ts"].values, nav.values, axis=0)
        int_values = int(ship_data["ts"])
        nav_data[i] = pd.DataFrame(int_values, columns=nav.columns)



    # Interpolate measurements to the same time stamps as valid ship data
    for i, meas in enumerate(meas_data):
        int = interp1d(meas["ts"].values, meas.values, axis=0)
        int_values = int(ship_data["ts"])
        meas_data[i] = pd.DataFrame(int_values, columns=meas.columns)


    
    return ship_data, nav_data, meas_data


if __name__ == "__main__":
    ship_data = pd.read_csv(SHIP_PATH)
    
    labels = []
    nav_data = []
    meas_data = []
    for label, exp in SOURCES.items():
        labels.append(label)
        nav_data.append(pd.read_csv(os.path.join(exp, "nav.csv")))
        meas_data.append(pd.read_csv(os.path.join(exp, "height.csv")))
        
        
    ship_data, nav_data, meas_data = align_data(ship_data, nav_data, meas_data)
    
    
    compare_altitude(ship_data, nav_data, meas_data, labels)
    