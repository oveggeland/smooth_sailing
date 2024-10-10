
import os

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as Rot
from scipy.interpolate import interp1d

import matplotlib.pyplot as plt

pd.set_option("display.precision", 2)


OUTPUT_DIR = "/home/oskar/Desktop/journal1/plots/"

SHIP_PATH = "/home/oskar/smooth_sailing/data/long3/ship.csv"
LEVER_ARM = np.array([0, 11.5, -17])
SIGMA_L = np.array([2, 2, 2])

SOURCES = {
    "ESKF": "/home/oskar/icewatch-analysis/GoNorth24_Right/runs/long3/test/navigation",
    #"Zero-mean": "/home/oskar/smooth_sailing/data/long3/exp/test_zero/navigation",
    #"Levered": "/home/oskar/smooth_sailing/data/long3/exp/test_nolidar/navigation",
    "Proposed": "/home/oskar/smooth_sailing/data/long3/exp/test_lidar/navigation"
}

SHIP_PATH = "/home/oskar/smooth_sailing/data/long3/ship.csv"

def predict_position_from_ship_data(ship_data, lever_arm):
    # Extract pose of ship
    rpy = ship_data[["roll", "pitch", "heading"]].to_numpy()
    
    # Estimate lever arm offset from ship orientation
    xyz_offset = Rot.from_euler('xyz', rpy, degrees=True).as_matrix() @ lever_arm
    xyz = xyz_offset + ship_data[["x", "y", "z"]].to_numpy()
    
    xyz_sigma = np.sqrt((np.eye(3) - Rot.from_euler('xyz', rpy, degrees=True).as_matrix())**2 @ (SIGMA_L**2))

    return xyz, xyz_sigma


plt.rcParams['font.size'] = 18

def compare_altitude(ship_data, nav_data, labels):
    colors = ['r', 'g', 'b', 'magenta', 'orange']
    
    xyz_pred, xyz_pred_sigma = predict_position_from_ship_data(ship_data, LEVER_ARM)
    z_pred = xyz_pred[:, 2]
    z_pred_sigma = xyz_pred_sigma[:, 2]
    
    ts = ship_data["ts"].values - ship_data["ts"].values[0] 
    z_pred_norm = z_pred - z_pred.mean()
    
    plt.figure("Single approach", figsize=(15, 6))
    # Ship stuff
    plt.plot(ts, z_pred_norm, label="Ship prediction", c=colors[0], linewidth=5, zorder=2)
    plt.fill_between(ts, z_pred_norm-3*z_pred_sigma, z_pred_norm+3*z_pred_sigma, alpha=0.3, color=colors[0], zorder=1, label="Confidence interval")

    for i, nav in enumerate(nav_data):
        plt.scatter(ts, nav["z"].values - nav["z"].values.mean(), label=labels[i], marker='x', s=50, c=colors[i+1], zorder=3)
        
    plt.legend(ncols=2)
    plt.ylabel("Relative altitude [m]")
    plt.xlabel("Time [s]")
    plt.axhline(0, linestyle="dashed", c='black')

    plt.xlim(400, 600)
    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, "altitude_compared.png"))
    plt.savefig(os.path.join(OUTPUT_DIR, "altitude_compared.pdf"))
    
    
    # Now we can do some stats
    z0, z_sigma, rmse, corr, inliers = [], [], [], [], []
    for i, nav in enumerate(nav_data):
        z = nav["z"].values
        z_mean = z.mean()
        z_norm = z - z_mean
        
        z0.append(z_mean)
        z_sigma.append(z.std())
        rmse.append(100*np.sqrt(np.mean((z_norm - z_pred_norm)**2)))
        corr.append(np.corrcoef(z_norm, z_pred_norm)[0, 1])
        
        inlier_count = np.sum(abs(z_norm - z_pred_norm) < 3*z_pred_sigma)
        inliers.append(100*inlier_count / z_norm.size)
    
    stats = np.vstack((labels, z0, z_sigma, rmse, corr, inliers)).T
    
    df = pd.DataFrame(stats, columns=["label", "z0", "z_sigma", "RMSE", "Correlation", "Inliers"])
    df.to_csv(os.path.join(OUTPUT_DIR, "stats_nav.csv"), index=False, float_format='%.2f')
    
    print("Ship mean and std: ", z_pred.mean(), z_pred.std())
    
    print("Estimate:")
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
        
    meas_data[0]["altitude"] *= (-1)
        
    ship_data, nav_data, meas_data = align_data(ship_data, nav_data, meas_data)
    
    
    compare_altitude(ship_data, nav_data, meas_data, labels)
    