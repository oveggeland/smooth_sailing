
import os
import csv
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as Rot
from scipy.interpolate import interp1d
import matplotlib.gridspec as gridspec

from matplotlib.lines import Line2D
import gtsam
import matplotlib.pyplot as plt

DEG2RAD = np.pi / 180
RAD2DEG = 1/DEG2RAD

OUTPUT_DIR = "/home/oskar/Desktop/journal1/plots/"

SHIP_PATH = "/home/oskar/smooth_sailing/data/long3/ship.csv"

ALIGN_XYZ = np.array([0, 10, -17])
ALIGN_RPY = np.array([0, 0, 0])

COLORS = ['r', 'b', 'g', 'c']

X0 = np.array([2688743.212375814, 20489795.43575031, 0])

EXP_LABELS = ["Proposed", "ESKF"]
SOURCES = [
    "/home/oskar/smooth_sailing/data/long3/exp/journal/navigation",
    "/home/oskar/icewatch-analysis/GoNorth24_Right/runs/long3/test/navigation"
]
SHIP_PATH = "/home/oskar/smooth_sailing/data/long3/ship.csv"

def read_poses_from_csv(file_path, yaw_name="yaw", degree=False):
    ts = []
    poses = []

    with open(file_path, newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        
        for row in reader:
            # Extract the translation
            x = float(row['x'])
            y = float(row['y'])
            z = float(row['z'])
            translation = gtsam.Point3(x, y, z)
            
            # Extract the rotation (roll, pitch, heading)
            roll = float(row['roll'])
            pitch = float(row['pitch'])
            heading = float(row[yaw_name])
            
            # Convert roll, pitch, and heading (yaw) to Rot3 using ZYX convention
            rotation = gtsam.Rot3.RzRyRx(roll*DEG2RAD, pitch*DEG2RAD, heading*DEG2RAD) \
                if degree else gtsam.Rot3.RzRyRx(roll, pitch, heading)
            
            # Create the Pose3 object
            pose = gtsam.Pose3(rotation, translation)
            poses.append(pose)
            
            ts.append(float(row['ts']))
    
    return np.array(ts), np.array(poses)


def compare(ship_data, nav_data, labels, idx=np.r_[:6], window=(0, -1)):
    x_pred = np.empty((ship_data.shape[0], 6), dtype=float)
    
    for T_ship in ship_data:
        pass
        
"""
    stats = np.vstack((labels, z0, rmse, corr, inliers)).T
    
    df = pd.DataFrame(stats, columns=["label", "z0", "RMSE", "Correlation", "Inliers"])
    df.to_csv(os.path.join(OUTPUT_DIR, "stats.csv"), index=False)

    print(df.head())
    
    plt.show()

    
"""
    

def interpolate_pose3(pose1, pose2, t):
    assert 0 <= t <= 1, "Interpolation factor t must be between 0 and 1"

    # Linear on translation
    t_int = (1 - t) * pose1.translation() + t * pose2.translation()
    
    # Spherical interpolation on rotation
    rot_int = pose1.rotation().slerp(t, pose2.rotation())

    return gtsam.Pose3(rot_int, t_int)
    

class poseExtractor:
    def __init__(self, ts, poses) -> None:
        self.ts = ts
        self.poses = poses

    def get_pose(self, t_query):
        idx = np.searchsorted(self.ts, t_query, side="left")
        if idx == 0 or idx == self.ts.size:
            return None
        
        t0, t1 = self.ts[idx-1], self.ts[idx]
        pose0, pose1 = self.poses[idx-1], self.poses[idx]
        
        t = (t_query - t0) / (t1 - t0)
        return interpolate_pose3(pose0, pose1, t)



def align_data(t_ship, ship_poses, timestamps, nav_data):
    # Find the time window which is valid for all datasets
    t0 = t_ship.min()
    t1 = t_ship.max()
    
    for ts in timestamps:
        if ts.min() > t0:
            t0 = ts.min()
        if ts.max() < t1:
            t1 = ts.max()
            
    # Extract only valid ship data
    ship_idx = (t_ship > t0) & (t_ship < t1)
    t_ship = t_ship[ship_idx]
    ship_poses = ship_poses[ship_idx]
    
    # Interpolate navigation to the same time stamps as valid ship data
    int_nav_poses = np.empty((len(nav_data), t_ship.size), dtype=object)
    for i in range(len(nav_data)):
        pe = poseExtractor(timestamps[i], nav_data[i])
        
        for j, ts in enumerate(t_ship):
            int_nav_poses[i, j] = pe.get_pose(ts)
        
    return t_ship, ship_poses, int_nav_poses


def planar_offset(poses):
    x0 = poses[0].translation()
    x0[2] = 0
    
    for i in range(poses.size):
        poses[i] = gtsam.Pose3(poses[i].rotation(), 
                               poses[i].translation() - X0)
    
    return poses


import gtsam
from gtsam import Pose3, NonlinearFactorGraph, Values, noiseModel
from gtsam.utils import plot

def estimate_alignment(ship_poses, system_poses):
    """
    Estimate the rigid transformation T_align between two sets of poses using GTSAM.

    Parameters:
    - ship_poses: list of gtsam.Pose3 objects representing the ship poses
    - system_poses: list of gtsam.Pose3 objects representing the instrumentation poses
    - initial_guess: initial guess for the T_align transformation as a gtsam.Pose3

    Returns:
    - Estimated gtsam.Pose3 for T_align
    """
    assert len(ship_poses) == len(system_poses), "Pose arrays must have the same length"
    
    # Create a factor graph
    graph = NonlinearFactorGraph()

    # Create a noise model (for the prior factor)
    noise = noiseModel.Diagonal.Variances([1e-6]*6)  # Small noise, assuming fairly accurate measurements
    
    # Key for T_align
    T_align_key = gtsam.symbol('T', 0)

    # Add a prior factor for each pair of ship and system poses
    for T_ship, T_system in zip(ship_poses, system_poses):
        # Calculate the measurement: T_ship.inverse() * T_system
        measured_transform = T_ship.inverse().compose(T_system)
        
        # Add a prior factor: T_align is constrained to be T_ship.inverse() * T_system
        graph.add(gtsam.PriorFactorPose3(T_align_key, measured_transform, noise))

    # Create initial estimate values for T_align
    initial_estimate = Values()
    initial_estimate.insert(T_align_key, ship_poses[0].inverse().compose(system_poses[0]))

    # Optimize the graph
    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial_estimate)
    result = optimizer.optimize()

    # Extract the optimized T_align
    T_align_estimated = result.atPose3(T_align_key)

    return T_align_estimated


MODE_LABELS = ["North [m]", "East [m]", "Down [m]", "Roll [deg]", "Pitch [deg]", "Yaw [deg]"]
def compare_estimates(ts, pred, nav, state_idx, relative=False, crop=(0, -1), fname=""):
    ts = ts[crop[0]:crop[1]]
    pred = pred[crop[0]:crop[1]]
    nav = nav[:, crop[0]:crop[1]]
    
    n_poses = ts.size
    n_exp = nav.shape[0]
    
    xyzrpy = np.zeros(((1 + n_exp), n_poses, 6))
    
    for i in range(n_poses):
        xyzrpy[0, i, :3] = pred[i].translation()
        xyzrpy[0, i, 3:] = pred[i].rotation().rpy()*RAD2DEG
        
        for j in range(n_exp):
            xyzrpy[1+j, i, :3] = nav[j, i].translation()
            xyzrpy[1+j, i, 3:] = nav[j, i].rotation().rpy()*RAD2DEG
            
    if relative:
        xyzrpy = xyzrpy - np.mean(xyzrpy, axis=1, keepdims=True)
    
    
    # Plot states
    fig, axes = plt.subplots(state_idx.size, 1, figsize=(10, 2*state_idx.size), sharex=True)

    for ax, idx in zip(axes, state_idx):
        # First plot prediction
        ax.plot(ts, xyzrpy[0, :, idx], c=COLORS[0], linewidth=5, zorder=0, label="Ship prediction")
        
        for j in range(n_exp):
            ax.plot(ts, xyzrpy[1+j, :, idx], c=COLORS[j+1], zorder=1, label=EXP_LABELS[j])
        
        
        ax.yaxis.tick_right()
        ax.set_ylabel(MODE_LABELS[idx], rotation=45, ha='right', va='bottom')
        
    axes[0].legend(ncols=3)
    axes[-1].set_xlabel("Time [s]")
    
    plt.tight_layout()
    if fname:
        plt.savefig(os.path.join(OUTPUT_DIR, fname+".pdf"))
        
        
    # Plot errors
    state_errors = xyzrpy[0] - xyzrpy[1:]
    print(state_errors.shape)
    fig, axes = plt.subplots(state_idx.size, 1, figsize=(10,  2*state_idx.size), sharex=True)

    for ax, idx in zip(axes, state_idx):
        for j in range(n_exp):
            ax.plot(ts, state_errors[j, :, idx], c=COLORS[j+1], zorder=1, label=EXP_LABELS[j])
        
        ax.yaxis.tick_right()
        ax.set_ylabel(MODE_LABELS[idx], rotation=45, ha='right', va='bottom')
        
    axes[0].legend(ncols=3)
    axes[-1].set_xlabel("Time [s]")
    
    plt.tight_layout()
    if fname:
        plt.savefig(os.path.join(OUTPUT_DIR, fname+"_errors.pdf"))
        
        

    # Stats from state errors
        
    
        
import matplotlib
matplotlib.rcParams.update({'font.size': 16})

if __name__ == "__main__":
    # Extract ship poses
    t_ship, ship_poses = read_poses_from_csv(SHIP_PATH, "heading", degree=True)
    
    # Extract estimated poses
    n_exp = len(SOURCES)
    timestamps = []
    nav_poses = []
    for exp in SOURCES:
        ts, poses = read_poses_from_csv(os.path.join(exp, "nav.csv"))
        timestamps.append(ts)
        nav_poses.append(poses)
                
    # Interpolate to align all datasets temporally
    t_ship, ship_poses, nav_poses = align_data(t_ship, ship_poses, timestamps, nav_poses)
    t_ship = t_ship - t_ship.min()
    
    # Remove planar offset
    ship_poses = planar_offset(ship_poses)
    nav_poses[1] = planar_offset(nav_poses[1])


    # Find alignment (best fit)
    align_poses = []
    for i in range(n_exp):
        T_align = estimate_alignment(ship_poses, nav_poses[i])
        align_poses.append(T_align)
 
    align_poses.append(interpolate_pose3(align_poses[0], align_poses[1], 0.5))
    
    xyzrpy = np.zeros((3, 6))
    for i, T in enumerate(align_poses):
        xyzrpy[i, :3] = T.translation()
        xyzrpy[i, 3:] = T.rotation().rpy()*RAD2DEG

    np.set_printoptions(precision=2, suppress=True)
    print(xyzrpy)
    
    
    # Predict from ship data
    T_align = align_poses[-1] # Use interpolated value for alignment in evaluation
    pred_poses = np.empty_like(ship_poses)
    
    for i, T_ship in enumerate(ship_poses):
        pred_poses[i] = T_ship.compose(T_align)
    
    compare_estimates(t_ship, pred_poses, nav_poses, state_idx=np.r_[:6], fname="full_traj")
    compare_estimates(t_ship, pred_poses, nav_poses, state_idx=np.r_[2:5], fname="arp_traj_rel", crop=(100, 500), relative=True)
    
    
    # Here we plot the biases
    nav_prop = pd.read_csv(os.path.join(SOURCES[0], "nav.csv"))
    nav_eskf = pd.read_csv(os.path.join(SOURCES[1], "nav.csv"))
    
    t_prop = nav_prop["ts"].values
    t_prop = t_prop - t_prop.min()
    
    t_eskf = nav_eskf["ts"].values
    t_eskf = t_eskf - t_eskf.min()
    
    acc_bias_prop = nav_prop[["bax", "bay", "baz"]].values
    acc_bias_eskf = nav_eskf[["bax", "bay", "baz"]].values
    
    gyro_bias_prop = nav_prop[["bgx", "bgy", "bgz"]].values
    gyro_bias_eskf = nav_eskf[["bgx", "bgy", "bgz"]].values
    
    fig = plt.figure(figsize=(10, 5))
    
    gs = gridspec.GridSpec(3, 1, height_ratios=[0.05, 1, 1])  # Adjust width ratios to leave space for legend

    # Create subplots (one column, two rows)
    ax_legend = fig.add_subplot(gs[0, 0])
    ax0 = fig.add_subplot(gs[1, 0])  # First subplot
    ax1 = fig.add_subplot(gs[2, 0], sharex=ax0)  # Second subplot
    
    for i in range(3):
        # plt.axhline(GT[i], c=COLORS[i], linewidth=3, linestyle="dashed")
        ax0.plot(t_prop, acc_bias_prop[:, i], c=COLORS[i])
        ax0.plot(t_eskf, acc_bias_eskf[:, i], c=COLORS[i], linestyle='dashed')
        
        ax1.plot(t_prop, gyro_bias_prop[:, i], c=COLORS[i])
        ax1.plot(t_eskf, gyro_bias_eskf[:, i], c=COLORS[i], linestyle='dashed')
    
    ax1.set_xlabel("Time [s]")
    
    ax0.set_ylabel("Acc bias [m/s^2]", rotation=45, ha='center', va='center', labelpad=60)
    ax1.set_ylabel("Gyro bias [deg/s]", rotation=45, ha='center', va='center', labelpad=60)
    
    ax0.yaxis.tick_right()
    ax1.yaxis.tick_right()
    
    plt.xlim(5, t_eskf.max())
    plt.xscale('log')
    
    colors = ['red', 'green', 'blue']
    line_styles = ['--', '-']
        
    # Create custom legend entries for each combination of color and line style
    legend_elements = [
        Line2D([0], [0], color='black', lw=2, linestyle='-', label='Proposed'),
        Line2D([0], [0], color='black', lw=2, linestyle='--', label='ESKF'),
        Line2D([0], [0], color='red', lw=10, linestyle='-', label='X-axis'),
        Line2D([0], [0], color='green', lw=10, linestyle='-', label='Y-axis'),
        Line2D([0], [0], color='blue', lw=10, linestyle='-', label='Z-axis')
    ]


    # Create a single legend and place it on top of the figure
    ax_legend.set_frame_on(False) 
    ax_legend.set_xticks([])  # Remove x-axis ticks
    ax_legend.set_yticks([])  # Remove y-axis ticks
    ax_legend.legend(handles=legend_elements, loc='center', ncol=5)#, bbox_to_anchor=(0.5, 0.95))

    plt.tight_layout()
    
    
    plt.savefig(os.path.join(OUTPUT_DIR, "bias.pdf"))
    
    plt.show()
    
    
    