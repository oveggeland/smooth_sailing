
import os

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as Rot
from scipy.interpolate import interp1d

import matplotlib.pyplot as plt

import gtsam
from gtsam import Pose3, Values, NonlinearFactorGraph, PriorFactorPose3, Rot3, Point3
from gtsam import noiseModel

plt.rcParams['font.size'] = 20
pd.set_option("display.precision", 2)
np.set_printoptions(precision=2)

RAD2DEG = 180 / np.pi
DEG2RAD = np.pi / 180

COLORS = ['r', 'g', 'b', 'magenta', 'orange']
OUTPUT_DIR = "/home/oskar/Desktop/journal1/plots/"

SHIP_PATH = "/home/oskar/smooth_sailing/data/long3/ship.csv"
LEVER_ARM = np.array([1.45, 13.37, -16.77])
EULER_ALIGN = np.array([0.09, -29.44, 90.49])

LEVER_ARM = np.array([0, 11.5, -17])
EULER_ALIGN = np.array([0, -30, 90])

T_ALIGN = Pose3(
    Rot3.RzRyRx(DEG2RAD*EULER_ALIGN),
    LEVER_ARM
)

SIGMA_L = np.array([2, 2, 2])

SOURCES = {
    "ESKF": "/home/oskar/icewatch-analysis/GoNorth24_Right/runs/long3/test/navigation",
    #"Zero-mean": "/home/oskar/smooth_sailing/data/long3/exp/test_zero/navigation",
    #"Levered": "/home/oskar/smooth_sailing/data/long3/exp/test_nolidar/navigation",
    "Proposed": "/home/oskar/smooth_sailing/data/long3/exp/journal1/navigation"
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



def df_to_pose3(df, angles=["roll", "pitch", "yaw"], deg=False):
    pose_list = []
    
    for _, row in df.iterrows():
        # Create rotation using roll, pitch, yaw
        if deg:
            rotation = Rot3.RzRyRx(DEG2RAD*row[angles[0]], DEG2RAD*row[angles[1]], DEG2RAD*row[angles[2]])
        else:
            rotation = Rot3.RzRyRx(row[angles[0]], row[angles[1]], row[angles[2]])
        # Create translation using x, y, z
        translation = np.array([row['x'], row['y'], row['z']])
        
        # Create Pose3 object and append to the list
        pose = Pose3(rotation, translation)
        pose_list.append(pose)
    
    return pose_list


def compare_altitude_roll_pitch(ship_data, nav_data, labels, tlim=(100, 700)):
    ts = ship_data["ts"].values
    ts = ts - ts.min()
    
    ship_poses = df_to_pose3(ship_data, angles=["roll", "pitch", "heading"], deg=True)
    
    x_pred = np.zeros((len(ship_poses), 3))
    
    for i, T_ship in enumerate(ship_poses):
        T_pred = T_ship.compose(T_ALIGN)
        x_pred[i, 0] = T_pred.translation()[2]
        x_pred[i, 1:] = RAD2DEG*T_pred.rotation().rpy()[:2]
        
        
    x_pred_norm = x_pred - np.mean(x_pred, axis=0)
    x_est_norm = np.zeros((len(labels), *x_pred_norm.shape))
    x_norm_error = np.zeros_like(x_est_norm)
    
    # Find norm and errors
    for i, nav in enumerate(nav_data):
        x_est = nav[["z", "roll", "pitch"]].values
        x_est[:, 1:] = x_est[:, 1:]*RAD2DEG
        x_est_norm[i] = x_est - np.mean(x_est, axis=0)
        x_norm_error[i] = x_est_norm[i] - x_pred_norm
    
    # Plot trajectories
    t0, t1 = tlim    
    ax_ylabels = ["Altitude [m]", "Roll [deg]", "Pitch [deg]"]
    fig, axs = plt.subplots(3, 1, sharex=True, figsize=(15, 8), num="ARP trajectory")
    for i in range(3):
        ax = axs[i]
        ax.plot(ts[t0:t1], x_pred_norm[t0:t1, i], label="Ship prediction", c=COLORS[0], linewidth=5, zorder=0)
        
        for j in range(len(labels)):
            ax.plot(ts[t0:t1], x_est_norm[j, t0:t1, i], label=labels[j], c=COLORS[1+j], zorder=1+j)#, linestyle='dashed')
            
        ax.set_ylabel(ax_ylabels[i], rotation=45, ha='right')
        ax.yaxis.tick_right()
    
    
    plt.tight_layout()
    axs[0].legend(ncols=3)
    plt.savefig(os.path.join(OUTPUT_DIR, "arp_states.pdf"))
            
    fig, axs = plt.subplots(3, 1, sharex=True, figsize=(15, 8), num="ARP errors")
    for i in range(3):
        ax = axs[i]
        
        for j in range(len(labels)):
            ax.plot(ts[t0:t1], x_norm_error[j, t0:t1, i], label=labels[j], c=COLORS[1+j], zorder=1+j)#, linestyle='dashed')
            
        ax.set_ylabel(ax_ylabels[i], rotation=45, ha='right')
        ax.yaxis.tick_right()

    plt.tight_layout()
    axs[0].legend(ncols=2)
    plt.savefig(os.path.join(OUTPUT_DIR, "arp_errors.pdf"))
            
        
        


def compare_all(ship_data, nav_data, labels):
    ts = ship_data["ts"].values
    ts = ts - ts.min()
    
    ship_poses = df_to_pose3(ship_data, angles=["roll", "pitch", "heading"], deg=True)
    
    x_pred = np.zeros((len(ship_poses), 6))
    
    for i, T_ship in enumerate(ship_poses):
        T_pred = T_ship.compose(T_ALIGN)
        x_pred[i, :3] = T_pred.translation()
        x_pred[i, 3:] = RAD2DEG*T_pred.rotation().rpy()
        
        
    x_pred_norm = x_pred - np.mean(x_pred, axis=0)
    x_est_norm = np.zeros((len(labels), *x_pred_norm.shape))
    x_norm_error = np.zeros_like(x_est_norm)
    
    # Find norm and errors
    for i, nav in enumerate(nav_data):
        x_est = nav[["x", "y", "z", "roll", "pitch", "yaw"]].values
        x_est[:, 3:] = x_est[:, 3:]*RAD2DEG
        x_est_norm[i] = x_est - np.mean(x_est, axis=0)
        x_norm_error[i] = x_est_norm[i] - x_pred_norm
    
    # Plot trajectories
    t0, t1 = 0, -1
    ax_ylabels = ["North [m]", "East [m]", "Altitude [m]", "Roll [deg]", "Pitch [deg]", "Yaw [deg]"]
    fig, axs = plt.subplots(6, 1, sharex=True, figsize=(15, 15), num="Full trajectory")
    for i in range(6):
        ax = axs[i]
        ax.plot(ts[t0:t1], x_pred_norm[t0:t1, i], label="Ship prediction", c=COLORS[0], linewidth=5, zorder=0)
        
        for j in range(len(labels)):
            ax.plot(ts[t0:t1], x_est_norm[j, t0:t1, i], label=labels[j], c=COLORS[1+j], zorder=1+j)#, linestyle='dashed')
            
        ax.set_ylabel(ax_ylabels[i], rotation=45, ha='right')
        ax.yaxis.tick_right()
    
    
    plt.tight_layout()
    axs[0].legend(ncols=3)
    plt.savefig(os.path.join(OUTPUT_DIR, "all_states.pdf"))
            
    fig, axs = plt.subplots(6, 1, sharex=True, figsize=(15, 15), num="Estimated errors")
    for i in range(6):
        ax = axs[i]
        
        for j in range(len(labels)):
            ax.plot(ts[t0:t1], x_norm_error[j, t0:t1, i], label=labels[j], c=COLORS[1+j], zorder=1+j)#, linestyle='dashed')
            
        ax.set_ylabel(ax_ylabels[i], rotation=45, ha='right')
        ax.yaxis.tick_right()

    plt.tight_layout()
    axs[0].legend(ncols=2)
    plt.savefig(os.path.join(OUTPUT_DIR, "all_errors.pdf"))
            
            
            
    rmse = np.sqrt(np.mean(x_norm_error**2, axis=1))
    print("RMSE")
    print(rmse)
    np.savetxt(os.path.join(OUTPUT_DIR, "all_rmse.csv"), rmse, fmt='%.4f')
    
    x_all_norm = np.concatenate((np.expand_dims(x_pred_norm, 0), x_est_norm))
    coeffs = np.zeros((2, 6))
    for i in range(6):
        coeffs[:, i] = np.corrcoef(x_all_norm[:, :, i])[0, 1:]
    
    print("CORR")
    print(coeffs)
    np.savetxt(os.path.join(OUTPUT_DIR, "all_coeffs.csv"), coeffs, fmt='%.2f')
        
        
        


def compare_attitude(ship_data, nav_data, labels):
    ts = ship_data["ts"].values
    ship_poses = df_to_pose3(ship_data, angles=["roll", "pitch", "heading"], deg=True)

    rpy_pred = np.zeros((len(ship_poses), 3))
    
    for i, T_ship in enumerate(ship_poses):
        T_pred = T_ship.compose(T_ALIGN)
        rpy_pred[i] = RAD2DEG*T_pred.rotation().rpy()
        
    rpy_pred_norm = rpy_pred - np.mean(rpy_pred, axis=0)

    
    # Compare roll and pitch in seperate subplots
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True, figsize=(15, 10))
    
    ax1.plot(ts, rpy_pred_norm[:, 0], label="Ship prediction", c=COLORS[0], linewidth=5)
    ax2.plot(ts, rpy_pred_norm[:, 1], label="Ship prediction", c=COLORS[0], linewidth=5)
    ax3.plot(ts, rpy_pred_norm[:, 2], label="Ship prediction", c=COLORS[0], linewidth=5)
    
    
    errors = np.zeros((2, *rpy_pred_norm.shape))
    for i, nav in enumerate(nav_data):
        rpy = RAD2DEG*nav[["roll", "pitch", "yaw"]].values
        rpy_norm = rpy - rpy.mean(axis=0)
        ax1.plot(ts, rpy_norm[:, 0], label=labels[i], c=COLORS[i+1])#, marker='x')
        ax2.plot(ts, rpy_norm[:, 1], label=labels[i], c=COLORS[i+1])#, marker='x')
        ax3.plot(ts, rpy_norm[:, 2], label=labels[i], c=COLORS[i+1])#, marker='x')
        
        errors[i] = rpy_norm - rpy_pred_norm

    stats = np.concatenate((np.array([labels]), np.std(errors, axis=1).T)).T

    ax1.legend()
    ax2.set_xlabel("Time [s]")
    
    ax1.set_ylabel(r"$\Delta \phi$")
    ax1.yaxis.tick_right()
    ax2.set_ylabel(r"$\Delta \theta$")
    ax2.yaxis.tick_right()
    ax3.set_ylabel(r"$\Delta \psi$")
    ax3.yaxis.tick_right()
    
    # Compare roll and pitch in seperate subplots
    fig, axs = plt.subplots(3, 1, sharex=True, figsize=(15, 10), num="Attitude errors")
    
    ylabels = [r"$\delta \phi$", r"$\delta \theta$", r"$\delta \psi$"]
    for i, ax in enumerate(axs):
        ax.axhline(0)
        ax.set_ylabel(ylabels[i])
        
        ax.plot(ts, errors[0, :, i], label=labels[0], c=COLORS[1])#, marker='x')
        ax.plot(ts, errors[1, :, i], label=labels[1], c=COLORS[2])#, marker='x')
    

    axs[0].legend()
    axs[-1].set_xlabel("Time [s]")
    
    
    # Save stats summary
    df = pd.DataFrame(stats, columns=["label", "d_roll_sigma", "d_pitch_sigma", "d_heading_sigma"])
    df.to_csv(os.path.join(OUTPUT_DIR, "stats_attitude.csv"), index=False, float_format='%.2f')
    
    print("Estimate:")
    print(df.head())
    

    

def compare_altitude(ship_data, nav_data, labels):
    xyz_pred, xyz_pred_sigma = predict_position_from_ship_data(ship_data, LEVER_ARM)
    z_pred = xyz_pred[:, 2]
    z_pred_sigma = xyz_pred_sigma[:, 2]
    
    ts = ship_data["ts"].values - ship_data["ts"].values[0] 
    z_pred_norm = z_pred - z_pred.mean()
    
    plt.figure("Single approach", figsize=(15, 6))
    # Ship stuff
    plt.plot(ts, z_pred_norm, label="Ship prediction", c=COLORS[0], linewidth=5, zorder=2)
    plt.fill_between(ts, z_pred_norm-3*z_pred_sigma, z_pred_norm+3*z_pred_sigma, alpha=0.3, color=COLORS[0], zorder=1, label="Confidence interval")

    for i, nav in enumerate(nav_data):
        plt.scatter(ts, nav["z"].values - nav["z"].values.mean(), label=labels[i], marker='x', s=50, c=COLORS[i+1], zorder=3)
        
    plt.legend(ncols=2)
    plt.ylabel("Relative altitude [m]")
    plt.xlabel("Time [s]")
    plt.axhline(0, linestyle="dashed", c='black')

    plt.xlim(400, 600)
    plt.gca().yaxis.tick_right()  # Move y-ticks to the right
    plt.tight_layout()
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
    

def ssa(angles, deg=True):
    half_rot = 180 if deg else np.pi
    return np.where(angles > half_rot, angles-2*half_rot, angles)


def compare_heading(ship_data, nav_data, labels):
    ts = ship_data["ts"].values - ship_data["ts"].values[0]
    ship_heading = ssa(ship_data["heading"].values)
    
    # Create the figure and subplots
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(15, 10))
    
    # First subplot: Absolute heading
    ax1.plot(ts, ship_heading, label="Ship heading", linewidth=3, c=COLORS[0])

    for i, nav in enumerate(nav_data):
        heading = RAD2DEG * nav["yaw"].values
        ax1.plot(ts, heading, label=labels[i], linewidth=3, c=COLORS[i+1])

    ax1.set_ylabel("Heading [deg]")
    ax1.yaxis.tick_right()  # Move y-ticks to the right
    ax1.legend()
    
    mean_diffs = []
    std_diffs = []
    
    # Second subplot: Heading deviation
    for i, nav in enumerate(nav_data):
        heading = RAD2DEG * nav["yaw"].values
        diff = heading - ship_heading
        ax2.plot(ts, diff, label=labels[i], linewidth=3, c=COLORS[i+1])
    
        mean_diffs.append(diff.mean())
        std_diffs.append(diff.std())
    
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Heading Deviation [deg]")
    ax2.yaxis.tick_right()  # Move y-ticks to the right

    # Adjust layout to prevent overlap
    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, "heading_compared.pdf"))
    
    
    # Find mean and std of deviation
    stats = np.vstack((labels, mean_diffs, std_diffs)).T
    
    df = pd.DataFrame(stats, columns=["label", "mean deviation", "std deviation"])
    df.to_csv(os.path.join(OUTPUT_DIR, "stats_heading.csv"), index=False, float_format='%.2f')
    
    print(df.head())



def error(ship_data, xyz_nav, L):
    xyz, _ = predict_position_from_ship_data(ship_data, L)
    return np.sum((xyz - xyz_nav)**2)


from scipy.optimize import minimize
def estimate_lever_arm(ship_data, nav_data):
    xyz_nav = nav_data[["x", "y", "z"]].values
    f = lambda L: error(ship_data, xyz_nav, L)
    res = minimize(f, LEVER_ARM)
    print(res)


def compare_position(ship_data, nav_data, labels):
    ts = ship_data["ts"].values - ship_data["ts"].values[0]
    ship_xyz, ship_xyz_sigma = predict_position_from_ship_data(ship_data, LEVER_ARM)
    
    ship_north = ship_xyz[:, 0]
    ship_east = ship_xyz[:, 1]
    
    # Create the figure and subplots
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(15, 10))
    
    # First subplot: Position
    #ax1.plot(ts, ship_north, label="Pred - North", marker='x', linewidth=3, c=COLORS[0])
    #ax2.plot(ts, ship_east, label="Pred - East", marker='|', linewidth=3, c=COLORS[0])
    
    
    d_north, d_east, sigma_north, sigma_east = [], [], [], []
    for i, nav in enumerate(nav_data):
        north, east = nav["x"].values, nav["y"].values
        dnorth = north-ship_north
        deast = east-ship_east
        
        ax1.plot(ts, dnorth, label=labels[i], linewidth=3, c=COLORS[i+1])
        ax2.plot(ts, deast, label=labels[i], linewidth=3, c=COLORS[i+1])
        
        # Metrics
        d_north.append(dnorth.mean())
        d_east.append(deast.mean())
        sigma_north.append(dnorth.std())
        sigma_east.append(deast.std())

    ax2.set_xlabel("Time [s]")
    ax1.set_ylabel("Northern deviation[m]")
    ax2.set_ylabel("Eastern deviation[m]")
    ax1.yaxis.tick_right()  # Move y-ticks to the right
    ax2.yaxis.tick_right()
    ax1.legend()
    
    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, "position_compared.pdf"))
    
    # Find mean and std of deviation
    stats = np.vstack((labels, d_north, d_east, sigma_north, sigma_east)).T
    
    df = pd.DataFrame(stats, columns=["label", "d_north", "d_east", "sigma_north", "sigma_east"])
    df.to_csv(os.path.join(OUTPUT_DIR, "stats_position.csv"), index=False, float_format='%.2f')
    
    print(df.head())


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


def estimate_alignment(ship_data, nav_data):
    T_ship_list = df_to_pose3(ship_data, angles=["roll", "pitch", "heading"], deg=True)
    T_body_list = df_to_pose3(nav_data)
    """
    Estimate the alignment Pose3 (T_align) between two sets of Pose3 objects (T_body and T_ship).
    
    Args:
    T_body_list (list of gtsam.Pose3): List of body poses.
    T_ship_list (list of gtsam.Pose3): List of corresponding ship poses.
    
    Returns:
    gtsam.Pose3: Estimated alignment pose (T_align).
    """
    
    assert len(T_body_list) == len(T_ship_list), "The two lists must have the same length."
    
    # Create an empty factor graph
    graph = NonlinearFactorGraph()

    # Create an empty set of initial values
    initial = Values()

    # Add the unknown alignment pose (T_align) with an initial guess
    key_align = gtsam.symbol('A', 0)
    initial.insert(key_align, T_ship_list[0].inverse().compose(T_body_list[0]))

    # Noise model for the BetweenFactor
    noise = noiseModel.Isotropic.Sigma(6, 0.1)  # 6 DoF with small noise

    # Add BetweenFactors for each pair of poses
    for i, (T_body, T_ship) in enumerate(zip(T_body_list, T_ship_list)):
        # Create a BetweenFactor: T_align * T_body ≈ T_ship
        graph.add(PriorFactorPose3(
            key_align, T_ship.inverse().compose(T_body), noise
        ))

    # Set up optimizer (Levenberg-Marquardt optimizer)
    params = gtsam.LevenbergMarquardtParams()
    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial, params)

    # Optimize to find the best alignment pose
    result = optimizer.optimize()

    # Get the optimized alignment pose
    T_align = result.atPose3(key_align)
    
    # Compute the covariance matrix of the estimated alignment pose (T_align)
    marginals = gtsam.Marginals(graph, result)
    covariance_matrix = marginals.marginalCovariance(key_align)

    return T_align, covariance_matrix


def interpolate_pose3(pose1, pose2, t):
    # Interpolate the rotation using SLERP
    interpolated_rot = pose1.rotation().slerp(t, pose2.rotation())

    # Interpolate the translations linearly
    interpolated_trans = (1 - t) * pose1.translation() + t * pose2.translation()

    # Return the interpolated Pose3
    return gtsam.Pose3(interpolated_rot, interpolated_trans)

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
   
    #nav_data[1]["x"] += find_in_yaml(os.path.join(SOURCES["Proposed"], "info.yaml"), "x0")
    #nav_data[1]["y"] += find_in_yaml(os.path.join(SOURCES["Proposed"], "info.yaml"), "y0")
        
    ship_data, nav_data, meas_data = align_data(ship_data, nav_data, meas_data)
    
    T_align_matrices = []
    for nav in nav_data:
        T_align, _ = estimate_alignment(ship_data, nav)
        print(T_align.translation(), RAD2DEG*T_align.rotation().rpy())
        
        T_align_matrices.append(T_align)
    
    T_align_int = interpolate_pose3(T_align_matrices[0], T_align_matrices[1], 0.5)
    print(T_align_int.translation(), RAD2DEG*T_align_int.rotation().rpy())
    
    
    compare_all(ship_data, nav_data, labels)
    compare_altitude_roll_pitch(ship_data, nav_data, labels)
    compare_altitude(ship_data, nav_data, labels)
    #compare_heading(ship_data, nav_data, labels)
    #compare_position(ship_data, nav_data, labels)
    #compare_attitude(ship_data, nav_data, labels)
    
    plt.show()
    
    