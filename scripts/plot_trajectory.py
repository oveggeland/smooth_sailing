#! /bin/python3 

import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np


if __name__ == "__main__":
    # Load the CSV file without a header
    file_path = '/home/oskar/navigation/src/gtsam_nav/data/traj.txt'
    data = pd.read_csv(file_path, header=None)

    # Extract the position data by index using NumPy arrays
    data_np = data.to_numpy()
    x = data_np[:, 0]  # position_x
    y = data_np[:, 1]  # position_y
    z = data_np[:, 2]  # position_z

    # Plot the 3D trajectory
    fig = plt.figure(figsize=(15, 10))

    # Generate time steps
    time = np.arange(len(x))

    ax1 = fig.add_subplot(321, projection='3d')
    sc = ax1.scatter(x, y, z, c=time, cmap='viridis', marker='o')
    ax1.set_xlabel('X Position')
    ax1.set_ylabel('Y Position')
    ax1.set_zlabel('Z Position')
    ax1.set_title('3D Trajectory')
    fig.colorbar(sc, ax=ax1, label='Time Step')

    # Extract velocity data
    velocity_x = data_np[:, 3]
    velocity_y = data_np[:, 4]
    velocity_z = data_np[:, 5]

    # Plot velocity
    ax2 = fig.add_subplot(322)
    ax2.plot(velocity_x, label='Velocity X')
    ax2.plot(velocity_y, label='Velocity Y')
    ax2.plot(velocity_z, label='Velocity Z')
    ax2.set_xlabel('Time Step')
    ax2.set_ylabel('Velocity')
    ax2.set_title('Velocity Over Time')
    ax2.legend()

    # Extract orientation data (roll, pitch, yaw)
    roll = data_np[:, 6]
    pitch = data_np[:, 7]
    yaw = data_np[:, 8]

    # Plot orientation
    ax3 = fig.add_subplot(323)
    ax3.plot(roll, label='Roll')
    ax3.plot(pitch, label='Pitch')
    ax3.plot(yaw, label='Yaw')
    ax3.set_xlabel('Time Step')
    ax3.set_ylabel('Angle (radians)')
    ax3.set_title('Orientation (Roll, Pitch, Yaw) Over Time')
    ax3.legend()

    # Extract acceleration bias data
    acc_bias_x = data_np[:, 9]
    acc_bias_y = data_np[:, 10]
    acc_bias_z = data_np[:, 11]

    # Plot acceleration bias
    ax4 = fig.add_subplot(324)
    ax4.plot(acc_bias_x, label='Acc Bias X')
    ax4.plot(acc_bias_y, label='Acc Bias Y')
    ax4.plot(acc_bias_z, label='Acc Bias Z')
    ax4.set_xlabel('Time Step')
    ax4.set_ylabel('Acceleration Bias')
    ax4.set_title('Acceleration Bias Over Time')
    ax4.legend()

    # Extract gyroscope bias data
    gyro_bias_x = data_np[:, 12]
    gyro_bias_y = data_np[:, 13]
    gyro_bias_z = data_np[:, 14]

    # Plot gyroscope bias
    ax5 = fig.add_subplot(325)
    ax5.plot(gyro_bias_x, label='Gyro Bias X')
    ax5.plot(gyro_bias_y, label='Gyro Bias Y')
    ax5.plot(gyro_bias_z, label='Gyro Bias Z')
    ax5.set_xlabel('Time Step')
    ax5.set_ylabel('Gyroscope Bias')
    ax5.set_title('Gyroscope Bias Over Time')
    ax5.legend()
    
    # Plot gyroscope bias
    ax6 = fig.add_subplot(326)
    ax6.scatter(y, x, c=time)
    ax6.set_xlabel('East[m]')
    ax6.set_ylabel('North[m]')
    ax6.set_title('Cartesian position')
    ax6.legend()



    # Adjust layout and show plot
    plt.tight_layout()
    plt.show()