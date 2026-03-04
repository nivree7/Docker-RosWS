import math
import numpy as np

def get_quarternion(roll, pitch, yaw):
    """Conver Euler angle into  a Quarternion."""
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]

def calc_orientation(ax, ay, az, mx, my, mz):
    norm_a = math.sqrt(ax**2 + ay**2 + az**2)

    if norm_a == 0: return [0, 0, 0, 1]

    ax /= norm_a
    ay /= norm_a
    az /= norm_a

    pitch = math.asin(-ax)

    roll = math.atan2(ay, az)

    my_1 = my * math.cos(roll) - mz * math.sin(roll)
    mz_1 = my * math.sin(roll) + mz * math.cos(roll)
    mx_1 = mx * math.cos(pitch) + mz_1 * math.sin(pitch)

    yaw = math.atan2(-my_1, mx_1)

    quarternion = get_quarternion(roll, pitch, yaw)

    return quarternion