import math

q0 = 0.9907
q1 = -0.0115
q2 = -0.0586
q3 = -0.1079

yaw = math.atan2(2 * (q0*q3 + q1*q2), 2 * (q0*q0 + q1*q1) - 1) * 57.295779513
pitch = math.asin(-2 * (q1*q3 - q0*q2)) * 57.295779513
roll = math.atan2(2 * (q0*q1 + q2*q3), 2 * (q0*q0 + q3*q3) - 1) * 57.295779513

print(f"yaw={yaw}, pitch={pitch}, roll={roll}")
