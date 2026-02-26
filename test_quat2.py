import math

# x, y, z, w
q0 = -0.1079 # w
q1 = 0.9907 # x
q2 = -0.0115 # y
q3 = -0.0586 # z

yaw = math.atan2(2 * (q0*q3 + q1*q2), 2 * (q0*q0 + q1*q1) - 1) * 57.295779513
pitch = math.asin(max(-1.0, min(1.0, -2 * (q1*q3 - q0*q2)))) * 57.295779513
roll = math.atan2(2 * (q0*q1 + q2*q3), 2 * (q0*q0 + q3*q3) - 1) * 57.295779513

print(f"If w=q3: yaw={yaw}, pitch={pitch}, roll={roll}")

q0 = -0.0586 
q1 = -0.1079 
q2 = 0.9907 
q3 = -0.0115 
yaw = math.atan2(2 * (q0*q3 + q1*q2), 2 * (q0*q0 + q1*q1) - 1) * 57.295779513
pitch = math.asin(max(-1.0, min(1.0, -2 * (q1*q3 - q0*q2)))) * 57.295779513
roll = math.atan2(2 * (q0*q1 + q2*q3), 2 * (q0*q0 + q3*q3) - 1) * 57.295779513
print(f"If rotation: yaw={yaw}, pitch={pitch}, roll={roll}")

