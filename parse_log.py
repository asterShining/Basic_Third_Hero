import re

log_data = """
[2026-02-26 07:18:17.949] [debug] [Gimbal] RX: mode=2, q=[0.9907,-0.0115,-0.0586,-0.1079], yaw=-0.2171, yaw_vel=0.0028, pitch=-0.1189, pitch_vel=-0.0035, bullet_speed=0.00, bullet_count=0, crc16=0x58A9
[2026-02-26 07:18:18.045] [debug] [Gimbal] RX: mode=2, q=[0.9907,-0.0115,-0.0586,-0.1079], yaw=-0.2171, yaw_vel=-0.0047, pitch=-0.1189, pitch_vel=-0.0056, bullet_speed=0.00, bullet_count=0, crc16=0x5094
[2026-02-26 07:18:18.144] [debug] [Gimbal] RX: mode=2, q=[0.9907,-0.0115,-0.0587,-0.1080], yaw=-0.2172, yaw_vel=-0.0047, pitch=-0.1190, pitch_vel=-0.0067, bullet_speed=0.00, bullet_count=0, crc16=0xCBBC
[2026-02-26 07:18:18.241] [debug] [Gimbal] RX: mode=2, q=[0.9907,-0.0115,-0.0587,-0.1079], yaw=-0.2172, yaw_vel=-0.0047, pitch=-0.1190, pitch_vel=0.0072, bullet_speed=0.00, bullet_count=0, crc16=0x411E
[2026-02-26 07:18:18.340] [debug] [Gimbal] RX: mode=2, q=[0.9907,-0.0115,-0.0587,-0.1079], yaw=-0.2171, yaw_vel=-0.0015, pitch=-0.1190, pitch_vel=-0.0003, bullet_speed=0.00, bullet_count=0, crc16=0xB1BD
[2026-02-26 07:18:18.437] [debug] [Gimbal] RX: mode=2, q=[0.9907,-0.0115,-0.0586,-0.1080], yaw=-0.2173, yaw_vel=-0.0068, pitch=-0.1189, pitch_vel=0.0040, bullet_speed=0.00, bullet_count=0, crc16=0xC1C8
"""

for line in log_data.strip().split('\n'):
    print(line)
