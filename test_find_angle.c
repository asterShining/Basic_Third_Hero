#include <stdio.h>
#include <math.h>

static float FindClosestAngle(float current_angle, float target_angle_norm)
{
    float target_norm = target_angle_norm;
    while (target_norm > 180.0f)
        target_norm -= 360.0f;
    while (target_norm <= -180.0f)
        target_norm += 360.0f;

    float current_round = roundf(current_angle / 360.0f);
    float target = current_round * 360.0f + target_norm;

    float diff = target - current_angle;
    if (diff > 180.0f) {
        target -= 360.0f;
    } else if (diff < -180.0f) {
        target += 360.0f;
    }

    return target;
}

int main() {
    printf("current: -169.19, vision_target: 181.41 -> cmd: %f\n", FindClosestAngle(-169.19, 181.41));
    return 0;
}
