#include <stdio.h>
#include <math.h>

int main() {
    float current = -169.19;
    float target = -178.589996; // from FindClosestAngle
    float err = target - current;
    printf("target: %f, current: %f, err: %f\n", target, current, err);
    return 0;
}
