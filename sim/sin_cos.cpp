#include <stdio.h>
#include <math.h>

#define PI 3.14159265

int angles[] = {0, 45, 90, 135, 180, 225, 270, 315, 360};

int main() {
    printf("Hello, World!\n");
    
    for (int i = 0; i < 9; ++i) {
        printf("Angle: %d - sin: %f\n", angles[i], sin(angles[i] * (PI/180.0f)));
    }
    
    for (int i = 0; i < 9; ++i) {
        printf("Angle: %d - cos: %f\n", angles[i], cos(angles[i] * (PI/180.0f)));
    }
    
    return 0;
}