#include <iostream>
#include <math.h>
#include <stdint.h>

using namespace std;

#define PI (3.14159265f)


uint8_t const x_ndx = 0;
uint8_t const y_ndx = 1;

uint8_t const pitch_ndx = 0;
uint8_t const roll_ndx = 1;

int16_t quad_coord[2] = {2, 3};
int16_t quad_rotation_deg = 0;
float quad_rotation_rad = (float)quad_rotation_deg * (PI / 180.0f);
int16_t target_coord[2] = {-99, 1};


void center_quad(int16_t quad_coord[], int16_t target_coord[], int16_t target_centered_coord[]) {
    for (uint8_t i = 0; i < 2; ++i) {
        target_centered_coord[i] = target_coord[i] - quad_coord[i];
    }
}

void rotate_axes(float quad_rotation, int16_t target_coord[], int16_t target_rotated_coord[]) {
    target_rotated_coord[x_ndx] = (int16_t)((float)target_coord[x_ndx] * cos(quad_rotation)) - ((float)target_coord[y_ndx] * sin(quad_rotation));
    target_rotated_coord[y_ndx] = (int16_t)((float)target_coord[y_ndx] * cos(quad_rotation)) + ((float)target_coord[x_ndx] * sin(quad_rotation));
}

void calc_pr_coeff(int16_t target_coord[], float pr_coeff[]) {
    pr_coeff[pitch_ndx] = (float)target_coord[y_ndx] / (float)abs(target_coord[x_ndx]);
    pr_coeff[roll_ndx] = (float)target_coord[x_ndx] / (float)abs(target_coord[y_ndx]);
}

void calc_pr(float pr_coeff[], float pr[]) {
    pr[pitch_ndx] = pr_coeff[pitch_ndx] / (abs(pr_coeff[pitch_ndx]) + abs(pr_coeff[roll_ndx]));
    pr[roll_ndx] = pr_coeff[roll_ndx] / (abs(pr_coeff[pitch_ndx]) + abs(pr_coeff[roll_ndx]));
}

float calc_distance(int16_t target_coord[]) {
    return ( sqrt( pow(target_coord[x_ndx], 2) + pow(target_coord[y_ndx], 2) ) );
}

float p_ctrl(float setpoint, float input, float p) {
    return (input - setpoint) * p;
}

void calc_angles(float reaction, float pr[], float angles[]) {
    angles[pitch_ndx] = pr[pitch_ndx] * reaction;
    angles[roll_ndx] = pr[roll_ndx] * reaction;
}




int main() {
    cout << "quad rotation: " << quad_rotation_deg << "deg, " << quad_rotation_rad << "rad\n";
    cout << "x quad: " << quad_coord[x_ndx] << ", y quad: " << quad_coord[y_ndx] << "\n";
    cout << "x target: " << target_coord[x_ndx] << ", y target: " << target_coord[y_ndx] << "\n";
    
    int16_t target_centered_coord[2] = {-123, -123};
    center_quad(quad_coord, target_coord, target_centered_coord);
    cout << "x cent: " << target_centered_coord[x_ndx] << ", y cent: " << target_centered_coord[y_ndx] << "\n";
    
    int16_t target_rotated_coord[2] = {-123, -123};
    rotate_axes(-quad_rotation_rad, target_centered_coord, target_rotated_coord);
    cout << "x rot: " << target_rotated_coord[x_ndx] << ", y rot: " << target_rotated_coord[y_ndx] << "\n";
    
    float pr_coeff[2] = {-123.123f, -123.123f};
    calc_pr_coeff(target_rotated_coord, pr_coeff);
    cout << "pitch coeff: " << pr_coeff[pitch_ndx] << ", roll coeff: " << pr_coeff[roll_ndx] << "\n";
    
    float pr[2] = {-123.123f, -123.123f};
    calc_pr(pr_coeff, pr);
    cout << "pitch mult: " << pr[pitch_ndx] << ", roll mult: " << pr[roll_ndx] << "\n";
    
    float distance = calc_distance(target_rotated_coord);
    float reaction = p_ctrl(0.0f, distance, 1.0f);
    
    cout << "distance: " << distance << "\n";
    cout << "reaction: " << reaction << "\n";
    
    float angles[2] = {-123.123f, -123.123f};
    calc_angles(reaction, pr, angles);
    cout << "pitch angle: " << angles[pitch_ndx] << ", roll angle: " << angles[roll_ndx] << "\n";

    return 0;
}
