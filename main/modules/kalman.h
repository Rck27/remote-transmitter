#ifndef KALMAN_H
#define KALMAN_H

#define NUM_VAR 6 

extern float Q[NUM_VAR];  
extern float R[NUM_VAR];  
extern float X[NUM_VAR];  
extern float P[NUM_VAR];  
extern float K[NUM_VAR];  

float kalmanFilterMulti(float measurement, int index);

#endif 
