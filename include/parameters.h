#ifndef PARAMETERS_H
#define PARAMETERS_H

#define INPUT_LAYER_SIZE 6
#define NUM_NEURONS_LAYER1 64
#define NUM_NEURONS_LAYER2 32
#define OUTPUT_LAYER_SIZE 1

extern float weights_layer_1[INPUT_LAYER_SIZE][NUM_NEURONS_LAYER1];
<<<<<<< HEAD
extern float bias_layer_1[NUM_NEURONS_LAYER1];
extern float weights_layer_2[NUM_NEURONS_LAYER1][NUM_NEURONS_LAYER2];
=======

extern float bias_layer_1[NUM_NEURONS_LAYER1];

extern float weights_layer_2[NUM_NEURONS_LAYER1][NUM_NEURONS_LAYER2];

>>>>>>> 772df6bd6e2665de6e4645aae0e0858849045bef
extern float bias_layer_2[NUM_NEURONS_LAYER2];
extern float weights_output[NUM_NEURONS_LAYER2][OUTPUT_LAYER_SIZE];
extern float bias_output[OUTPUT_LAYER_SIZE]; 

#endif // PARAMETERS_H
