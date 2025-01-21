#include "forward.h"
#include <stdlib.h>
#include <stdio.h>

// Feed-forward
float forward(float input[INPUT_LAYER_SIZE]) {
    float layer1_output[NUM_NEURONS_LAYER1] = {0.0f};  // First hidden layer output
    float layer2_output[NUM_NEURONS_LAYER2] = {0.0f};  // Second hidden layer output
    float output = 0.0f;  // Final output

    // First hidden layer
    for (int i = 0; i < NUM_NEURONS_LAYER1; i++) {
        float sum = bias_layer_1[i];  // Bias
        for (int j = 0; j < INPUT_LAYER_SIZE; j++) {
            sum += input[j] * weights_layer_1[j][i]; // Output = input * weights + bias  
        }
        layer1_output[i] = relu(sum);  // ReLU
    }

    // Second hidden layer
    for (int i = 0; i < NUM_NEURONS_LAYER2; i++) {
        float sum = bias_layer_2[i];  // Bias
        for (int j = 0; j < NUM_NEURONS_LAYER1; j++) {
            sum += layer1_output[j] * weights_layer_2[j][i];
        }
        layer2_output[i] = relu(sum);
    }

    // Final output
    output = bias_output[0];
    for (int i = 0; i < NUM_NEURONS_LAYER2; i++) {
        output += layer2_output[i] * weights_output[i][0];
    }
    return sigmoid(output);  // Sigmoid
}