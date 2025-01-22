#include "include/activation_func.h"

float sigmoid(float x) {
    return 1.0f / (1.0f + expf(-x));
}

float relu(float x) {
    return (x > 0) ? x : 0.0f;
}