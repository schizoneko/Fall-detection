#ifndef FORWARD_H
#define FORWARD_H

#include "activation_func.h"
#include "parameters.h"

#ifdef __cplusplus
extern "C" {
#endif

float forward(float input[INPUT_LAYER_SIZE]);

#ifdef __cplusplus
}
#endif

#endif // FORWARD_H
