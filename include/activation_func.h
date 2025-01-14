#ifndef ACTIVATION_FUNC_H
#define ACTIVATION_FUNC_H

#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

float sigmoid(float x);

float relu(float x);

#ifdef __cplusplus
}
#endif

#endif // ACTIVATION_FUNCTIONS_H