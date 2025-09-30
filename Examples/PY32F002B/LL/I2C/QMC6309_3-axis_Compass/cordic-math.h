#pragma once

#include "stdint.h"

/**
 * @brief CORDIC_MATH_FRACTION_BITS is the number of bits represented by the decimals.
 * This variable can be changed but the recomended value is inbetween 8 and 22.
 * Default is set to 16.
 */
#define CORDIC_MATH_FRACTION_BITS 16
/**
 * @brief CORDIC_SPEED_FACTOR is the number of loops the Cordic algorithm goes through. 
 * Keep this variable between 8 and 15!
 * This variable can be lowered to get more speed and less accuracy. Default is 15.
 */
#define CORDIC_SPEED_FACTOR 15

typedef struct {
	int x;
	int y;
	int theta;
    int r;
} Coordinates;

int32_t cordic_atan(int32_t y, int32_t x);
int32_t cordic_hypotenuse(int32_t y, int32_t x);
int32_t cordic_cos(int32_t theta);
int32_t cordic_sin(int32_t theta);
int32_t cordic_asin(int32_t yInput);
int32_t cordic_acos(int32_t xInput);
int32_t cordic_tan(int32_t theta);
int32_t cordic_sqrt(int32_t x);
int32_t cordic_abs(int32_t input);
int32_t isEven(int32_t input);
int32_t isOdd(int32_t input);
int32_t to_degree(int32_t input);
int32_t to_radians(int32_t input);
int32_t cordic_arctanh(int32_t y, int32_t x);
int32_t cordic_ln(int32_t input);
int32_t cordic_arccosh(int32_t x);
int32_t cordic_arcsinh(int32_t y);
int32_t cordic_sinh(int32_t theta);
int32_t cordic_cosh(int32_t theta);
int32_t cordic_tanh(int32_t theta);
int32_t cordic_exp(int32_t exponent);
int32_t cordic_pow(int32_t base, int32_t exponent);
int32_t cordic_polar_rectangular(Coordinates *input);
int32_t cordic_rectangular_polar(Coordinates *input);