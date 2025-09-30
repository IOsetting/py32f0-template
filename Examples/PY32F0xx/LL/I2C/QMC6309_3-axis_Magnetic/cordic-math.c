#include "cordic-math.h"


#define FLOAT_TO_INT(x) ((x) >= 0 ? (int)((x) + 0.5) : (int)((x)-0.5))

static const uint32_t EULER = FLOAT_TO_INT(2.71828182846 * (1 << CORDIC_MATH_FRACTION_BITS));
static const uint32_t CORDIC_GAIN = FLOAT_TO_INT(0.607253 * (1 << CORDIC_MATH_FRACTION_BITS));
static const uint32_t CORDIC_GAIN_HYPERBOLIC_VECTOR = FLOAT_TO_INT(0.82816 * (1 << CORDIC_MATH_FRACTION_BITS));
static const uint32_t CORDIC_GAIN_HYPERBOLIC_CIRCULAR = FLOAT_TO_INT(1.64676 * (1 << CORDIC_MATH_FRACTION_BITS));
static const uint32_t DECIMAL_TO_FP = (1 << CORDIC_MATH_FRACTION_BITS);
//static const uint32_t PI = FLOAT_TO_INT(3.14159265359 * (1 << CORDIC_MATH_FRACTION_BITS));
static const uint32_t ONE_EIGHTY_DIV_PI = FLOAT_TO_INT((180 / 3.14159265359) * (1 << CORDIC_MATH_FRACTION_BITS));
static const uint32_t ONE_DIV_CORDIC_GAIN_HYPERBOLIC = FLOAT_TO_INT((1.0 / 0.82816) * (1 << CORDIC_MATH_FRACTION_BITS));

static const uint32_t LUT_CORDIC_ATAN[15] =  {FLOAT_TO_INT(45.0000 * (1 << CORDIC_MATH_FRACTION_BITS)),  /* 45.000    degrees */
                                              FLOAT_TO_INT(26.5651 * (1 << CORDIC_MATH_FRACTION_BITS)),  /* 26.566    degrees */
                                              FLOAT_TO_INT(14.0362 * (1 << CORDIC_MATH_FRACTION_BITS)),  /* 26.566    degrees */
                                              FLOAT_TO_INT(7.1250  * (1 << CORDIC_MATH_FRACTION_BITS)),  /* 14.035    degrees */
                                              FLOAT_TO_INT(3.5763  * (1 << CORDIC_MATH_FRACTION_BITS)),  /* 3.578     degrees */
                                              FLOAT_TO_INT(1.7899  * (1 << CORDIC_MATH_FRACTION_BITS)),  /* 1.789     degrees */
                                              FLOAT_TO_INT(0.8952  * (1 << CORDIC_MATH_FRACTION_BITS)),  /* 0.894     degrees */
                                              FLOAT_TO_INT(0.4476  * (1 << CORDIC_MATH_FRACTION_BITS)),  /* 0.449     degrees */
                                              FLOAT_TO_INT(0.2238  * (1 << CORDIC_MATH_FRACTION_BITS)),  /* 0.223     degrees */
                                              FLOAT_TO_INT(0.1119  * (1 << CORDIC_MATH_FRACTION_BITS)),  /* 0.109     degrees */
                                              FLOAT_TO_INT(0.0560  * (1 << CORDIC_MATH_FRACTION_BITS)),  /* 0.055     degrees */
                                              FLOAT_TO_INT(0.0280  * (1 << CORDIC_MATH_FRACTION_BITS)),  /* 0.027     degrees */
                                              FLOAT_TO_INT(0.0140  * (1 << CORDIC_MATH_FRACTION_BITS)),  /* 0.016     degrees */
                                              FLOAT_TO_INT(0.0070  * (1 << CORDIC_MATH_FRACTION_BITS)),  /* 0.008     degrees */
                                              FLOAT_TO_INT(0.0035  * (1 << CORDIC_MATH_FRACTION_BITS))}; /* 0.004     degrees */

static const uint32_t LUT_CORDIC_ATANH[14] = {FLOAT_TO_INT(31.4729 * (1 << CORDIC_MATH_FRACTION_BITS)),  /* 31.473    degrees */
                                              FLOAT_TO_INT(14.6341 * (1 << CORDIC_MATH_FRACTION_BITS)),  /* 14.633    degrees */
                                              FLOAT_TO_INT(7.1996  * (1 << CORDIC_MATH_FRACTION_BITS)),  /* 7.199     degrees */
                                              FLOAT_TO_INT(3.5857  * (1 << CORDIC_MATH_FRACTION_BITS)),  /* 3.586     degrees */
                                              FLOAT_TO_INT(1.7911  * (1 << CORDIC_MATH_FRACTION_BITS)),  /* 1.793     degrees */
                                              FLOAT_TO_INT(0.8953  * (1 << CORDIC_MATH_FRACTION_BITS)),  /* 0.895     degrees */
                                              FLOAT_TO_INT(0.4476  * (1 << CORDIC_MATH_FRACTION_BITS)),  /* 0.449     degrees */
                                              FLOAT_TO_INT(0.2238  * (1 << CORDIC_MATH_FRACTION_BITS)),  /* 0.203     degrees */
                                              FLOAT_TO_INT(0.1119  * (1 << CORDIC_MATH_FRACTION_BITS)),  /* 0.113     degrees */
                                              FLOAT_TO_INT(0.0560  * (1 << CORDIC_MATH_FRACTION_BITS)),  /* 0.055     degrees */
                                              FLOAT_TO_INT(0.0280  * (1 << CORDIC_MATH_FRACTION_BITS)),  /* 0.027     degrees */
                                              FLOAT_TO_INT(0.0140  * (1 << CORDIC_MATH_FRACTION_BITS)),  /* 0.016     degrees */
                                              FLOAT_TO_INT(0.0070  * (1 << CORDIC_MATH_FRACTION_BITS)),  /* 0.008     degrees */
                                              FLOAT_TO_INT(0.0035  * (1 << CORDIC_MATH_FRACTION_BITS))}; /* 0.004     degrees */

/**
 * @brief Perform fixed-point multiplication.
 *
 * This function multiplies two fixed-point numbers. The result is
 * scaled down by the fixed-point scaling factor to maintain the correct
 * fixed-point representation.
 *
 * @param a Fixed-point number in the range defined by CORDIC_MATH_FRACTION_BITS.
 * @param b Fixed-point number in the range defined by CORDIC_MATH_FRACTION_BITS.
 * @return The product of the two numbers, scaled appropriately to fit the
 * fixed-point format.
 */
int32_t fixed_mul(int32_t a, int32_t b) {
    return (a * b) >> CORDIC_MATH_FRACTION_BITS;
}

/**
 * @brief Perform fixed-point division.
 *
 * This function divides one fixed-point number by another. The numerator
 * is scaled up by the fixed-point scaling factor to maintain precision
 * before performing the division.
 *
 * @param a Fixed-point numerator in the range defined by CORDIC_MATH_FRACTION_BITS.
 * @param b Fixed-point denominator in the range defined by CORDIC_MATH_FRACTION_BITS.
 * @return The quotient of the division, scaled appropriately to fit the
 * fixed-point format.
 */
int32_t fixed_div(int32_t a, int32_t b) {
    return (a << CORDIC_MATH_FRACTION_BITS) / b;
}


/*****************************************VECTORING MODE***********************************************/

/**
 * @brief Fast fixedpoint calculation of arcustangens using the cordic algorithm
 * 
 * @param y fixedpoint according to CORDIC_MATH_FRACTION_BITS, numerator, arctan(y/x)
 * @param x fixedpoint according to CORDIC_MATH_FRACTION_BITS, denominator, arctan(y/x)
 * 
 * @return 32 bit int fixedpoint according to CORDIC_MATH_FRACTION_BITS, arctan(y/x)
 */
int32_t cordic_atan(int32_t y, int32_t x) {
    int sumAngle = 0, tempX;
    if (x < 0) {
        x = -x;
        y = -y;
    }
    for (int i = 0; i < CORDIC_SPEED_FACTOR; i++) {
        tempX = x;
        if (y > 0) {
            /* Rotate clockwise */
            x += (y >> i);
            y -= (tempX >> i);
            sumAngle += LUT_CORDIC_ATAN[i];
        } else {
            /* Rotate counterclockwise */
            x -= (y >> i);
            y += (tempX >> i);
            sumAngle -= LUT_CORDIC_ATAN[i];
        }
    }
    return sumAngle;
}

/**
 * @brief fast fixedpoints calculation of hypotenuse using the cordic
 * algorithm
 *
 * @param y fixedpoint according to CORDIC_MATH_FRACTION_BITS
 * @param x fixedpoint according to CORDIC_MATH_FRACTION_BITS
 *
 * @return 32 bit int fixedpoint according to CORDIC_MATH_FRACTION_BITS, sqrt( x*x + y*y )
 */
int32_t cordic_hypotenuse(int32_t y, int32_t x) {
    int tempX;
    x = cordic_abs(x);
    y = cordic_abs(y);

    for (int i = 0; i < CORDIC_SPEED_FACTOR; i++) {
        tempX = x;
        if (y > 0) {
            /* Rotate clockwise */
            x += (y >> i);
            y -= (tempX >> i);
        } else {
            /* Rotate counterclockwise */
            x -= (y >> i);
            y += (tempX >> i);
        }
    }

    return ((long)x * CORDIC_GAIN) >> CORDIC_MATH_FRACTION_BITS;
}

/**
 * @brief Fast fixedpoint cossinus using the cordic algorithm
 *
 * @param theta, cos(theta), theta = fixedpoint according to CORDIC_MATH_FRACTION_BITS in degrees
 *
 * @return 32 bit int, cos of theta, fixedpoint according to CORDIC_MATH_FRACTION_BITS
 */
int32_t cordic_cos(int32_t theta) {
    int x = CORDIC_GAIN, y = 0, sumAngle = 0, tempX;

    theta %= (360 << CORDIC_MATH_FRACTION_BITS);

    if (theta > (90 << CORDIC_MATH_FRACTION_BITS)) {
        sumAngle = 180 << CORDIC_MATH_FRACTION_BITS;
    }
    if (theta > (270 << CORDIC_MATH_FRACTION_BITS)) {
        sumAngle = 360 << CORDIC_MATH_FRACTION_BITS;
    }

    for (int i = 0; i < CORDIC_SPEED_FACTOR; i++) {
        tempX = x;
        if (theta > sumAngle) {
            /* Rotate counter clockwise */
            x -= (y >> i);
            y += (tempX >> i);
            sumAngle += LUT_CORDIC_ATAN[i];
        } else {
            /* Rotate clockwise */
            x += (y >> i);
            y -= (tempX >> i);
            sumAngle -= LUT_CORDIC_ATAN[i];
        }
    }
    if (theta > (90 << CORDIC_MATH_FRACTION_BITS) &&
        theta < (270 << CORDIC_MATH_FRACTION_BITS)) {
        x = -x;
    }
    return x;
}

/**
 * @brief Fast fixedpoint sinus using the cordic algorithm
 *
 * @param theta, sin(theta), theta = fixedpoint according to CORDIC_MATH_FRACTION_BITS in degrees
 *
 * @return 32 bit int, sin of theta, fixedpoint according to CORDIC_MATH_FRACTION_BITS
 */
int32_t cordic_sin(int32_t theta) {
    int x = CORDIC_GAIN, y = 0, sumAngle = 0, tempX;

    theta %= (360 << CORDIC_MATH_FRACTION_BITS);

    if (theta > (90 << CORDIC_MATH_FRACTION_BITS)) {
        sumAngle = 180 << CORDIC_MATH_FRACTION_BITS;
    }
    if (theta > (270 << CORDIC_MATH_FRACTION_BITS)) {
        sumAngle = 360 << CORDIC_MATH_FRACTION_BITS;
    }

    for (int i = 0; i < CORDIC_SPEED_FACTOR; i++) {
        tempX = x;
        if (theta > sumAngle) {
            /* Rotate counter clockwise */
            x -= (y >> i);
            y += (tempX >> i);
            sumAngle += LUT_CORDIC_ATAN[i];
        } else {
            /* Rotate clockwise */
            x += (y >> i);
            y -= (tempX >> i);
            sumAngle -= LUT_CORDIC_ATAN[i];
        }
    }

    if (theta > (90 << CORDIC_MATH_FRACTION_BITS) &&
        theta < (270 << CORDIC_MATH_FRACTION_BITS)) {
        y = -y;
    }

    return y;
}

/**
 * @brief Fast fixedpoint arccosinus using the cordic algorithm
 *
 * @param yInput, arcsin(xInput), xInput = fixedpoint according to CORDIC_MATH_FRACTION_BITS
 *
 * @return 32 bit int, arcsin of yInput, fixedpoint according to CORDIC_MATH_FRACTION_BITS
 */
int32_t cordic_asin(int32_t input) {
    int x = CORDIC_GAIN, y = 0, sumAngle = 0, tempX,
        ninety = (90 << CORDIC_MATH_FRACTION_BITS);

    for (int i = 0; i < CORDIC_SPEED_FACTOR; i++) {
        tempX = x;
        if (y < input) {
            /* Rotate counter clockwise */
            x -= (y >> i);
            y += (tempX >> i);
            sumAngle += LUT_CORDIC_ATAN[i];
        } else {
            /* Rotate clockwise */
            x += (y >> i);
            y -= (tempX >> i);
            sumAngle -= LUT_CORDIC_ATAN[i];
        }
    }
    if (sumAngle < -ninety) {
        sumAngle = -ninety;
    } else if (sumAngle > ninety) {
        sumAngle = ninety;
    }
    return sumAngle;
}

/**
 * @brief Fast fixedpoint arccosinus using the cordic algorithm
 *
 * @param xInput, arccos(xInput), xInput = fixedpoint according to CORDIC_MATH_FRACTION_BITS
 *
 * @return 32 bit int, arccos of xInput, fixedpoint according to CORDIC_MATH_FRACTION_BITS
 */
int32_t cordic_acos(int32_t xInput) {
    int x = 0, y = CORDIC_GAIN, sumAngle = 90 << CORDIC_MATH_FRACTION_BITS,
        tempX;

    for (int i = 0; i < CORDIC_SPEED_FACTOR; i++) {
        tempX = x;
        if (x > xInput) {
            /* Rotate counter clockwise */
            x -= (y >> i);
            y += (tempX >> i);
            sumAngle += LUT_CORDIC_ATAN[i];
        } else {
            /* Rotate clockwise */
            x += (y >> i);
            y -= (tempX >> i);
            sumAngle -= LUT_CORDIC_ATAN[i];
        }
    }
    if (sumAngle > 180 * DECIMAL_TO_FP) {
        sumAngle = 180 * DECIMAL_TO_FP;
    } else if (sumAngle < 0) {
        sumAngle = 0;
    }
    return sumAngle;
}

/**
 * @brief Fast fixedpoint tan using the cordic algorithm
 *
 * @param theta, tan(degree), degree = fixedpoint according to CORDIC_MATH_FRACTION_BITS in degrees.
 *
 * @return 32 bit int, tan of degree, fixedpoint according to CORDIC_MATH_FRACTION_BITS
 */
int32_t cordic_tan(int32_t theta) {
        int x = CORDIC_GAIN, y = 0, sumAngle = 0, tempX;

    theta %= (360 << CORDIC_MATH_FRACTION_BITS);

    if (theta > (90 << CORDIC_MATH_FRACTION_BITS)) {
        sumAngle = 180 << CORDIC_MATH_FRACTION_BITS;
    }
    if (theta > (270 << CORDIC_MATH_FRACTION_BITS)) {
        sumAngle = 360 << CORDIC_MATH_FRACTION_BITS;
    }

    for (int i = 0; i < CORDIC_SPEED_FACTOR; i++) {
        tempX = x;
        if (theta > sumAngle) {
            /* Rotate counter clockwise */
            x -= (y >> i);
            y += (tempX >> i);
            sumAngle += LUT_CORDIC_ATAN[i];
        } else {
            /* Rotate clockwise */
            x += (y >> i);
            y -= (tempX >> i);
            sumAngle -= LUT_CORDIC_ATAN[i];
        }

    }

    if (theta > (90 << CORDIC_MATH_FRACTION_BITS) &&
        theta < (270 << CORDIC_MATH_FRACTION_BITS)) {
        x = -x;    
        y = -y;
    }

    return (y << CORDIC_MATH_FRACTION_BITS) / x;
}

/**
 * @brief Fast fixedpoint rectangular to polar conversion using the cordic algorithm
 * 
 * @param input, Coordinate struct pointer, x and y = fixedpoint coordinates according to CORDIC_MATH_FRACTION_BITS
 * 
 * @return 0, but struct r and theta will be the coordinates in polar form
 */
int32_t cordic_rectangular_polar(Coordinates *input) {
    int tempX, sumAngle = 0, x = input->x, y = input->y;
    if (x < 0 && y >= 0) {
        sumAngle = 90 * (1 << CORDIC_MATH_FRACTION_BITS);
        x = cordic_abs(x);
    } else if (x < 0 && y < 0) {
        sumAngle = 180 * (1 << CORDIC_MATH_FRACTION_BITS);
        x = cordic_abs(x);
        y = cordic_abs(y);
    }

    for (int i = 0; i < 15; i++) {
        tempX = x;
        if (y > 0) {
            /* Rotate clockwise */
            x += (y >> i);
            y -= (tempX >> i);
            sumAngle += LUT_CORDIC_ATAN[i];
        } else {
            /* Rotate counterclockwise */
            x -= (y >> i);
            y += (tempX >> i);
            sumAngle -= LUT_CORDIC_ATAN[i];
        }
    }
    input->theta = sumAngle;
    input->r = ((long)x * CORDIC_GAIN) >> CORDIC_MATH_FRACTION_BITS;
    return 0;
}

/**
 * @brief Fast fixedpoint polar to rectangular conversion using the cordic algorithm
 * 
 * @param input, Coordinate struct pointer, r and theta = fixedpoint coordinates according to CORDIC_MATH_FRACTION_BITS
 *  
 * @return 0, but struct x and y will be the coordinates in rectangular form
 */
int32_t cordic_polar_rectangular(Coordinates *input) {
    int tempX, sumAngle = 0, x = CORDIC_GAIN, y = 0;

    input->theta %= (360 << CORDIC_MATH_FRACTION_BITS);

    if (input->theta > (90 * (1 << CORDIC_MATH_FRACTION_BITS)) &&
        input->theta < (270 * (1 << CORDIC_MATH_FRACTION_BITS))) {

        sumAngle = 180 * (1 << CORDIC_MATH_FRACTION_BITS);
        x = -x;
    }
    for (int i = 0; i < 15; i++) {
        tempX = x;
        if (input->theta > sumAngle) {
            /* Rotate counter clockwise */
            x -= (y >> i);
            y += (tempX >> i);
            sumAngle += LUT_CORDIC_ATAN[i];
        } else {
            /* Rotate clockwise */
            x += (y >> i);
            y -= (tempX >> i);
            sumAngle -= LUT_CORDIC_ATAN[i];
        }
    }
    input->x = ((long)x * input->r) >> CORDIC_MATH_FRACTION_BITS;
    input->y = ((long)y * input->r) >> CORDIC_MATH_FRACTION_BITS;
    return 0;
}

/*****************************************HYPERBOLIC MODE***********************************************/

/**
 * @brief Fast fixedpoint calculation of squareroot using the cordic
 * algorithm
 *
 * @param x, sqrt(x), x = fixedpoint according to CORDIC_MATH_FRACTION_BITS
 *
 * @return 32 bit int, squareroot of x, fixedpoint according to CORDIC_MATH_FRACTION_BITS
 */
int32_t cordic_sqrt(int32_t x) {
    int poweroftwo;
    int y;

    if (x == 0) {
        return 0;
    }
    if (x == DECIMAL_TO_FP) {
        return DECIMAL_TO_FP;
    }
    poweroftwo = DECIMAL_TO_FP;

    if (x < DECIMAL_TO_FP) {
        while (x <=
               (((long)poweroftwo * poweroftwo) >> CORDIC_MATH_FRACTION_BITS)) {
            poweroftwo >>= 1;
        }
        y = poweroftwo;
    } else if (x > DECIMAL_TO_FP) {
        while ((((long)poweroftwo * poweroftwo) >> CORDIC_MATH_FRACTION_BITS) <=
               x) {
            poweroftwo <<= 1;
        }
        y = poweroftwo >> 1;
    }
    for (int i = 1; i <= CORDIC_SPEED_FACTOR; i++) {
        poweroftwo >>= 1;
        if (((long)(y + poweroftwo) * (y + poweroftwo) >>
             CORDIC_MATH_FRACTION_BITS) <= x) {
            y = y + poweroftwo;
        }
    }
    return y;
}

/**
 * @brief Fast fixedpoint calculation of arcustangens hyperbolic using
 * the cordic algorithm
 *
 * @param y fixedpoint according to CORDIC_MATH_FRACTION_BITS, numerator, arctanh(y/x)
 * @param x fixedpoint according to CORDIC_MATH_FRACTION_BITS, denominator, arctanh(y/x)
 * 
 * @return 32 bit int fixedpoint according to CORDIC_MATH_FRACTION_BITS, arctanh(y/x)
 */
int32_t cordic_arctanh(int32_t y, int32_t x) {
    int tempX, k = 4, sumAngle = 0;

    for (int i = 1; i < CORDIC_SPEED_FACTOR; i++) {
        tempX = x;
        if (y < 0) {
            /* Rotate clockwise */
            x += (y >> i);
            y += (tempX >> i);
            sumAngle -= LUT_CORDIC_ATANH[i - 1];
        } else {
            /* Rotate counterclockwise */
            x -= (y >> i);
            y -= (tempX >> i);
            sumAngle += LUT_CORDIC_ATANH[i - 1];
        }
        if (i == k) {
            k = (3 * k) + 1;
            tempX = x;
            if (y < 0) {
                /* Rotate clockwise */
                x += (y >> i);
                y += (tempX >> i);
                sumAngle -= LUT_CORDIC_ATANH[i - 1];
            } else {
                /* Rotate counterclockwise */
                x -= (y >> i);
                y -= (tempX >> i);
                sumAngle += LUT_CORDIC_ATANH[i - 1];
            }
        }
    }
    return sumAngle;
}

/**
 * @brief Fast fixedpoint calculation of natural logarithm using the
 * cordic algorithm
 *
 * @param input fixedpoint according to CORDIC_MATH_FRACTION_BITS, ln(input)
 *
 * @return 32 bit int fixedpoint according to CORDIC_MATH_FRACTION_BITS, ln(input)
 */
int32_t cordic_ln(int32_t input) {
    int k = 0;
    long calculate = input;

    while (calculate > EULER) {
        calculate <<= CORDIC_MATH_FRACTION_BITS;
        // printf("calculate : %ld\n", calculate);
        calculate /= EULER;
        k += DECIMAL_TO_FP;
    }

    int y = calculate - DECIMAL_TO_FP;
    int x = calculate + DECIMAL_TO_FP;

    return (to_radians(cordic_arctanh(y, x) << 1) + k);
}

/**
 * @brief Fast fixedpoint calculation of arccosinus hyperbollic using the
 * cordic algorithm
 *
 * @param x fixedpoint according to CORDIC_MATH_FRACTION_BITS, arccosh(x)
 *
 * @return 32 bit int fixedpoint according to CORDIC_MATH_FRACTION_BITS, arccosinus-hyperbollic(x)
 */
int32_t cordic_arccosh(int32_t x) {
    int tempX, k = 4, sumAngle = 0, y = DECIMAL_TO_FP, xt = x;

    for (int i = 1; i < CORDIC_SPEED_FACTOR; i++) {
        tempX = x;
        if (y < 0) {
            /* Rotate clockwise */
            x += (y >> i);
            y += (tempX >> i);
            sumAngle -= LUT_CORDIC_ATANH[i - 1];
        } else {
            /* Rotate counterclockwise */
            x -= (y >> i);
            y -= (tempX >> i);
            sumAngle += LUT_CORDIC_ATANH[i - 1];
        }
        if (i == k) {
            k = (3 * k) + 1;
            tempX = x;
            if (y < 0) {
                /* Rotate clockwise */
                x += (y >> i);
                y += (tempX >> i);
                sumAngle -= LUT_CORDIC_ATANH[i - 1];
            } else {
                /* Rotate counterclockwise */
                x -= (y >> i);
                y -= (tempX >> i);
                sumAngle += LUT_CORDIC_ATANH[i - 1];
            }
        }
    }

    return to_degree(cordic_ln((((long)x << CORDIC_MATH_FRACTION_BITS) /
                                CORDIC_GAIN_HYPERBOLIC_VECTOR) +
                               xt));
}

/**
 * @brief Fast fixedpoint calculation of arcsinus hyperbollic using the
 * cordic algorithm
 *
 * @param y fixedpoint according to CORDIC_MATH_FRACTION_BITS, arcsinh(y)
 *
 * @return 32 bit int fixedpoint according to CORDIC_MATH_FRACTION_BITS, arcsinus-hyperbollic(y)
 */
int32_t cordic_arcsinh(int32_t y) {
    int tempX, sumAngle = 0, x = DECIMAL_TO_FP, yt = y;

    for (int i = 0; i < CORDIC_SPEED_FACTOR; i++) {
        tempX = x;
        if (y < 0) {
            /* Rotate clockwise */
            x -= (y >> i);
            y += (tempX >> i);
            sumAngle -= LUT_CORDIC_ATAN[i];
        } else {
            /* Rotate counterclockwise */
            x += (y >> i);
            y -= (tempX >> i);
            sumAngle += LUT_CORDIC_ATAN[i];
        }
    }

    return to_degree(cordic_ln((((long)x << CORDIC_MATH_FRACTION_BITS) /
                                CORDIC_GAIN_HYPERBOLIC_CIRCULAR) +
                               yt));
}

/**
 * @brief Fast fixedpoint calculation of sinus hyperbollic using the
 * cordic algorithm
 *
 * @param theta Fixedpoint according to CORDIC_MATH_FRACTION_BITS in degrees, arcsinh(theta)
 *
 * @return 32 bit int fixedpoint according to CORDIC_MATH_FRACTION_BITS, sinus-hyperbollic(theta)
 */
int32_t cordic_sinh(int32_t theta) {
    int tempX, k = 4, sumAngle = theta, y = 0,
               x = ONE_DIV_CORDIC_GAIN_HYPERBOLIC;
    for (int i = 1; i < CORDIC_SPEED_FACTOR; i++) {
        tempX = x;
        if (sumAngle > 0) {
            /* Rotate clockwise */
            x += (y >> i);
            y += (tempX >> i);
            sumAngle -= LUT_CORDIC_ATANH[i - 1];
        } else {
            /* Rotate counterclockwise */
            x -= (y >> i);
            y -= (tempX >> i);
            sumAngle += LUT_CORDIC_ATANH[i - 1];
        }
        if (i == k) {
            k = (3 * k) + 1;
            tempX = x;
            if (sumAngle > 0) {
                /* Rotate clockwise */
                x += (y >> i);
                y += (tempX >> i);
                sumAngle -= LUT_CORDIC_ATANH[i - 1];
            } else {
                /* Rotate counterclockwise */
                x -= (y >> i);
                y -= (tempX >> i);
                sumAngle += LUT_CORDIC_ATANH[i - 1];
            }
        }
    }
    return y;
}

/**
 * @brief Fast fixedpoint calculation of cossinus hyperbollic using the
 * cordic algorithm
 *
 * @param theta Fixedpoint according to CORDIC_MATH_FRACTION_BITS in degrees, arccosh(theta)
 *
 * @return 32 bit int fixedpoint according to CORDIC_MATH_FRACTION_BITS, cossinus-hyperbollic(theta)
 */
int32_t cordic_cosh(int32_t theta) {
    int tempX, k = 4, sumAngle = theta, y = 0,
               x = ONE_DIV_CORDIC_GAIN_HYPERBOLIC;
    for (int i = 1; i < CORDIC_SPEED_FACTOR; i++) {
        tempX = x;
        if (sumAngle > 0) {
            /* Rotate clockwise */
            x += (y >> i);
            y += (tempX >> i);
            sumAngle -= LUT_CORDIC_ATANH[i - 1];
        } else {
            /* Rotate counterclockwise */
            x -= (y >> i);
            y -= (tempX >> i);
            sumAngle += LUT_CORDIC_ATANH[i - 1];
        }
        if (i == k) {
            k = (3 * k) + 1;
            tempX = x;
            if (sumAngle > 0) {
                /* Rotate clockwise */
                x += (y >> i);
                y += (tempX >> i);
                sumAngle -= LUT_CORDIC_ATANH[i - 1];
            } else {
                /* Rotate counterclockwise */
                x -= (y >> i);
                y -= (tempX >> i);
                sumAngle += LUT_CORDIC_ATANH[i - 1];
            }
        }
    }
    return x;
}

/**
 * @brief Fast fixedpoint calculation of tangens hyperbollic using the
 * cordic algorithm
 *
 * @param theta Fixedpoint according to CORDIC_MATH_FRACTION_BITS in degrees, tanh(theta)
 *
 * @return 32 bit int fixedpoint according to CORDIC_MATH_FRACTION_BITS, tangens-hyperbollic(theta)
 */
int32_t cordic_tanh(int32_t theta) {
    return (((long)cordic_sinh(theta) << CORDIC_MATH_FRACTION_BITS) /
            cordic_cosh(theta));
}

/**
 * @brief Fast fixedpoint calculation of e^x using the cordic algorithm
 *
 * @param exponent fixedpoint according to CORDIC_MATH_FRACTION_BITS exponent, e^exponent
 *
 * @return 32 bit int fixedpoint according to CORDIC_MATH_FRACTION_BITS, e^exponent
 */
int32_t cordic_exp(int32_t exponent) {
    int tempX, k = 4, sumAngle = to_degree(exponent),
               y = ONE_DIV_CORDIC_GAIN_HYPERBOLIC,
               x = ONE_DIV_CORDIC_GAIN_HYPERBOLIC, n = 0;

    while (sumAngle > ONE_EIGHTY_DIV_PI) {
        sumAngle -= ONE_EIGHTY_DIV_PI;
        n++;
    }

    while (sumAngle < 0) {
        sumAngle += ONE_EIGHTY_DIV_PI;
        n--;
    }

    for (int i = 1; i < CORDIC_SPEED_FACTOR; i++) {
        tempX = x;
        if (sumAngle > 0) {
            /* Rotate clockwise */
            x += (y >> i);
            y += (tempX >> i);
            sumAngle -= LUT_CORDIC_ATANH[i - 1];
        } else {
            /* Rotate counterclockwise */
            x -= (y >> i);
            y -= (tempX >> i);
            sumAngle += LUT_CORDIC_ATANH[i - 1];
        }
        if (i == k) {
            k = (3 * k) + 1;
            tempX = x;
            if (sumAngle > 0) {
                /* Rotate clockwise */
                x += (y >> i);
                y += (tempX >> i);
                sumAngle -= LUT_CORDIC_ATANH[i - 1];
            } else {
                /* Rotate counterclockwise */
                x -= (y >> i);
                y -= (tempX >> i);
                sumAngle += LUT_CORDIC_ATANH[i - 1];
            }
        }
    }

    y = DECIMAL_TO_FP;
    for (int i = 0; i < n; i++) {
        y = (int32_t)(((int64_t)y * EULER) >> CORDIC_MATH_FRACTION_BITS);
    }
    for (int i = 0; i > n; i--) {
        y = (int32_t)(((int64_t)y << CORDIC_MATH_FRACTION_BITS) / EULER);
    }

    return (int32_t)(((int64_t)x * y) >> CORDIC_MATH_FRACTION_BITS);
}

/**
 * @brief Fast fixedpoint calculation of a^x using the cordic algorithm
 *
 * @param base fixedpoint according to CORDIC_MATH_FRACTION_BITS base, base^exponent
 * @param exponent fixedpoint according to CORDIC_MATH_FRACTION_BITS base, base^exponent
 *
 * @return 32 bit int fixedpoint according to CORDIC_MATH_FRACTION_BITS, base^exponent
 */
int32_t cordic_pow(int32_t base, int32_t exponent) {
    int64_t ln_base = cordic_ln(base);
    int64_t product = (int64_t)exponent * ln_base;
    return (int32_t)cordic_exp((int32_t)(product >> CORDIC_MATH_FRACTION_BITS));
}

/**
 * @brief Fast calculation of absolute
 *
 * @param input int
 *
 * @return 32 bit int, |input|
 */
int32_t cordic_abs(int32_t input) {
    return (input >= 0) ? input : -input;
}

/**
 * @brief Calculation if the input is even
 *
 * @param input int
 *
 * @return 32 bit int
 */
int32_t isEven(int32_t input) {
    return (input % 2) == 0;
}

/**
 * @brief Calculation if the input is odd
 *
 * @param input int
 *
 * @return 32 bit int
 */
int32_t isOdd(int32_t input) {
    return (input % 2) != 0;
}

/**
 * @brief Converts radians to degrees
 *
 * @param input fixedpoint according to CORDIC_MATH_FRACTION_BITS in radians
 *
 * @return 32 bit int fixedpoint according to CORDIC_MATH_FRACTION_BITS in degrees
 */
int32_t to_degree(int32_t input) {
    return ((long)input * ONE_EIGHTY_DIV_PI >> CORDIC_MATH_FRACTION_BITS);
}

/**
 * @brief Converts degrees to radians
 *
 * @param input fixedpoint according to CORDIC_MATH_FRACTION_BITS in degrees
 *
 * @return 32 bit int fixedpoint according to CORDIC_MATH_FRACTION_BITS in degrees
 */
int32_t to_radians(int32_t input) {
    return (((long)input << CORDIC_MATH_FRACTION_BITS) / ONE_EIGHTY_DIV_PI);
}
