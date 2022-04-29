#include "mbed.h"

#include "arm_math.h"
#include "math_helper.h"

#include <cstdint>
#include <cstdio>
#include <stdio.h>

#include "stm32l475e_iot01_gyro.h"
#include <iostream>
#include <fstream>

// Macro Defines
#define TEST_LENGTH_SAMPLES  320
/*This SNR is a bit small. Need to understand why
this example is not giving better SNR ...*/
#define SNR_THRESHOLD_F32    75.0f
#define BLOCK_SIZE            32

#if defined(ARM_MATH_MVEF) && !defined(ARM_MATH_AUTOVECTORIZE)
/* Must be a multiple of 16 */
#define NUM_TAPS_ARRAY_SIZE              32
#else
#define NUM_TAPS_ARRAY_SIZE              29
#endif
#define NUM_TAPS              29

/* -------------------------------------------------------------------
 * The input signal and reference output (computed with MATLAB)
 * are defined externally in arm_fir_lpf_data.c.
 * ------------------------------------------------------------------- */
extern float32_t testInput_f32_1kHz_15kHz[TEST_LENGTH_SAMPLES];
extern float32_t refOutput[TEST_LENGTH_SAMPLES];

// Declare Test output buffer
static float32_t testOutput[TEST_LENGTH_SAMPLES];

// Declare State buffer of size (numTaps + blockSize - 1)
#if defined(ARM_MATH_MVEF) && !defined(ARM_MATH_AUTOVECTORIZE)
static float32_t firStateF32[2 * BLOCK_SIZE + NUM_TAPS - 1];
#else
static float32_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1];
#endif 

// FIR Coefficients buffer generated using fir1() MATLAB function.
// fir1(28, 6/24)
#if defined(ARM_MATH_MVEF) && !defined(ARM_MATH_AUTOVECTORIZE)
const float32_t firCoeffs32[NUM_TAPS_ARRAY_SIZE] = {
  -0.0018225230f, -0.0015879294f, +0.0000000000f, +0.0036977508f, +0.0080754303f, +0.0085302217f, -0.0000000000f, -0.0173976984f,
  -0.0341458607f, -0.0333591565f, +0.0000000000f, +0.0676308395f, +0.1522061835f, +0.2229246956f, +0.2504960933f, +0.2229246956f,
  +0.1522061835f, +0.0676308395f, +0.0000000000f, -0.0333591565f, -0.0341458607f, -0.0173976984f, -0.0000000000f, +0.0085302217f,
  +0.0080754303f, +0.0036977508f, +0.0000000000f, -0.0015879294f, -0.0018225230f, 0.0f,0.0f,0.0f
};
#else
const float32_t firCoeffs32[NUM_TAPS_ARRAY_SIZE] = {
  -0.0018225230f, -0.0015879294f, +0.0000000000f, +0.0036977508f, +0.0080754303f, +0.0085302217f, -0.0000000000f, -0.0173976984f,
  -0.0341458607f, -0.0333591565f, +0.0000000000f, +0.0676308395f, +0.1522061835f, +0.2229246956f, +0.2504960933f, +0.2229246956f,
  +0.1522061835f, +0.0676308395f, +0.0000000000f, -0.0333591565f, -0.0341458607f, -0.0173976984f, -0.0000000000f, +0.0085302217f,
  +0.0080754303f, +0.0036977508f, +0.0000000000f, -0.0015879294f, -0.0018225230f
};
#endif

// FIR for x
#if defined(ARM_MATH_MVEF) && !defined(ARM_MATH_AUTOVECTORIZE)
static float32_t xfirStateF32[2 * BLOCK_SIZE + NUM_TAPS - 1];
#else
static float32_t xfirStateF32[BLOCK_SIZE + NUM_TAPS - 1];
#endif 

#if defined(ARM_MATH_MVEF) && !defined(ARM_MATH_AUTOVECTORIZE)
const float32_t xfirCoeffs32[NUM_TAPS_ARRAY_SIZE] = {
  -0.0018225230f, -0.0015879294f, +0.0000000000f, +0.0036977508f, +0.0080754303f, +0.0085302217f, -0.0000000000f, -0.0173976984f,
  -0.0341458607f, -0.0333591565f, +0.0000000000f, +0.0676308395f, +0.1522061835f, +0.2229246956f, +0.2504960933f, +0.2229246956f,
  +0.1522061835f, +0.0676308395f, +0.0000000000f, -0.0333591565f, -0.0341458607f, -0.0173976984f, -0.0000000000f, +0.0085302217f,
  +0.0080754303f, +0.0036977508f, +0.0000000000f, -0.0015879294f, -0.0018225230f, 0.0f,0.0f,0.0f
};
#else
const float32_t xfirCoeffs32[NUM_TAPS_ARRAY_SIZE] = {
  -0.0018225230f, -0.0015879294f, +0.0000000000f, +0.0036977508f, +0.0080754303f, +0.0085302217f, -0.0000000000f, -0.0173976984f,
  -0.0341458607f, -0.0333591565f, +0.0000000000f, +0.0676308395f, +0.1522061835f, +0.2229246956f, +0.2504960933f, +0.2229246956f,
  +0.1522061835f, +0.0676308395f, +0.0000000000f, -0.0333591565f, -0.0341458607f, -0.0173976984f, -0.0000000000f, +0.0085302217f,
  +0.0080754303f, +0.0036977508f, +0.0000000000f, -0.0015879294f, -0.0018225230f
};
#endif

// FIR for y
#if defined(ARM_MATH_MVEF) && !defined(ARM_MATH_AUTOVECTORIZE)
static float32_t yfirStateF32[2 * BLOCK_SIZE + NUM_TAPS - 1];
#else
static float32_t yfirStateF32[BLOCK_SIZE + NUM_TAPS - 1];
#endif 

#if defined(ARM_MATH_MVEF) && !defined(ARM_MATH_AUTOVECTORIZE)
const float32_t yfirCoeffs32[NUM_TAPS_ARRAY_SIZE] = {
  -0.0018225230f, -0.0015879294f, +0.0000000000f, +0.0036977508f, +0.0080754303f, +0.0085302217f, -0.0000000000f, -0.0173976984f,
  -0.0341458607f, -0.0333591565f, +0.0000000000f, +0.0676308395f, +0.1522061835f, +0.2229246956f, +0.2504960933f, +0.2229246956f,
  +0.1522061835f, +0.0676308395f, +0.0000000000f, -0.0333591565f, -0.0341458607f, -0.0173976984f, -0.0000000000f, +0.0085302217f,
  +0.0080754303f, +0.0036977508f, +0.0000000000f, -0.0015879294f, -0.0018225230f, 0.0f,0.0f,0.0f
};
#else
const float32_t yfirCoeffs32[NUM_TAPS_ARRAY_SIZE] = {
  -0.0018225230f, -0.0015879294f, +0.0000000000f, +0.0036977508f, +0.0080754303f, +0.0085302217f, -0.0000000000f, -0.0173976984f,
  -0.0341458607f, -0.0333591565f, +0.0000000000f, +0.0676308395f, +0.1522061835f, +0.2229246956f, +0.2504960933f, +0.2229246956f,
  +0.1522061835f, +0.0676308395f, +0.0000000000f, -0.0333591565f, -0.0341458607f, -0.0173976984f, -0.0000000000f, +0.0085302217f,
  +0.0080754303f, +0.0036977508f, +0.0000000000f, -0.0015879294f, -0.0018225230f
};
#endif

// FIR for z
#if defined(ARM_MATH_MVEF) && !defined(ARM_MATH_AUTOVECTORIZE)
static float32_t zfirStateF32[2 * BLOCK_SIZE + NUM_TAPS - 1];
#else
static float32_t zfirStateF32[BLOCK_SIZE + NUM_TAPS - 1];
#endif 

#if defined(ARM_MATH_MVEF) && !defined(ARM_MATH_AUTOVECTORIZE)
const float32_t zfirCoeffs32[NUM_TAPS_ARRAY_SIZE] = {
  -0.0018225230f, -0.0015879294f, +0.0000000000f, +0.0036977508f, +0.0080754303f, +0.0085302217f, -0.0000000000f, -0.0173976984f,
  -0.0341458607f, -0.0333591565f, +0.0000000000f, +0.0676308395f, +0.1522061835f, +0.2229246956f, +0.2504960933f, +0.2229246956f,
  +0.1522061835f, +0.0676308395f, +0.0000000000f, -0.0333591565f, -0.0341458607f, -0.0173976984f, -0.0000000000f, +0.0085302217f,
  +0.0080754303f, +0.0036977508f, +0.0000000000f, -0.0015879294f, -0.0018225230f, 0.0f,0.0f,0.0f
};
#else
const float32_t zfirCoeffs32[NUM_TAPS_ARRAY_SIZE] = {
  -0.0018225230f, -0.0015879294f, +0.0000000000f, +0.0036977508f, +0.0080754303f, +0.0085302217f, -0.0000000000f, -0.0173976984f,
  -0.0341458607f, -0.0333591565f, +0.0000000000f, +0.0676308395f, +0.1522061835f, +0.2229246956f, +0.2504960933f, +0.2229246956f,
  +0.1522061835f, +0.0676308395f, +0.0000000000f, -0.0333591565f, -0.0341458607f, -0.0173976984f, -0.0000000000f, +0.0085302217f,
  +0.0080754303f, +0.0036977508f, +0.0000000000f, -0.0015879294f, -0.0018225230f
};
#endif

// Global variables for FIR LPF Example
uint32_t blockSize = BLOCK_SIZE;
uint32_t numBlocks = TEST_LENGTH_SAMPLES/BLOCK_SIZE;

float32_t  snr;

float32_t pGyroDataXYZ[3] = {0};

float32_t testInput_X[TEST_LENGTH_SAMPLES];
float32_t testInput_Y[TEST_LENGTH_SAMPLES];
float32_t testInput_Z[TEST_LENGTH_SAMPLES];

float32_t testOutput_X[TEST_LENGTH_SAMPLES];
float32_t testOutput_Y[TEST_LENGTH_SAMPLES];
float32_t testOutput_Z[TEST_LENGTH_SAMPLES];

// FIR LPF Example
int32_t main(void)
{
    uint32_t i;
    arm_fir_instance_f32 S;
    arm_status status;
    float32_t  *inputF32, *outputF32;

    /* Initialize input and output buffer pointers */
    inputF32 = &testInput_f32_1kHz_15kHz[0];
    outputF32 = &testOutput[0];

    /* Call FIR init function to initialize the instance structure. */
    arm_fir_init_f32(&S, NUM_TAPS, (float32_t *)&firCoeffs32[0], &firStateF32[0], blockSize);

    arm_fir_instance_f32 Sx;
    /* Call FIR init function to initialize the instance structure. */
    arm_fir_init_f32(&Sx, NUM_TAPS, (float32_t *)&xfirCoeffs32[0], &xfirStateF32[0], blockSize);

    arm_fir_instance_f32 Sy;
    /* Call FIR init function to initialize the instance structure. */
    arm_fir_init_f32(&Sy, NUM_TAPS, (float32_t *)&yfirCoeffs32[0], &yfirStateF32[0], blockSize);

    arm_fir_instance_f32 Sz;
    /* Call FIR init function to initialize the instance structure. */
    arm_fir_init_f32(&Sz, NUM_TAPS, (float32_t *)&zfirCoeffs32[0], &zfirStateF32[0], blockSize);


    /* ----------------------------------------------------------------------
    ** Call the FIR process function for every blockSize samples
    ** ------------------------------------------------------------------- */
    for(i=0; i < numBlocks; i++)
    {
        arm_fir_f32(&S, inputF32 + (i * blockSize), outputF32 + (i * blockSize), blockSize);
    }

    /* ----------------------------------------------------------------------
    ** Compare the generated output against the reference output computed
    ** in MATLAB.
    ** ------------------------------------------------------------------- */
    snr = arm_snr_f32(&refOutput[0], &testOutput[0], TEST_LENGTH_SAMPLES);

    status = (snr < SNR_THRESHOLD_F32) ? ARM_MATH_TEST_FAILURE : ARM_MATH_SUCCESS;
    
    if (status != ARM_MATH_SUCCESS)
    {
        printf("FAILURE\n");
    }
    else
    {
        printf("SUCCESS\n");
    }

    BSP_GYRO_Init();

    float32_t  *inputF32_X, *outputF32_X;
    inputF32_X = &testInput_X[0];
    outputF32_X = &testOutput_X[0];

    float32_t  *inputF32_Y, *outputF32_Y;
    inputF32_Y = &testInput_Y[0];
    outputF32_Y = &testOutput_Y[0];

    float32_t  *inputF32_Z, *outputF32_Z;
    inputF32_Z = &testInput_Z[0];
    outputF32_Z = &testOutput_Z[0];

    printf("Input\n");

    for(i=0; i<TEST_LENGTH_SAMPLES; i++){
        BSP_GYRO_GetXYZ(pGyroDataXYZ);

        printf("%f %f %f\n", pGyroDataXYZ[0], pGyroDataXYZ[1], pGyroDataXYZ[2]);

        testInput_X[i] = pGyroDataXYZ[0];
        testInput_Y[i] = pGyroDataXYZ[1];
        testInput_Z[i] = pGyroDataXYZ[2];

        ThisThread::sleep_for(10ms);
    }

    for(i=0; i < numBlocks; i++){
        arm_fir_f32(&Sx, inputF32_X + (i * blockSize), outputF32_X + (i * blockSize), blockSize);
        arm_fir_f32(&Sy, inputF32_Y + (i * blockSize), outputF32_Y + (i * blockSize), blockSize);
        arm_fir_f32(&Sz, inputF32_Z + (i * blockSize), outputF32_Z + (i * blockSize), blockSize);
    }

    printf("Output\n");

    for(i=0; i<TEST_LENGTH_SAMPLES; i++){
        printf("%f %f %f\n", testOutput_X[i], testOutput_Y[i], testOutput_Z[i]);
    }

    printf("Finish\n");
}
