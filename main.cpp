// #include "mbed.h"

// #include "stm32l475e_iot01_gyro.h"
// #include "stm32l475e_iot01_accelero.h"
// #include <cstdint>
// #include <cstdio>

// //-------------------FIR--------------------//
// #include "arm_math.h"
// #include "math_helper.h"

// #include <stdio.h>

// // Macro Defines
// #define TEST_LENGTH_SAMPLES  1024
// /*
// This SNR is a bit small. Need to understand why
// this example is not giving better SNR ...
// */
// #define SNR_THRESHOLD_F32    75.0f
// #define BLOCK_SIZE            32

// #if defined(ARM_MATH_MVEF) && !defined(ARM_MATH_AUTOVECTORIZE)
// /* Must be a multiple of 16 */
// #define NUM_TAPS_ARRAY_SIZE              32
// #else
// #define NUM_TAPS_ARRAY_SIZE              29
// #endif
// #define NUM_TAPS              29

// // Declare State buffer of size (numTaps + blockSize - 1)
// #if defined(ARM_MATH_MVEF) && !defined(ARM_MATH_AUTOVECTORIZE)
// static float32_t firStateF32[2 * BLOCK_SIZE + NUM_TAPS - 1];
// #else
// static float32_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1];
// #endif

// // FIR Coefficients buffer generated using fir1() MATLAB function.
// // fir1(28, 6/24)
// #if defined(ARM_MATH_MVEF) && !defined(ARM_MATH_AUTOVECTORIZE)
// const float32_t firCoeffs32[NUM_TAPS_ARRAY_SIZE] = {
//   -0.0018225230f, -0.0015879294f, +0.0000000000f, +0.0036977508f, +0.0080754303f, +0.0085302217f, -0.0000000000f, -0.0173976984f,
//   -0.0341458607f, -0.0333591565f, +0.0000000000f, +0.0676308395f, +0.1522061835f, +0.2229246956f, +0.2504960933f, +0.2229246956f,
//   +0.1522061835f, +0.0676308395f, +0.0000000000f, -0.0333591565f, -0.0341458607f, -0.0173976984f, -0.0000000000f, +0.0085302217f,
//   +0.0080754303f, +0.0036977508f, +0.0000000000f, -0.0015879294f, -0.0018225230f, 0.0f,0.0f,0.0f
// };
// #else
// const float32_t firCoeffs32[NUM_TAPS_ARRAY_SIZE] = {
//   -0.0018225230f, -0.0015879294f, +0.0000000000f, +0.0036977508f, +0.0080754303f, +0.0085302217f, -0.0000000000f, -0.0173976984f,
//   -0.0341458607f, -0.0333591565f, +0.0000000000f, +0.0676308395f, +0.1522061835f, +0.2229246956f, +0.2504960933f, +0.2229246956f,
//   +0.1522061835f, +0.0676308395f, +0.0000000000f, -0.0333591565f, -0.0341458607f, -0.0173976984f, -0.0000000000f, +0.0085302217f,
//   +0.0080754303f, +0.0036977508f, +0.0000000000f, -0.0015879294f, -0.0018225230f
// };
// #endif

// // Global variables for FIR LPF Example
// uint32_t blockSize = BLOCK_SIZE;
// uint32_t numBlocks = TEST_LENGTH_SAMPLES/BLOCK_SIZE;
// //------------------------------------------//

// bool isCollecting = false;
// int16_t pDataXYZ[3] = {0};
// float pGyroDataXYZ[3] = {0};
// double totalAcc[3] = {0};
// float32_t velX[TEST_LENGTH_SAMPLES] = {0};
// float32_t velY[TEST_LENGTH_SAMPLES] = {0};
// float32_t velZ[TEST_LENGTH_SAMPLES] = {0};
// float32_t velOutX[TEST_LENGTH_SAMPLES] = {0};
// float32_t velOutY[TEST_LENGTH_SAMPLES] = {0};
// float32_t velOutZ[TEST_LENGTH_SAMPLES] = {0};
// uint32_t counter = 0;

// arm_fir_instance_f32 SX;
// arm_fir_instance_f32 SY;
// arm_fir_instance_f32 SZ;

// float32_t vX = 0;
// float32_t vY = 0;
// float32_t vZ = 0;

// float32_t  *inputXF32, *outputXF32, *inputYF32, *outputYF32, *inputZF32, *outputZF32;

// int16_t avgDataXYZ[3] = {0};

// Semaphore print_sem(1);
// InterruptIn button(BUTTON1);
// EventQueue *queue = mbed_event_queue();

// void print_acc(void){
//     // float32_t vX = 0;
//     // float32_t vY = 0;
//     // float32_t vZ = 0;

//     // for(uint32_t i=0; i < numBlocks; i++)
//     // {
//     //     arm_fir_f32(&SX, inputXF32 + (i * blockSize), outputXF32 + (i * blockSize), blockSize);
//     //     arm_fir_f32(&SY, inputYF32 + (i * blockSize), outputYF32 + (i * blockSize), blockSize);
//     //     arm_fir_f32(&SZ, inputZF32 + (i * blockSize), outputZF32 + (i * blockSize), blockSize);
//     // }

//     // for(uint16_t i=0; i < TEST_LENGTH_SAMPLES; i++){
//     //     // vX += accOutX[i];
//     //     // vY += accOutY[i];
//     //     // vZ += accOutZ[i];

//     //     // vX += accX[i];
//     //     // vY += accY[i];
//     //     // vZ += accZ[i];

//     //     // totalAcc[0] += vX;
//     //     // totalAcc[1] += vY;
//     //     // totalAcc[2] += vZ;
//     //     // accX[i] = 0;
//     //     // accY[i] = 0;
//     //     // accZ[i] = 0;
//     //     // accOutX[i] = 0;
//     //     // accOutY[i] = 0;
//     //     // accOutZ[i] = 0;
//     // }

//     // for(uint32_t i=0; i < numBlocks; i++)
//     // {
//     //     arm_fir_f32(&SX, inputXF32 + (i * blockSize), outputXF32 + (i * blockSize), blockSize);
//     //     arm_fir_f32(&SY, inputYF32 + (i * blockSize), outputYF32 + (i * blockSize), blockSize);
//     //     arm_fir_f32(&SZ, inputZF32 + (i * blockSize), outputZF32 + (i * blockSize), blockSize);
//     // }

//     // for(uint32_t i=0; i < TEST_LENGTH_SAMPLES; i++){
//     //     totalAcc[0] += velOutX[i];
//     //     totalAcc[1] += velOutY[i];
//     //     totalAcc[2] += velOutZ[i];
//     //     velX[i] = 0;
//     //     velY[i] = 0;
//     //     velZ[i] = 0;
//     //     velOutX[i] = 0;
//     //     velOutY[i] = 0;
//     //     velOutZ[i] = 0;
//     // }
//     for(uint32_t i=0; i < TEST_LENGTH_SAMPLES; i++){
//         velX[i] = 0;
//         velY[i] = 0;
//         velZ[i] = 0;
//         velOutX[i] = 0;
//         velOutY[i] = 0;
//         velOutZ[i] = 0;
//     }

//     print_sem.acquire();
//     printf("\nTOTAL_X = %.2lf, TOTAL_Y = %.2lf, TOTAL_Z = %.2lf\n", totalAcc[0], totalAcc[1], totalAcc[2]);
//     print_sem.release();
//     totalAcc[0] = 0;
//     totalAcc[1] = 0;
//     totalAcc[2] = 0;
//     vX = 0;
//     vY = 0;
//     vZ = 0;
//     isCollecting = false;
//     counter = 0;
// }

// void collecting_data(void){
//     if(isCollecting){
//         // print_sem.acquire();
//         // BSP_GYRO_GetXYZ(pGyroDataXYZ);
//         // printf("\nGYRO_X = %.2f, GYRO_Y = %.2f, GYRO_Z = %.2f\n", pGyroDataXYZ[0], pGyroDataXYZ[1], pGyroDataXYZ[2]);
//         BSP_ACCELERO_AccGetXYZ(pDataXYZ);
//         // printf("\nACCELERO_X = %d, ACCELERO_Y = %d, ACCELERO_Z = %d\n", pDataXYZ[0], pDataXYZ[1], pDataXYZ[2]);

//         // totalAcc[0] += pDataXYZ[0]*0.01;
//         // totalAcc[1] += pDataXYZ[1]*0.01;
//         // totalAcc[2] += pDataXYZ[2]*0.01;
//         // if(counter!=TEST_LENGTH_SAMPLES){
//         //     accX[counter] = pDataXYZ[0]*0.01;
//         //     accY[counter] = pDataXYZ[1]*0.01;
//         //     accZ[counter] = pDataXYZ[2]*0.01;
//         //     counter++;
//         // }
//         vX += (pDataXYZ[0]-avgDataXYZ[0])*0.0001;
//         vY += (pDataXYZ[1]-avgDataXYZ[1])*0.0001;
//         vZ += (pDataXYZ[2]-avgDataXYZ[2])*0.0001;

//         if(counter!=TEST_LENGTH_SAMPLES){
//             velX[counter] = vX;
//             velY[counter] = vY;
//             velZ[counter] = vZ;
//             counter++;
//         }
//         else{
//             for(uint32_t i=0; i < numBlocks; i++)
//             {
//                 arm_fir_f32(&SX, inputXF32 + (i * blockSize), outputXF32 + (i * blockSize), blockSize);
//                 arm_fir_f32(&SY, inputYF32 + (i * blockSize), outputYF32 + (i * blockSize), blockSize);
//                 arm_fir_f32(&SZ, inputZF32 + (i * blockSize), outputZF32 + (i * blockSize), blockSize);
//             }

//             for(uint32_t i=0; i < TEST_LENGTH_SAMPLES; i++){
//                 totalAcc[0] += velOutX[i];
//                 totalAcc[1] += velOutY[i];
//                 totalAcc[2] += velOutZ[i];
//                 velX[i] = 0;
//                 velY[i] = 0;
//                 velZ[i] = 0;
//                 velOutX[i] = 0;
//                 velOutY[i] = 0;
//                 velOutZ[i] = 0;
//             }
//             counter = 0;
//         }
//         // totalAcc[0] += vX;
//         // totalAcc[1] += vY;
//         // totalAcc[2] += vZ;

//         // print_sem.release();
//     }
// }

// void button_pressed(void){
//     if(!isCollecting){
//         isCollecting = true;
//     }
//     else{
//         queue->call(print_acc);
//     }
// }

// // main() runs in its own thread in the OS
// int main()
// {
//     /* Initialize input and output buffer pointers */
//     // inputXF32 = &accX[0];
//     // outputXF32 = &accOutX[0];

//     // inputYF32 = &accY[0];
//     // outputYF32 = &accOutY[0];

//     // inputZF32 = &accZ[0];
//     // outputZF32 = &accOutZ[0];

//     inputXF32 = &velX[0];
//     outputXF32 = &velOutX[0];

//     inputYF32 = &velY[0];
//     outputYF32 = &velOutY[0];

//     inputZF32 = &velZ[0];
//     outputZF32 = &velOutZ[0];

//     arm_fir_init_f32(&SX, NUM_TAPS, (float32_t *)&firCoeffs32[0], &firStateF32[0], blockSize);
//     arm_fir_init_f32(&SY, NUM_TAPS, (float32_t *)&firCoeffs32[0], &firStateF32[0], blockSize);
//     arm_fir_init_f32(&SZ, NUM_TAPS, (float32_t *)&firCoeffs32[0], &firStateF32[0], blockSize);

//     BSP_GYRO_Init();
//     BSP_ACCELERO_Init();

//     int64_t avgDataXYZSum[3] = {0};
//     for(uint32_t i=0; i<TEST_LENGTH_SAMPLES; i++){
//         BSP_ACCELERO_AccGetXYZ(pDataXYZ);
//         avgDataXYZSum[0] += pDataXYZ[0];
//         avgDataXYZSum[1] += pDataXYZ[1];
//         avgDataXYZSum[2] += pDataXYZ[2];
//         ThisThread::sleep_for(1ms);   
//     }
//     avgDataXYZ[0] = avgDataXYZSum[0]/(TEST_LENGTH_SAMPLES);
//     avgDataXYZ[1] = avgDataXYZSum[1]/(TEST_LENGTH_SAMPLES);
//     avgDataXYZ[2] = avgDataXYZSum[2]/(TEST_LENGTH_SAMPLES);
//     printf("\nReady\n");

//     button.fall(button_pressed);

//     queue->call_every(
//         1ms,
//         collecting_data
//     );

//     queue->dispatch_forever();
// }


// Example of FIR

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
/*
This SNR is a bit small. Need to understand why
this example is not giving better SNR ...
*/
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

    // ofstream fileX;
    // fileX.open ("acceleroInputX.txt");
    // ofstream fileY;
    // fileY.open ("acceleroInputY.txt");
    // ofstream fileZ;
    // fileZ.open ("acceleroInputZ.txt");

    // fileX << "X_Input" << endl;
    // fileY << "Y_Input" << endl;
    // fileZ << "Z_Input" << endl;

    printf("Input\n");

    for(i=0; i<TEST_LENGTH_SAMPLES; i++){
        BSP_GYRO_GetXYZ(pGyroDataXYZ);

        // fileX << pGyroDataXYZ[0] << " ";
        // fileY << pGyroDataXYZ[1] << " ";
        // fileZ << pGyroDataXYZ[2] << " ";

        printf("[x: %f, y: %f, z: %f]\n", pGyroDataXYZ[0], pGyroDataXYZ[1], pGyroDataXYZ[2]);

        testInput_X[i] = pGyroDataXYZ[0];
        testInput_Y[i] = pGyroDataXYZ[1];
        testInput_Z[i] = pGyroDataXYZ[2];

        ThisThread::sleep_for(10ms);
    }

    for(i=0; i < numBlocks; i++){
        arm_fir_f32(&S, inputF32_X + (i * blockSize), outputF32_X + (i * blockSize), blockSize);
        arm_fir_f32(&S, inputF32_Y + (i * blockSize), outputF32_Y + (i * blockSize), blockSize);
        arm_fir_f32(&S, inputF32_Z + (i * blockSize), outputF32_Z + (i * blockSize), blockSize);
    }

    // fileX << endl << "X_Output" << endl;
    // fileY << endl << "Y_Output" << endl;
    // fileZ << endl << "Z_Output" << endl;

    printf("Output\n");

    for(i=0; i<TEST_LENGTH_SAMPLES; i++){
        // fileX << testOutput_X[i] << " ";
        // fileY << testOutput_Y[i] << " ";
        // fileZ << testOutput_Z[i] << " ";
        printf("[x: %f, y: %f, z: %f]\n", testOutput_X[i], testOutput_Y[i], testOutput_Z[i]);
    }

    // fileX.close();
    // fileY.close();
    // fileZ.close();

    printf("Finish\n");
}
