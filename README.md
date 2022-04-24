HW7_Mbed_DSP_Programming
===

# 目標
本次實作主要是結合Mbed-OS與CMSIS-DSP庫。首先需要先驗證實做的DSP的正確性，本次分別透過Matlab與CMSIS-DSP以FIR Filter處理弦波（包含1kHz與15kHz）訊號，並透過訊號雜訊比驗證我們是否正確使用CMSIS-DSP庫函式。接著透過一樣的FIR Filter來處理STM控制板所偵測的陀螺儀訊號，並將其記錄下來，進行對應處理或圖像化。

# 執行方式
1. File -> New Program -> empty Mbed OS program
2. 將main.cpp修改為[arm_fir_example_f32.c](https://github.com/ARM-software/CMSIS_5/tree/develop/CMSIS/DSP/Examples/ARM/arm_fir_example)
3. 新增[arm_fir_data.c](https://github.com/ARM-software/CMSIS_5/tree/develop/CMSIS/DSP/Examples/ARM/arm_fir_example)並改為.cpp檔
4. 新增[mbed-dsp](https://os.mbed.com/teams/mbed-official/code/mbed-dsp)的Library
5. 在/mbed-dsp/cmsis_dsp/TransformFunctions/arm_bitreversal2.S中`#if defined(__CC_ARM)`前新增`#define __CC_ARM`