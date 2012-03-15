/*****************************************************************************
 * $Id:: cr_dsplib.h 3698 2010-06-10 23:29:17Z nxp27266                $
 *
 * Project: NXP LPC1000 Cortex-M3 family DSP library
 *
 * Description: Definition of DSP Library API
 *
 * Copyright(C) 2010, NXP Semiconductor
 * All rights reserved.
 *
 * Developed for NXP by Code Red Technologies Inc. www.code-red-tech.com
 *
 *****************************************************************************
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * NXP Semiconductors assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. NXP Semiconductors
 * reserves the right to make changes in the software without
 * notification. NXP Semiconductors also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.
 *****************************************************************************/
#ifndef MATRIX_MATH_H_
#define MATRIX_MATH_H_

void vectorCrossProduct(float* vectorC, float* vectorA, float* vectorB);

void matrixMultiply(int aRows, int aCols_bRows, int bCols, float* matrixC, float* matrixA, float* matrixB);

void matrixAdd(int rows, int cols, float* matrixC, float* matrixA, float* matrixB);
void matrixSubtract(int rows, int cols, float* matrixC, float* matrixA, float* matrixB);
void matrixScale(int rows, int cols, float* matrixC, float* matrixA, float scaler);
float vectorDotProduct(int length, float* vector1, float* vector2);

//Converts floating point to Q16.16 format
int floatToQ (float num);

//converts Q16.16 to floating point
float QToFloat(int num);

#endif
