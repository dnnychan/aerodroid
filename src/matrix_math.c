/*******************************************************************************
 * @file
 * @purpose        
 * @version        0.1
 *------------------------------------------------------------------------------
 * Copyright (C) 2012 Gumstix Inc.
 * All rights reserved.
 *
 * Contributer(s):
 *   Danny Chan   <danny@gumstix.com>
 *------------------------------------------------------------------------------
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 * 
 * Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include <math.h>

#include "matrix_math.h"

void vectorCrossProduct(float* vectorC, float* vectorA, float* vectorB)
{
  vectorC[0] = (vectorA[1] * vectorB[2]) - (vectorA[2] * vectorB[1]);
  vectorC[1] = (vectorA[2] * vectorB[0]) - (vectorA[0] * vectorB[2]);
  vectorC[2] = (vectorA[0] * vectorB[1]) - (vectorA[1] * vectorB[0]);
}

////////////////////////////////////////////////////////////////////////////////
//  Matrix Multiply
//  Multiply matrix A times matrix B, matrix A dimension m x n, matrix B dimension n x p
//  Result placed in matrix C, dimension m x p
//
//  Call as: matrixMultiply(m, n, p, C, A, B)
////////////////////////////////////////////////////////////////////////////////

void matrixMultiply(int aRows, int aCols_bRows, int bCols, float* matrixC, float* matrixA, float* matrixB)
{
  int i, j, k;

  for (i = 0; i < aRows * bCols; i++)
  {
    matrixC[i] = 0.0;
  }

  for (i = 0; i < aRows; i++)
  {
    for(j = 0; j < aCols_bRows; j++)
    {
      for(k = 0;  k < bCols; k++)
      {
       matrixC[i * bCols + k] += matrixA[i * aCols_bRows + j] * matrixB[j * bCols + k];
      }
    }
  }
}

void matrixAdd(int rows, int cols, float* matrixC, float* matrixA, float* matrixB)
{
  int i;

  for (i = 0; i < rows * cols; i++)
  {
    matrixC[i] = matrixA[i] + matrixB[i];
  }
}

void matrixSubtract(int rows, int cols, float* matrixC, float* matrixA, float* matrixB)
{
  int i;

  for (i = 0; i < rows * cols; i++)
  {
    matrixC[i] = matrixA[i] - matrixB[i];
  }
}

void matrixScale(int rows, int cols, float* matrixC, float* matrixA, float scaler)
{
  int i;

  for (i = 0; i < rows * cols; i++)
  {
    matrixC[i] = scaler * matrixA[i];
  }
}

float vectorDotProduct(int length, float* vector1, float* vector2)
{
  float dotProduct = 0;
  int   i;

  for (i = 0; i < length; i++)
  {
    dotProduct += vector1[i] * vector2[i];
  }

  return dotProduct;
}

//Converts floating point to Q16.16 format
int floatToQ (float num)
{
  return (int) (num * pow(2,16));
}

//converts Q16.16 to floating point
float QToFloat(int num)
{
  return pow(2,-16)*(float)num;
}
