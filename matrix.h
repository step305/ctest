/*
 * matrix.h
 *
 *  Created on: May 27, 2019
 *      Author: VM
 */

#ifndef MATRIX_H_
#define MATRIX_H_

#include <stdio.h>
#include <math.h>

void mat_mult(float *A, float *B, float *C, float alpha, int m, int n, int l);
void mat_add(float *A, float *B, float *C, float alpha, int m, int n);
void mat_transpose(float *A, float *B, float alpha, int m, int n);
void vec_normalize(float *v, int n);
float vec_norm(float *v, int n);
void cross(float a[3], float b[3], float alpha, float c[3]);

#endif /* MATRIX_H_ */
