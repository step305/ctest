/*
 * matrix.c
 *
 *  Created on: May 27, 2019
 *      Author: VM
 */

#include "matrix.h"

//Matrix Multiplication C = alpha*(A*B)
void mat_mult(float *A, float *B, float *C, float alpha, int m, int n, int l)
{
	int i, j, k;
	for (i = 0; i < m; i++)
		for (j = 0; j < l; j++) {
			C[l*i+j] = 0;
			for (k = 0; k < n; k++)
				C[l*i+j] += alpha*A[n*i+k]*B[l*k+j];
		}
}

//Matrix Addition C=A+alpha*B
void mat_add(float *A, float *B, float *C, float alpha, int m, int n)
{
	int i, j;
	for (i = 0; i < m; i++)
		for (j = 0; j < n; j++)
			C[n*i+j] = A[n*i+j]+(alpha*B[n*i+j]);
}

//Matrix Transpose B = alpha*A'
void mat_transpose(float *A, float *B, float alpha, int m, int n)
{
	int i, j;
	for (i = 0; i < m; i++)
		for (j = 0; j < n; j++)
			B[m*j+i] = alpha*A[n*i+j];
}

//Vector Norm
float vec_norm(float *v, int n)
{
	float sum = 0.0f;
	int i;
	for(i=0;i<n;i++)
		sum += v[i]*v[i];
	return sqrtf(sum);
}

//Normalize Vector
void vec_normalize(float *v, int n)
{
	int i;
	float norm = vec_norm(v, n);
	for(i=0;i<n;i++)
		v[i] /= norm;
}

//Cross product of two vectors
void cross(float a[3], float b[3], float alpha, float c[3])
{
    c[0] = alpha*(a[1]*b[2]-a[2]*b[1]);
    c[1] = alpha*(a[2]*b[0]-a[0]*b[2]);
    c[2] = alpha*(a[0]*b[1]-a[1]*b[0]);
}
