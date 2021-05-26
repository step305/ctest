/*
 * transformations.h
 *
 *  Created on: May 27, 2019
 *      Author: VM
 */

#ifndef TRANSFORMATIONS_H_
#define TRANSFORMATIONS_H_
#include <math.h>
#include "matrix.h"

void angle2dcm(float Cbn[3][3], float rx, float ry, float rz);
void dcm_angle(float r[3], float Cbn[3][3]);
void quat_mult(float q[4], float l[4], float m[4]);
void quat2dcm(float Cbn[3][3], float q[4]);
void skew(float C[3][3], float x[3]);
void quat_rot(float n[3], float q[4], float b[3]);
void quat2angle(float q[4], float r[3]);
void quatconj(float q[4]);

#endif /* TRANSFORMATIONS_H_ */
