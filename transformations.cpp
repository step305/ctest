/*
 * transformations.c
 *
 *  Created on: May 27, 2019
 *      Author: VM
 */

#include "transformations.h"

//Euler angles to DCM
void angle2dcm(float Cbn[3][3], float rx, float ry, float rz)
{
	float sz = sinf(rz);
	float cz = cosf(rz);
	float sy = sinf(ry);
	float cy = cosf(ry);
	float sx = sinf(rx);
	float cx = cosf(rx);

	float Cx[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, cx, sx}, {0.0f, -sx, cx}};
	float Cy[3][3] = {{cy, 0.0f, -sy}, {0.0f, 1.0f, 0.0f}, {sy, 0.0f, cy}};
	float Cz[3][3] = {{cz, sz, 0.0f}, {-sz, cz, 0.0f}, {0.0f, 0.0f, 1.0f}};
	float C[3][3];

	mat_mult((float *)Cx, (float *)Cy, (float *)C, 1.0f, 3, 3, 3);
	mat_mult((float *)C, (float *)Cz, (float *)Cbn, 1.0f, 3, 3, 3);
}

//DCM to Euler angles
void dcm_angle(float r[3], float Cbn[3][3])
{
	r[2] = atan2f(Cbn[0][1], Cbn[0][0]);
	if (Cbn[0][2]*Cbn[0][2] < 1.0f)
		r[1] = -atan2f(Cbn[0][2], sqrtf(1.0f-Cbn[0][2]*Cbn[0][2]));
	else
		r[1] = -atan2f(Cbn[0][2], 0.0f);
	r[0] = atan2f(Cbn[1][2], Cbn[2][2]);
}

//Quaternion Multiplication
void quat_mult(float q[4], float l[4], float m[4])
{
	q[0] = l[0]*m[0] - l[1]*m[1] - l[2]*m[2] - l[3]*m[3];
	q[1] = l[0]*m[1] + l[1]*m[0] + l[2]*m[3] - l[3]*m[2];
	q[2] = l[0]*m[2] + l[2]*m[0] - l[1]*m[3] + l[3]*m[1];
	q[3] = l[0]*m[3] + l[1]*m[2] - l[2]*m[1] + l[3]*m[0];
}

//Quaternion to DCM
void quat2dcm(float Cbn[3][3], float q[4])
{
	Cbn[0][0] = q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3];
	Cbn[0][1] = 2.0f*(q[1]*q[2]+q[0]*q[3]);
	Cbn[0][2] = 2.0f*(q[1]*q[3]-q[0]*q[2]);
	Cbn[1][0] = 2.0f*(q[1]*q[2]-q[0]*q[3]);
	Cbn[1][1] = q[0]*q[0]-q[1]*q[1]+q[2]*q[2]-q[3]*q[3];
	Cbn[1][2] = 2.0f*(q[2]*q[3]+q[0]*q[1]);
	Cbn[2][0] = 2.0f*(q[1]*q[3]+q[0]*q[2]);
	Cbn[2][1] = 2.0f*(q[2]*q[3]-q[0]*q[1]);
	Cbn[2][2] = q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3];
}

//Skew-symmetric 3x3 matrix
void skew(float C[3][3], float x[3])
{
	C[0][0] =  0.0f;
	C[0][1] = -x[2];
	C[0][2] = -x[1];
	C[1][0] =  x[2];
	C[1][1] =  0.0f;
	C[1][2] = -x[0];
	C[2][0] = -x[1];
	C[2][1] =  x[0];
	C[2][2] =  0.0f;
}

//Rotate Vector using Quaternion
void quat_rot(float n[3], float q[4], float b[3])
{
	float Cbn[3][3];
	quat2dcm(Cbn, q);
	n[0] = b[0]*Cbn[0][0] + b[1]*Cbn[0][1] + b[2]*Cbn[0][2];
	n[1] = b[0]*Cbn[1][0] + b[1]*Cbn[1][1] + b[2]*Cbn[1][2];
	n[2] = b[0]*Cbn[2][0] + b[1]*Cbn[2][1] + b[2]*Cbn[2][2];
}

//Quaternion to Euler Angles
void quat2angle(float q[4], float r[3])
{
	float Cbn[3][3];
	quat2dcm(Cbn, q);
	dcm_angle(r, Cbn);
}

//Quaternion Conjugate
void quatconj(float q[4])
{
	int i;
	for(i=1;i<4;i++)
		q[i] *= -1.0f;
}

