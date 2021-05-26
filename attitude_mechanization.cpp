/*
 * attitude_mechanization.c
 *
 *  Created on: May 30, 2019
 *      Author: VM
 */

#include "attitude_mechanization.h"

//Attitude Mechanization
void attitude_mechanization( float q[4], float dThe[3], float dt )
{
	float dw[3];
	for (int i=0; i<3; ++i)
		dw[i] = dThe[i];//*dt;
	float gam = vec_norm((float *)dw, 3);
	float sgam = 0.5f - powf( gam, 2.0f ) / 48.0f;
	float cgam = 1.0f - powf( gam, 2.0f ) / 8.0f + powf( gam, 4.0f ) / 384.0f;
	float lam[4];
	if (gam > 1e-16f) {
		lam[0] = cgam;
		lam[1] = -dw[0]*sgam / gam;
		lam[2] = -dw[1]*sgam / gam;
		lam[3] = -dw[2]*sgam / gam;
	} else {
		lam[0] = 1.0f;
		lam[1] = 0.0f;
		lam[2] = 0.0f;
		lam[3] = 0.0f;
	}
	float qtemp[4];
	memcpy( qtemp, q, sizeof(float)*4 );
	quat_mult( q, lam, qtemp );
	vec_normalize( (float *)q, 4 );
}
