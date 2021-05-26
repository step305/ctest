/*
 * attitude_mechanization.h
 *
 *  Created on: May 30, 2019
 *      Author: mactravel
 */

#ifndef ATTITUDE_MECHANIZATION_H_
#define ATTITUDE_MECHANIZATION_H_

#include <string.h>
#include <math.h>
#include "matrix.h"
#include "transformations.h"

void attitude_mechanization(float q[4], float dThe[3], float dt);
void coning_compensation(float dThe1[3], float dThe2[3], float dThe3[3], float dThe[3]);

#endif /* ATTITUDE_MECHANIZATION_H_ */
