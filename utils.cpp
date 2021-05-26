/*
 * utils.cpp
 *
 *  Created on: Jul 2, 2019
 *      Author: mems
 */

#include "utils.hpp"

//Copy row of Mat to uchar array
void get_descriptor(unsigned char *desc, uchar *a)
{
	memcpy(a,desc,sizeof(uchar)*32);
}

//Copy row of Mat to uchar array
void get_descriptor32(unsigned char *desc, int32_t *a)
{
	memcpy(a,desc,sizeof(uchar)*32);
}

