/*
 * utils.hpp
 *
 *  Created on: Jul 2, 2019
 *      Author: mems
 */

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>

void get_descriptor(unsigned char *desc, uchar *a);
void get_descriptor32(unsigned char *desc, int32_t *a);


#endif /* UTILS_HPP_ */
