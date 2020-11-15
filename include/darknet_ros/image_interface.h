/*
 * image_interface.h
 *
 *  Created on: Dec 19, 2016
 *      Author: Marko Bjelonic
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#ifndef IMAGE_INTERFACE_H
#define IMAGE_INTERFACE_H

#include "image.h"

// EDIT : START : cv::Mat dependencies and function definitions
// OpenCv
#include <cv_bridge/cv_bridge.h>
#ifdef __cplusplus
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#endif

static float get_pixel(image m, int x, int y, int c);
image** load_alphabet_with_file(char* datafile);
void generate_image(image p, cv::Mat* disp);  // TODO : Should i remove the pointer?

#endif
// EDIT : START : cv::Mat dependencies and function definitions
