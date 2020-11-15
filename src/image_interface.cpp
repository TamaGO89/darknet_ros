/*
 * image_interface.c
 *
 *  Created on: Dec 19, 2016
 *      Author: Marko Bjelonic
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include "darknet_ros/image_interface.h"

static float get_pixel(image m, int x, int y, int c) {
  assert(x < m.w && y < m.h && c < m.c);
  return m.data[c * m.h * m.w + y * m.w + x];
}

image** load_alphabet_with_file(char* datafile) {
  int i, j;
  const int nsize = 8;
  image** alphabets = (image**)calloc(nsize, sizeof(image));  // EDIT : cast to (image**)
  char const* labels = "/labels/%d_%d.png";
  char* files = (char*)malloc(1 + strlen(datafile) + strlen(labels));
  strcpy(files, datafile);
  strcat(files, labels);
  for (j = 0; j < nsize; ++j) {
    alphabets[j] = (image*)calloc(128, sizeof(image));  // EDIT : cast to (image*)
    for (i = 32; i < 127; ++i) {
      char buff[256];
      sprintf(buff, files, i, j);
      alphabets[j][i] = load_image_color(buff, 0, 0);
    }
  }
  return alphabets;
}
// EDIT : START : generate_image args changed to cv::Mat
/*
#ifdef OPENCV
void generate_image(image p, IplImage* disp) {  // DONE : Remove dependency, generate_image will work with image and cv::Mat
  int x, y, k;
  if (p.c == 3) rgbgr_image(p);
  // normalize_image(copy);

  int step = disp->widthStep;
  for (y = 0; y < p.h; ++y) {
    for (x = 0; x < p.w; ++x) {
      for (k = 0; k < p.c; ++k) {
        disp->imageData[y * step + x * p.c + k] = (unsigned char)(get_pixel(p, x, y, k) * 255);
      }
    }
  }
}
#endif

#ifdef OPENCV  // TODO : Check if this version is better... i doubt it
void generate_image(image p, cv::Mat disp) {
  int x, y, k;
  if (p.c == 3) rgbgr_image(p);
  // normalize_image(copy);

  int step = disp.step[0];
  for (y = 0; y < p.h; ++y) {
    for (x = 0; x < p.w; ++x) {
      for (k = 0; k < p.c; ++k) {
        disp.data[y * step + x * p.c + k] = (unsigned char)(get_pixel(p, x, y, k) * 255);
      }
    }
  }
}
#endif
*/
#ifdef OPENCV
void generate_image(image im, cv::Mat* mat) {  // DONE : Remove dependency, generate_image will work with image and cv::Mat
  int w = mat->cols;
  int h = mat->rows;
  int c = mat->channels();
  im = make_image(w, h, c);
  unsigned char *data = (unsigned char *)mat->data;
  int step = mat->step;
  for (int y = 0; y < h; ++y) {
    for (int k = 0; k < c; ++k) {
      for (int x = 0; x < w; ++x) {
        im.data[k*w*h + y*w + x] = data[y*step + x*c + k] / 255.0f;
      }
    }
  }
}
#endif
// EDIT : END : generate_image args changed to cv::Mat