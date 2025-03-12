#ifndef CANNY_EDGE_H
#define CANNY_EDGE_H

#include "std.h"

void canny_edge_init(void);
void canny_edge_detect(uint8_t *image, int width, int height);

#endif /* CANNY_EDGE_H */

