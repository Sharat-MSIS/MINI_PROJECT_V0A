/*
 * cam_capture.h
 *
 *  Created on: Oct 25, 2026
 *      Author: SHARAT
 */

#ifndef CAM_CAPTURE_H
#define CAM_CAPTURE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* What the OV7670 sends (QQVGA) */
#define SENSOR_W 160
#define SENSOR_H 120

/* What we keep: centre window cropped by the DCMI hardware */
#define FRAME_W  96
#define FRAME_H  96

void cam_start(void);                 /* reset camera, load registers, set DCMI crop */
void cam_poll(void);                  /* handle one PC command from the main loop    */
int  cam_grab(void);                  /* capture one 96x96 frame, 0 = OK             */
const uint8_t *cam_frame(void);       /* pointer to the RGB565 frame (high byte 1st) */

/* Commands: C=send frame  R=capture+classify  T=colourbar  I=ID  D=diag
 *           G=pin activity  V/H/P=flip VSYNC/HSYNC/PCLK polarity           */

#ifdef __cplusplus
}
#endif
#endif
