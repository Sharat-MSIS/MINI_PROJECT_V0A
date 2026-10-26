/*
 * cam_classify.cpp
 *
 *  Created on: Oct 25, 2026
 *      Author: SHARAT
 */

#include "cam_classify.h"
#include "cam_capture.h"
#include "main.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"

using namespace ei;

extern UART_HandleTypeDef huart2;

/* Set to 1 if red and blue look swapped in the camera image */
#define SWAP_RB 0

/* ---- printing (needed by the Edge Impulse SDK) ---- */
static void vprint(const char *fmt, va_list argp)
{
    char string[200];
    if (0 < vsnprintf(string, sizeof(string), fmt, argp)) {
        HAL_UART_Transmit(&huart2, (uint8_t *)string, strlen(string), 0xffffff);
    }
}

void ei_printf(const char *format, ...)
{
    va_list myargs;
    va_start(myargs, format);
    vprint(format, myargs);
    va_end(myargs);
}

/* ---- camera -> classifier ---- */
static const uint8_t *g_frame;

/* The classifier asks for 'length' pixels starting at 'offset'.
 * Each pixel = one float holding 0xRRGGBB. The frame is already 96x96. */
static int get_camera_data(size_t offset, size_t length, float *out)
{
    for (size_t i = 0; i < length; i++) {
        const uint8_t *p = g_frame + (offset + i) * 2;
        uint16_t v = ((uint16_t)p[0] << 8) | p[1];      /* high byte first */

        uint8_t r5 = (v >> 11) & 0x1F;
        uint8_t g6 = (v >> 5)  & 0x3F;
        uint8_t b5 =  v        & 0x1F;

        uint8_t r = (r5 << 3) | (r5 >> 2);
        uint8_t g = (g6 << 2) | (g6 >> 4);
        uint8_t b = (b5 << 3) | (b5 >> 2);
#if SWAP_RB
        uint8_t t = r; r = b; b = t;
#endif
        out[i] = (float)(((uint32_t)r << 16) | ((uint32_t)g << 8) | b);
    }
    return 0;
}

extern "C" void cam_classify_once(void)
{
    if (EI_CLASSIFIER_INPUT_WIDTH != FRAME_W || EI_CLASSIFIER_INPUT_HEIGHT != FRAME_H) {
        ei_printf("ERR: model input %dx%d but frame is %dx%d\r\n",
                  (int)EI_CLASSIFIER_INPUT_WIDTH, (int)EI_CLASSIFIER_INPUT_HEIGHT,
                  FRAME_W, FRAME_H);
        return;
    }

    if (cam_grab() != 0) {
        ei_printf("ERR: capture failed\r\n");
        return;
    }
    g_frame = cam_frame();

    signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &get_camera_data;

    ei_impulse_result_t result = {0};
    EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
    ei_printf("run_classifier returned: %d\r\n", (int)res);
    if (res != EI_IMPULSE_OK) return;

    ei_printf("DSP %d ms, Classification %d ms\r\n",
              (int)result.timing.dsp, (int)result.timing.classification);

    size_t best = 0;
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        ei_printf("  %s: ", result.classification[ix].label);
        ei_printf_float(result.classification[ix].value);
        ei_printf("\r\n");
        if (result.classification[ix].value > result.classification[best].value) best = ix;
    }
    ei_printf("RESULT: %s\r\n\r\n", result.classification[best].label);
}


