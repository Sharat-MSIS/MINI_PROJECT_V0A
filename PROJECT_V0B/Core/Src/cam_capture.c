#include "cam_capture.h"
#include "cam_classify.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

extern DCMI_HandleTypeDef hdcmi;
extern DMA_HandleTypeDef  hdma_dcmi;
extern I2C_HandleTypeDef  hi2c2;
extern UART_HandleTypeDef huart2;

#define OV7670_ADDR        (0x21 << 1)
#define FRAME_BYTES        ((uint32_t)FRAME_W * FRAME_H * 2U)   /* 18,432 */
#define CAPTURE_TIMEOUT_MS 1500U

static struct { uint32_t risr, sr, cr, ndtr; } last_diag;
static uint8_t frame_buf[FRAME_BYTES] __attribute__((aligned(4)));
static uint8_t colorbar_on = 0;

/* ------------------------------------------------------------ SCCB -- */
static HAL_StatusTypeDef ov7670_read(uint8_t reg, uint8_t *val)
{
    HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(&hi2c2, OV7670_ADDR, &reg, 1, 100);
    if (st != HAL_OK) return st;
    return HAL_I2C_Master_Receive(&hi2c2, OV7670_ADDR, val, 1, 100);
}

static HAL_StatusTypeDef ov7670_write(uint8_t reg, uint8_t val)
{
    uint8_t b[2] = { reg, val };
    return HAL_I2C_Master_Transmit(&hi2c2, OV7670_ADDR, b, 2, 100);
}

typedef struct { uint8_t reg; uint8_t val; } cam_reg_t;

static const cam_reg_t cam_regs[] = {
    { 0x11, 0x01 },
    { 0x12, 0x04 }, { 0x8C, 0x00 }, { 0x04, 0x00 }, { 0x40, 0x10 }, { 0x14, 0x38 },
    { 0x4F, 0xB3 }, { 0x50, 0xB3 }, { 0x51, 0x00 }, { 0x52, 0x3D }, { 0x53, 0xA7 },
    { 0x54, 0xE4 }, { 0x3D, 0xC0 },
    /* 160x120 (UNVERIFIED values, check the image first) */
    { 0x0C, 0x04 }, { 0x3E, 0x1A }, { 0x72, 0x22 }, { 0x73, 0xF2 }, { 0x70, 0x3A },
    { 0x71, 0x35 }, { 0xA2, 0x02 }, { 0x17, 0x16 }, { 0x18, 0x04 }, { 0x32, 0x24 },
    { 0x19, 0x02 }, { 0x1A, 0x7A }, { 0x03, 0x0A },
};

static HAL_StatusTypeDef ov7670_init(void)
{
    HAL_StatusTypeDef st = ov7670_write(0x12, 0x80);
    if (st != HAL_OK) return st;
    HAL_Delay(30);
    for (uint32_t i = 0; i < sizeof(cam_regs) / sizeof(cam_regs[0]); i++) {
        st = ov7670_write(cam_regs[i].reg, cam_regs[i].val);
        if (st != HAL_OK) return st;
    }
    return HAL_OK;
}

static void ov7670_set_colorbar(uint8_t on)
{
    ov7670_write(0x70, 0x3A);
    ov7670_write(0x71, on ? (uint8_t)(0x35 | 0x80) : 0x35);
}

/* ------------------------------------------------------------ UART -- */
static void uart_print(const char *s)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)strlen(s), 200);
}

static void send_frame(void)
{
    const uint8_t hdr[9] = {
        'F', 'R', 'M', '0',
        (uint8_t)(FRAME_W & 0xFF), (uint8_t)(FRAME_W >> 8),
        (uint8_t)(FRAME_H & 0xFF), (uint8_t)(FRAME_H >> 8),
        1 /* RGB565 */
    };
    HAL_UART_Transmit(&huart2, (uint8_t *)hdr, sizeof(hdr), 200);
    HAL_UART_Transmit(&huart2, frame_buf, (uint16_t)FRAME_BYTES, 3000);
}

/* --------------------------------------------------------- capture -- */

/* Keep only a FRAME_W x FRAME_H window from the centre of the sensor frame.
 * X is counted in pixel clocks: RGB565 = 2 clocks per pixel. */
static void dcmi_crop_init(void)
{
    uint32_t x0 = ((SENSOR_W - FRAME_W) / 2) * 2U;
    uint32_t y0 =  (SENSOR_H - FRAME_H) / 2U;
    HAL_DCMI_ConfigCrop(&hdcmi, x0, y0, FRAME_W * 2U - 1U, FRAME_H - 1U);
    HAL_DCMI_EnableCrop(&hdcmi);
}

static HAL_StatusTypeDef capture_frame(void)
{
    (void)HAL_DCMI_Stop(&hdcmi);
    memset(frame_buf, 0, sizeof(frame_buf));
    DCMI->ICR = 0x1FU;

    if (HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT,
                           (uint32_t)frame_buf, FRAME_BYTES / 4U) != HAL_OK) {
        return HAL_ERROR;
    }

    uint32_t t0 = HAL_GetTick();
    while (HAL_DMA_GetState(&hdma_dcmi) != HAL_DMA_STATE_READY) {
        if ((HAL_GetTick() - t0) > CAPTURE_TIMEOUT_MS) {
            last_diag.risr = DCMI->RISR;
            last_diag.sr   = DCMI->SR;
            last_diag.cr   = DCMI->CR;
            last_diag.ndtr = hdma_dcmi.Instance->NDTR;
            (void)HAL_DCMI_Stop(&hdcmi);
            return HAL_TIMEOUT;
        }
    }
    return HAL_OK;
}

int cam_grab(void)               { return (capture_frame() == HAL_OK) ? 0 : -1; }
const uint8_t *cam_frame(void)   { return frame_buf; }

/* ----------------------------------------------------- diagnostics -- */
static void print_id(void)
{
    uint8_t pid = 0, ver = 0;
    char msg[80];
    HAL_StatusTypeDef a = ov7670_read(0x0A, &pid);
    HAL_StatusTypeDef b = ov7670_read(0x0B, &ver);
    snprintf(msg, sizeof(msg), "PID=0x%02X VER=0x%02X (st=%d,%d) expect 0x76 0x73\r\n",
             pid, ver, (int)a, (int)b);
    uart_print(msg);
}

static void print_pol(void)
{
    char msg[96];
    uint32_t cr = DCMI->CR;
    snprintf(msg, sizeof(msg), "POL: PCLK=%s  HSYNC=%s  VSYNC=%s\r\n",
             (cr & DCMI_CR_PCKPOL) ? "rising" : "falling",
             (cr & DCMI_CR_HSPOL)  ? "active-high" : "active-low",
             (cr & DCMI_CR_VSPOL)  ? "active-high" : "active-low");
    uart_print(msg);
}

static void toggle_pol(uint32_t bit)
{
    (void)HAL_DCMI_Stop(&hdcmi);
    DCMI->CR ^= bit;
    print_pol();
}

static void diag_capture(void)
{
    char msg[140];
    HAL_StatusTypeDef r = capture_frame();
    if (r == HAL_OK) {
        snprintf(msg, sizeof(msg),
                 "D: capture OK. first bytes: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                 frame_buf[0], frame_buf[1], frame_buf[2], frame_buf[3],
                 frame_buf[4], frame_buf[5], frame_buf[6], frame_buf[7]);
    } else {
        snprintf(msg, sizeof(msg),
                 "D: TIMEOUT RISR=0x%02lX SR=0x%02lX CR=0x%04lX NDTR=%lu/%lu\r\n",
                 (unsigned long)last_diag.risr, (unsigned long)last_diag.sr,
                 (unsigned long)last_diag.cr, (unsigned long)last_diag.ndtr,
                 (unsigned long)(FRAME_BYTES / 4U));
    }
    uart_print(msg);
}

static void pin_activity(void)
{
    char msg[160];
    char data[9];
    uint32_t xclk = 0, pclk = 0, href = 0, vsync = 0;
    uint32_t chgB = 0, chgC = 0;
    uint32_t pa = GPIOA->IDR, pb = GPIOB->IDR, pc = GPIOC->IDR;

    for (uint32_t i = 0; i < 1200000U; i++) {
        uint32_t a = GPIOA->IDR, b = GPIOB->IDR, c = GPIOC->IDR;
        uint32_t xa = a ^ pa, xb = b ^ pb, xc = c ^ pc;
        if (xa & (1U << 8)) xclk++;
        if (xa & (1U << 6)) pclk++;
        if (xa & (1U << 4)) href++;
        if (xb & (1U << 7)) vsync++;
        chgB |= xb;  chgC |= xc;
        pa = a; pb = b; pc = c;
    }
    data[0] = (chgC & (1U << 6))  ? '1' : '0';
    data[1] = (chgC & (1U << 7))  ? '1' : '0';
    data[2] = (chgC & (1U << 8))  ? '1' : '0';
    data[3] = (chgC & (1U << 9))  ? '1' : '0';
    data[4] = (chgC & (1U << 11)) ? '1' : '0';
    data[5] = (chgB & (1U << 6))  ? '1' : '0';
    data[6] = (chgB & (1U << 8))  ? '1' : '0';
    data[7] = (chgB & (1U << 9))  ? '1' : '0';
    data[8] = '\0';

    snprintf(msg, sizeof(msg),
             "G: edges XCLK=%lu PCLK=%lu HREF=%lu VSYNC=%lu | data D0..D7 toggled: %s\r\n",
             (unsigned long)xclk, (unsigned long)pclk,
             (unsigned long)href, (unsigned long)vsync, data);
    uart_print(msg);
}

/* ---------------------------------------------------------- public -- */
void cam_start(void)
{
    char msg[64];

    HAL_GPIO_WritePin(CAM_RESET_GPIO_Port, CAM_RESET_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(CAM_RESET_GPIO_Port, CAM_RESET_Pin, GPIO_PIN_SET);
    HAL_Delay(100);

    print_id();

    HAL_StatusTypeDef st = ov7670_init();
    snprintf(msg, sizeof(msg), "OV7670 init: %s (st=%d)\r\n",
             (st == HAL_OK) ? "OK" : "FAILED", (int)st);
    uart_print(msg);

    dcmi_crop_init();
    print_pol();
    uart_print("READY: C=frame R=classify T=colourbar I=ID D=diag G=pins V/H/P=pol\r\n");
}

void cam_poll(void)
{
    uint8_t cmd = 0;

    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE)) {
        __HAL_UART_CLEAR_OREFLAG(&huart2);
    }
    if (HAL_UART_Receive(&huart2, &cmd, 1, 100) != HAL_OK) {
        return;
    }

    if (cmd == 'C') {
        HAL_StatusTypeDef r = capture_frame();
        if (r == HAL_OK) {
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            send_frame();
        } else if (r == HAL_TIMEOUT) {
            char msg[100];
            snprintf(msg, sizeof(msg), "ERR: capture timeout RISR=0x%02lX NDTR=%lu\r\n",
                     (unsigned long)last_diag.risr, (unsigned long)last_diag.ndtr);
            uart_print(msg);
        } else {
            uart_print("ERR: DCMI start failed\r\n");
        }
    } else if (cmd == 'R') {
        cam_classify_once();
    } else if (cmd == 'T') {
        colorbar_on ^= 1U;
        ov7670_set_colorbar(colorbar_on);
        uart_print(colorbar_on ? "colour bar ON\r\n" : "colour bar OFF\r\n");
    } else if (cmd == 'I') {
        print_id();
    } else if (cmd == 'D') {
        diag_capture();
    } else if (cmd == 'G') {
        pin_activity();
    } else if (cmd == 'V') {
        toggle_pol(DCMI_CR_VSPOL);
    } else if (cmd == 'H') {
        toggle_pol(DCMI_CR_HSPOL);
    } else if (cmd == 'P') {
        toggle_pol(DCMI_CR_PCKPOL);
    }
}
