/*
 * TFmini Plus distance receiver (USART1, RXNE interrupt, register level).
 *
 * Frame format (datasheet p.2), 9 bytes, streamed continuously:
 *   Byte0-1: 0x59 0x59      header
 *   Byte2-3: Dist_L, Dist_H distance, little endian
 *   Byte4-7: strength, temperature (not used here)
 *   Byte8  : checksum = low 8 bits of the sum of bytes 0..7
 *
 * UART setup (115200, 8N1, pins) is done by CubeMX in MX_USART1_UART_Init().
 * Do NOT tick "USART1 global interrupt" in CubeMX, or its generated
 * USART1_IRQHandler will clash with the one below.
 */
#include "main.h"
#include "lidar.h"

static volatile uint16_t latest_dist = 0;
static volatile uint8_t  have_frame  = 0;

static uint8_t rx_buf[9];
static uint8_t rx_n = 0;

static void parse_byte(uint8_t b)
{
    if (rx_n == 0) {                    /* hunting for first 0x59 */
        if (b == 0x59) rx_buf[rx_n++] = b;
        return;
    }
    if (rx_n == 1) {                    /* need a second 0x59 */
        if (b == 0x59) rx_buf[rx_n++] = b;
        else           rx_n = 0;
        return;
    }

    rx_buf[rx_n++] = b;
    if (rx_n < 9) return;
    rx_n = 0;                           /* frame complete */

    uint8_t sum = 0;                    /* low 8 bits of sum of first 8 bytes */
    for (int i = 0; i < 8; i++) sum += rx_buf[i];
    if (sum != rx_buf[8]) return;       /* corrupted frame, drop it */

    latest_dist = (uint16_t)(rx_buf[2] | (rx_buf[3] << 8));
    have_frame  = 1;
}

void USART1_IRQHandler(void)
{
    uint32_t sr = USART1->SR;

    if (sr & (USART_SR_RXNE | USART_SR_ORE)) {
        uint8_t b = (uint8_t)USART1->DR;    /* reading SR then DR clears the flags */

        if (sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)) {
            rx_n = 0;                       /* line error: drop partial frame, resync */
            return;
        }
        parse_byte(b);
    }
}

void lidar_init(void)
{
    (void)USART1->SR;                       /* discard anything stale */
    (void)USART1->DR;
    rx_n = 0;

    NVIC_SetPriority(USART1_IRQn, 1);       /* below the button EXTI (priority 0) */
    NVIC_EnableIRQ(USART1_IRQn);
    USART1->CR1 |= USART_CR1_RXNEIE;        /* interrupt on every received byte */
}

uint8_t lidar_get_distance(uint16_t *dist)
{
    if (!have_frame) return 0;
    *dist = latest_dist;                    /* aligned 16-bit read is atomic on Cortex-M */
    return 1;
}
