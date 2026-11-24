#ifndef LIDAR_H
#define LIDAR_H

#include <stdint.h>

/* Enable the USART1 RXNE interrupt. Call after MX_USART1_UART_Init(). */
void    lidar_init(void);

/* Copy the newest received distance into *dist.
 * Returns 1 on success, 0 if no valid frame has arrived yet. */
uint8_t lidar_get_distance(uint16_t *dist);

#endif
