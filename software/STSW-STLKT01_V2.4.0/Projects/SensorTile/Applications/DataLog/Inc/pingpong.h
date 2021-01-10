#pragma once

#include <stdint.h>

void LoRa_init();

int LoRa_dataexchange(uint8_t * txData,
                      uint16_t txDataLen,
                      uint8_t * rxData,
                      uint16_t rxDataBufSize);
