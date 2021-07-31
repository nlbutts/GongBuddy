#ifndef GONG_IO_TEST_H_
#define GONG_IO_TEST_H_

#include <stdint.h>
#include "main.h"

/**
 * @brief Run a test on the gong IO functions
 *
 * @param spi a pointer to the SPI interface
 */
int gong_io_test_main(SPI_HandleTypeDef * spi);

#endif /* GONG_IO_TEST_H_ */
