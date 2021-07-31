#include <stdint.h>
#include "gong_io_test.h"
#include "gong_io.h"
#include "debug_print.h"
#include "cmsis_os.h"

int gong_io_test_main(SPI_HandleTypeDef * spi)
{
    int state = 0;
    gong_io_init(spi);

    while (1)
    {
        osDelay(1000);
        gong_io_set_led(state);
        state = !state;
        int button = gong_io_read_button();
        int PG = gong_io_read_pg();
        int dio0 = gong_io_read_dio0();
        int dio1 = gong_io_read_dio1();
        DEBUG_PRINTF(0, "Button: %d  PG: %d  DIO0: %d  DIO1: %d\n", button, PG, dio0, dio1);
    }
}
