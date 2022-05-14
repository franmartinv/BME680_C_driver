
/*
 * BME280 - temperature, pressure and humidity programming code
 */


#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "BME680.h"

void hal_i2c_init();

void app_main(void)
{
	BME680_calib_t		NVM_coef;
	float				press_comp, hum_comp, tempe_comp;

	hal_i2c_init();

	BME680_init(&NVM_coef);
	printf("\n");

	while(1) {
		BME680_get_data(&press_comp, &hum_comp, &tempe_comp, &NVM_coef);

		printf("Temperature: %f Celsius\nPressure: %f hPa\nRelative humidity: %f \n", tempe_comp, press_comp, hum_comp);

		vTaskDelay(2000/portTICK_RATE_MS);
	}
}
