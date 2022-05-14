# MS5611 sensor API
## Author
Name:			Francisco Martín Villegas

Email:			f.martinvillegas00@gmail.com

Colleague:		University of Almería (Spain)

## Introduction
This package contains the BME680 temperature, humidity and pressure sensor.

The sensor driver package includes BME680.c and BME680.h files.


## Integration details
* Integrate BME680.c and BME680.h file in to the project.
* Include the BME680.h file in your code like below.
``` c
#include "BME680.h"
```

## File information
* BME680.h : This header file has the constants, function declarations, macros and datatype declarations.
* BME680.c : This source file contains the definitions of the sensor driver APIs.

## Supported sensor interfaces
* SPI 4-wire	:	SDI, SDO and CSB pins
* I2C			:	SDA and SDC pins

SPI 3-wire is currently not supported in the API.
## Usage guide
### Initializing the master
First you need to initialize the I2C master with:
```c
hal_i2c_init();
```
### Initializing the sensor
Create an int variable that indicates the mode of work of the sensor.

Then initialize the sensor using:
```c
BME680_init(mode_number);
```

#### Example for I2C
Create a BME680_calib_t struct to safe the calibration data.

Create some FLOAT variables to save final pressure, humidity and temperature, and the call the function:
```c
BME680_get_data(float *press_comp, float *hum_comp, float *tempe_comp, BME680_calib_t *NVM_coef);
```

### Sensor data units
> The temperature data is read in Celsius. 
> The pressure data is read in hPa.
> The humidity data is read in %RH.


### Templates for function
``` c
/**
 * @brief	Master initialition
 *
 */
void hal_i2c_init();

/*
 * @brief Data reading of BMP680
 *
 * @param[in]		* buffer_out	:	(uint8_t)	pointer to array buffer_out
 * @param[in]	  	BME680_command	:	(uint8_t)	command to where we want to read
 * @param[in]		size			:	(unsigned)	number of bytes that we need to read
 *
 * @param[out]		ret				:	(esp_err_t)	variable that indicates if there was a problem
 *
 */
esp_err_t BME680_read(uint8_t * buffer_out, uint8_t BME680_command, unsigned size)

/*
 * @brief Data writting in BME680
 *
 * @param[in]		BME680_command	: 	(uint8_t)		command to where we want to write
 *
 * @param[out]		ret				:  	(esp_err_t)		variable that indicates if there was a problem
 *
 */
esp_err_t BME680_write_command(uint8_t BME680_command)

/*
 * @brief Data writting in BME680
 *
 * @param[in]		BME680_register			: (uint8_t)		command to where we want to write
 * @param[in]		BME680_register_value	: (uint8_t) 	value that we want to write in the register called BME680_register
 *
 * @param[out]		ret						: (esp_err_t)	variable that indicates if there was a problem
 *
 */
esp_err_t BME680_write_register(uint8_t BME680_register, uint8_t BME680_register_value)

/**
 * @brief	Calibration coefficients reading
 *
 * @param[in]		* NVM_coef		: (BME680_calib_t)	pointer to struct that contains all of the calibration coefficients
 *
 * @param[out]		ret				: (esp_err_t) 		variable that indicates if there was a problem
 *
 */
esp_err_t BME680_calibration_data(BME680_calib_t *NVM_coef)

/**
 * @brief	Set sensor settings
 *
 * @param[in]		* NVM_coef		: (BME680_calib_t)	pointer to struct that contains all of the calibration coefficients
 *
 */
void BME680_settings(BME680_calib_t *NVM_coef)

/**
 * @brief	BME680 set sensor sleep mode
 *
 */
int BME680_set_mode()

/**
 * @brief	Reset sensor function
 *
 * @param[out]		ret		: (esp_err_t)	variable that indicates if there was a problem resetting the hardware
 *
 */
esp_err_t BME680_reset_function()

/**
 * @brief	Slave initialition
 *
 * @param[in]		* NVM_coef		: (BME680_calib_t)	pointer to BME680_calib_t struct variable called NVM_coef
 *
 */
int BME680_init(BME680_calib_t *NVM_coef)

/**
 * @brief	Pressure data compensation
 *
 * @param[in]		pres_adc		: (uint32_t)		raw pressure data read from the ADC
 * @param[in]		* NVM_coef		: (BME680_calib_t)	pointer to struct that contains all of the calibration coefficients
 *
 * @param[out]		pressure_comp	: (uint32_t)		valor of the pressure when its compensate
 *
 */
uint32_t BME680_pressure_compensation(uint32_t pres_adc , BME680_calib_t *NVM_coef)

/**
 * @brief	Temperature data compensation
 *
 * @param[in]		temp_adc		: (uint32_t)		raw temperature data read from the ADC
 * @param[in]		* NVM_coef		: (BME680_calib_t)	pointer to struct that contains all of the calibration coefficients
 *
 * @param[out]		calc_temp		: (uint32_t)		valor of the temperature when its compensate
 *
 */
int16_t BME680_temperature_compensation(uint32_t temp_adc, BME680_calib_t *NVM_coef)

/**
 * @brief	Humidity data compensation
 *
 * @param[in]		hum_adc			: (uint32_t)		raw humidity data read from the ADC
 * @param[in]		* NVM_coef		: (BME680_calib_t)	pointer to struct that contains all of the calibration coefficients
 *
 * @param[out]		calc_hum		: (uint32_t)		valor of the humidity when its compensate
 *
 */
uint32_t BME680_humidity_compensation(uint16_t hum_adc, BME680_calib_t *NVM_coef)

/**
 * @brief	Getting sensor data and then doing the compensation
 *
 * @param[in]		* press_comp		: (float)			pointer to valor of the pressure compensate
 * @param[in]		* hum_comp			: (float)			pointer to valor of the humidity compensate
 * @param[in]		* tempe_comp		: (float)			pointer to valor of the temperature compensate
 * @param[in]		* NVM_coef			: (BME680_calib_t)	pointer to struct that contains all of the calibration coefficients
 *
 */
int BME680_get_data(float *press_comp, float *hum_comp, float *tempe_comp, BME680_calib_t *NVM_coef)






```