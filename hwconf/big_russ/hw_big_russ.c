/*
	Copyright 2018 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "hw.h"

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "utils_math.h"
#include <math.h>
#include "mc_interface.h"

// Variables
static volatile bool i2c_running = false;

// I2C configuration
static const I2CConfig i2cfg = {
		OPMODE_I2C,
		100000,
		STD_DUTY_CYCLE
};

void hw_init_gpio(void) {
	// GPIO clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	// LEDs
	palSetPadMode(LED_RED_GPIO, LED_RED_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);

	//PWM pins: GPIOA Configuration: Channel 1 to 3 as alternate function push-pull
	palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);

	palSetPadMode(GPIOB, 13, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOB, 14, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOB, 15, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);

	//RS-485 driver/receiver enable pins
	palSetPadMode(HW_UART_DE_PORT, HW_UART_DE_PIN, PAL_MODE_OUTPUT_PUSHPULL);
	palSetPadMode(HW_UART_RE_PORT, HW_UART_RE_PIN, PAL_MODE_OUTPUT_PUSHPULL);
	palClearPad(HW_UART_RE_PORT, HW_UART_RE_PIN);
	palSetPad(HW_UART_DE_PORT, HW_UART_DE_PIN);

	// Hall sensors
	palSetPadMode(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3, PAL_MODE_INPUT_PULLUP);

	// ADC Pins
	palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG); //in0
	palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_ANALOG); //in1
	palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG); //in2
	palSetPadMode(GPIOA, 3, PAL_MODE_INPUT_ANALOG); //in3
	palSetPadMode(GPIOA, 4, PAL_MODE_INPUT_ANALOG); //in4
	palSetPadMode(GPIOA, 5, PAL_MODE_INPUT_ANALOG); //in5
	palSetPadMode(GPIOA, 6, PAL_MODE_INPUT_ANALOG); //in6
	palSetPadMode(GPIOA, 7, PAL_MODE_INPUT_ANALOG); //in7
	palSetPadMode(GPIOB, 0, PAL_MODE_INPUT_ANALOG); //in8
	//in9 not analog
	palSetPadMode(GPIOC, 0, PAL_MODE_INPUT_ANALOG); //in10
	palSetPadMode(GPIOC, 1, PAL_MODE_INPUT_ANALOG); //in11
	palSetPadMode(GPIOC, 2, PAL_MODE_INPUT_ANALOG); //in12
	palSetPadMode(GPIOC, 3, PAL_MODE_INPUT_ANALOG); //in13
	palSetPadMode(GPIOC, 4, PAL_MODE_INPUT_ANALOG); //in14
	palSetPadMode(GPIOC, 5, PAL_MODE_INPUT_ANALOG); //in15
}

void hw_setup_adc_channels(void) {
	const uint8_t ADC_CHAN_VSEN1 = ADC_Channel_0;
	const uint8_t ADC_CHAN_VSEN2 = ADC_Channel_1;
	const uint8_t ADC_CHAN_VSEN3 = ADC_Channel_2;
	const uint8_t ADC_CHAN_PH2_TEMP = ADC_Channel_3;
	const uint8_t ADC_CHAN_ISEN_BIAS = ADC_Channel_4;
	const uint8_t ADC_CHAN_PH3_TEMP = ADC_Channel_5;
	const uint8_t ADC_CHAN_EXT1 = ADC_Channel_6;
	const uint8_t ADC_CHAN_EXTRA4 = ADC_Channel_7;
	const uint8_t ADC_CHAN_EXTRA5 = ADC_Channel_8;
	//const uint8_t ADC_CHAN_NC = ADC_Channel_9;
	const uint8_t ADC_CHAN_CURR1 = ADC_Channel_10;
	const uint8_t ADC_CHAN_CURR2 = ADC_Channel_11;
	const uint8_t ADC_CHAN_CURR3 = ADC_Channel_12;
	const uint8_t ADC_CHAN_AN_IN = ADC_Channel_13;
	const uint8_t ADC_CHAN_TEMP_MOTOR = ADC_Channel_14;
	const uint8_t ADC_CHAN_PH1_TEMP = ADC_Channel_15;

	// ADC1 regular channels
	ADC_RegularChannelConfig(ADC1, ADC_CHAN_CURR1, 1, ADC_SampleTime_15Cycles); //0
	ADC_RegularChannelConfig(ADC1, ADC_CHAN_VSEN1, 2, ADC_SampleTime_15Cycles); //3
	ADC_RegularChannelConfig(ADC1, ADC_CHAN_EXT1, 3, ADC_SampleTime_15Cycles); //6
	ADC_RegularChannelConfig(ADC1, ADC_CHAN_TEMP_MOTOR, 4, ADC_SampleTime_15Cycles); //9
	ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 5, ADC_SampleTime_15Cycles); //12
	ADC_RegularChannelConfig(ADC1, ADC_CHAN_PH3_TEMP, 6, ADC_SampleTime_15Cycles); //15

	// ADC2 regular channels
	ADC_RegularChannelConfig(ADC2, ADC_CHAN_CURR2, 1, ADC_SampleTime_15Cycles); //1
	ADC_RegularChannelConfig(ADC2, ADC_CHAN_VSEN2, 2, ADC_SampleTime_15Cycles); //4
	ADC_RegularChannelConfig(ADC2, ADC_CHAN_EXTRA5, 3, ADC_SampleTime_15Cycles); //7
	ADC_RegularChannelConfig(ADC2, ADC_CHAN_EXTRA4, 4, ADC_SampleTime_15Cycles); //10
	ADC_RegularChannelConfig(ADC2, ADC_CHAN_ISEN_BIAS, 5, ADC_SampleTime_15Cycles); //13
	ADC_RegularChannelConfig(ADC2, ADC_CHAN_PH1_TEMP, 6, ADC_SampleTime_15Cycles); //16

	// ADC3 regular channels
	ADC_RegularChannelConfig(ADC3, ADC_CHAN_CURR3, 1, ADC_SampleTime_15Cycles); //2
	ADC_RegularChannelConfig(ADC3, ADC_CHAN_VSEN3, 2, ADC_SampleTime_15Cycles); //5
	ADC_RegularChannelConfig(ADC3, ADC_CHAN_PH2_TEMP, 3, ADC_SampleTime_15Cycles); //8
	ADC_RegularChannelConfig(ADC3, ADC_CHAN_AN_IN, 4, ADC_SampleTime_15Cycles); //11
	ADC_RegularChannelConfig(ADC3, ADC_CHAN_VSEN2, 5, ADC_SampleTime_15Cycles); //14
	ADC_RegularChannelConfig(ADC3, ADC_CHAN_VSEN3, 6, ADC_SampleTime_15Cycles); //17

	// Injected channels
	ADC_InjectedChannelConfig(ADC1, ADC_CHAN_CURR1, 1, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC2, ADC_CHAN_CURR2, 1, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC3, ADC_CHAN_CURR3, 1, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC1, ADC_CHAN_CURR1, 2, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC2, ADC_CHAN_CURR2, 2, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC3, ADC_CHAN_CURR3, 2, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC1, ADC_CHAN_CURR1, 3, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC2, ADC_CHAN_CURR2, 3, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC3, ADC_CHAN_CURR3, 3, ADC_SampleTime_15Cycles);
}

void hw_start_i2c(void) {
	i2cAcquireBus(&HW_I2C_DEV);

	if (!i2c_running) {
		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);
		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		i2cStart(&HW_I2C_DEV, &i2cfg);
		i2c_running = true;
	}

	i2cReleaseBus(&HW_I2C_DEV);
}

void hw_stop_i2c(void) {
	i2cAcquireBus(&HW_I2C_DEV);

	if (i2c_running) {
		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN, PAL_MODE_INPUT);
		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN, PAL_MODE_INPUT);

		i2cStop(&HW_I2C_DEV);
		i2c_running = false;

	}

	i2cReleaseBus(&HW_I2C_DEV);
}

/**
 * Try to restore the i2c bus
 */
void hw_try_restore_i2c(void) {
	if (i2c_running) {
		i2cAcquireBus(&HW_I2C_DEV);

		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		palSetPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);

		chThdSleep(1);

		for(int i = 0;i < 16;i++) {
			palClearPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
			chThdSleep(1);
			palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
			chThdSleep(1);
		}

		// Generate start then stop condition
		palClearPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);
		chThdSleep(1);
		palClearPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		chThdSleep(1);
		palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		chThdSleep(1);
		palSetPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);

		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		HW_I2C_DEV.state = I2C_STOP;
		i2cStart(&HW_I2C_DEV, &i2cfg);

		i2cReleaseBus(&HW_I2C_DEV);
	}
}

float hw_big_rus_max_temp(void) {
	float max = NTC_TEMP_INDIVIDUAL(ADC_Value[ADC_IND_TEMP_MOS]);
	float t2 = NTC_TEMP_INDIVIDUAL(ADC_Value[ADC_IND_TEMP_MOS_2]);
	float t3 = NTC_TEMP_INDIVIDUAL(ADC_Value[ADC_IND_TEMP_MOS_3]);

	if(t2 > max){
		max = t2;
	}
	if(t3> max){
		max = t3;
	}

	return max;
}
