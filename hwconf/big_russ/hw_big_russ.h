/*
	Copyright 2018 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef HW_BIG_RUSS_H_
#define HW_BIG_RUSS_H_

#define HW_NAME					"big_russ"

// HW properties
#define HW_USE_16MHZ_XTAL
#define HW_HAS_3_SHUNTS
#define HW_HAS_PHASE_SHUNTS
#define INVERTED_SHUNT_POLARITY

// Macros
#define LED_RED_GPIO			GPIOB
#define LED_RED_PIN				1

#define LED_GREEN_ON()
#define LED_GREEN_OFF()
#define LED_RED_ON()			palSetPad(LED_RED_GPIO, LED_RED_PIN)
#define LED_RED_OFF()			palClearPad(LED_RED_GPIO, LED_RED_PIN)

/* ADC Input mapping from schematic
 *
 * ADC123_IN0: VSEN1
 * ADC123_IN1: VSEN2
 * ADC123_IN2: VSEN3
 * ADC123_IN3: ADC_TEMP (PH2_TEMP)
 * ADC12 _IN4: ADC_EXTRA3 (ISEN_BIAS)
 * ADC12 _IN5: ADC_EXTRA1 (PH3_TEMP)
 * ADC12 _IN6: ADC_EXT1 (PIN HEADER EXTRA #3 (buffered))
 * ADC12 _IN7: ADC_EXTRA4 (PIN HEADER EXTRA #1)
 * ADC12 _IN8: ADC_EXTRA5 (PIN HEADER EXTRA #2)
 * ADC12 _IN9 (ADC NC): LED_RED
 * ADC123_IN10: CURRENT1
 * ADC123_IN11: CURRENT2
 * ADC123_IN12: CURRENT3
 * ADC123_IN13: AN_IN (V+_VSEN)
 * ADC12 _IN14: TEMP_MOTOR
 * ADC12 _IN15: ADC_EXTRA2 (PH1_TEMP)
 *
 * ADC Vector (ADC1, ADC2, ADC3 into vector sequentially)
 *
 * IDX    ADC	CHAN	NAME
 *  0     (1):	IN10	CURR1
 *  1     (2):	IN11	CURR2
 *  2     (3):	IN12	CURR3
 *  3     (1):	IN0		SENS1
 *  4     (2):	IN1		SENS2
 *  5     (3):	IN2		SENS3
 *  6     (1):	IN6		ADC_EXT1 (buffered extra #3)
 *  7     (2):	IN8		ADC_EXT2 (header extra #2)
 *  8     (3):	IN3		TEMP_MOS_2
 *  9     (1):	IN14	TEMP_MOTOR
 *  10    (2):	IN7		ADC_EXT3 (header extra #1)
 *  11    (3):	IN13	AN_IN
 *  12    (1):	Vrefint
 *  13    (2):	IN4		ISEN_BIAS
 *  14    (3):	IN1		SENS2
 *  15    (1):  IN5		TEMP_MOS_3
 *  16    (2):  IN15	TEMP_MOS
 *  17    (3):  IN3		SENS3
 */

#define HW_ADC_CHANNELS			18
#define HW_ADC_INJ_CHANNELS		3
#define HW_ADC_NBR_CONV			6

// ADC Indexes
#define ADC_IND_CURR1			0
#define ADC_IND_CURR2			1
#define ADC_IND_CURR3			2
#define ADC_IND_SENS1			3
#define ADC_IND_SENS2			4
#define ADC_IND_SENS3			5
#define ADC_IND_EXT				6
#define ADC_IND_EXT2			7
#define ADC_IND_TEMP_MOS_2		8
#define ADC_IND_TEMP_MOTOR		9
#define ADC_IND_EXT3			10
#define ADC_IND_VIN_SENS		11
#define ADC_IND_VREFINT			12
#define ADC_IND_ISEN_BIAS		13 //not implemented
#define ADC_IND_TEMP_MOS_3		15
#define ADC_IND_TEMP_MOS		16

// ADC macros and settings

// Component parameters (can be overridden)
#ifndef V_REG
#define V_REG					3.3
#endif
#ifndef VIN_R1
#define VIN_R1					56000.0
#endif
#ifndef VIN_R2
#define VIN_R2					2200.0
#endif
#ifndef CURRENT_AMP_GAIN
#define CURRENT_AMP_GAIN		20.0
#endif
#ifndef CURRENT_SHUNT_RES
#define CURRENT_SHUNT_RES		(0.0005 / 2.0)
#endif

// Input voltage
#define GET_INPUT_VOLTAGE()		((V_REG / 4095.0) * (float)ADC_Value[ADC_IND_VIN_SENS] * ((VIN_R1 + VIN_R2) / VIN_R2))

// NTC Termistors
#define NTC_TEMP_INDIVIDUAL(adc) (1.0f / ((logf(NTC_RES(adc) / 10000.0f) / 3380.0f) + (1.0f / 298.15f)) - 273.15f)
#define NTC_RES(adc_val)		((4095.0 * 10000.0) / adc_val - 10000.0)
#define NTC_TEMP(adc_ind)		hw_big_rus_max_temp()

#define NTC_RES_MOTOR(adc_val)	(10000.0 / ((4095.0 / (float)adc_val) - 1.0)) // Motor temp sensor on low side
#define NTC_TEMP_MOTOR(beta)	(1.0 / ((logf(NTC_RES_MOTOR(ADC_Value[ADC_IND_TEMP_MOTOR]) / 10000.0) / beta) + (1.0 / 298.15)) - 273.15)

// Voltage on ADC channel
#define ADC_VOLTS(ch)			((float)ADC_Value[ch] / 4096.0 * V_REG)

// UART Peripheral
#define HW_UART_DEV				SD3
#define HW_UART_GPIO_AF			GPIO_AF_USART3
#define HW_UART_TX_PORT			GPIOC
#define HW_UART_TX_PIN			10
#define HW_UART_RX_PORT			GPIOC
#define HW_UART_RX_PIN			11
#define HW_UART_DE_PORT			GPIOB
#define HW_UART_DE_PIN			3
#define HW_UART_RE_PORT			GPIOB
#define HW_UART_RE_PIN			4

// ICU Peripheral for servo decoding
#define HW_USE_SERVO_TIM4
#define HW_ICU_TIMER			TIM4
#define HW_ICU_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE)
#define HW_ICU_DEV				ICUD4
#define HW_ICU_CHANNEL			ICU_CHANNEL_1
#define HW_ICU_GPIO_AF			GPIO_AF_TIM4
#define HW_ICU_GPIO				GPIOB
#define HW_ICU_PIN				6

// I2C Peripheral
#define HW_I2C_DEV				I2CD2
#define HW_I2C_GPIO_AF			GPIO_AF_I2C2
#define HW_I2C_SCL_PORT			GPIOB
#define HW_I2C_SCL_PIN			10
#define HW_I2C_SDA_PORT			GPIOB
#define HW_I2C_SDA_PIN			11

// Hall/encoder pins
#define HW_HALL_ENC_GPIO1		GPIOC
#define HW_HALL_ENC_PIN1		6
#define HW_HALL_ENC_GPIO2		GPIOC
#define HW_HALL_ENC_PIN2		7
#define HW_HALL_ENC_GPIO3		GPIOC
#define HW_HALL_ENC_PIN3		8
#define HW_ENC_TIM				TIM3
#define HW_ENC_TIM_AF			GPIO_AF_TIM3
#define HW_ENC_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE)
#define HW_ENC_EXTI_PORTSRC		EXTI_PortSourceGPIOC
#define HW_ENC_EXTI_PINSRC		EXTI_PinSource8
#define HW_ENC_EXTI_CH			EXTI9_5_IRQn
#define HW_ENC_EXTI_LINE		EXTI_Line8
#define HW_ENC_EXTI_ISR_VEC		EXTI9_5_IRQHandler
#define HW_ENC_TIM_ISR_CH		TIM3_IRQn
#define HW_ENC_TIM_ISR_VEC		TIM3_IRQHandler

//configured as "external IMU"
//#define LSM6DS3_SDA_GPIO        GPIOB
//#define LSM6DS3_SDA_PIN         11
//#define LSM6DS3_SCL_GPIO        GPIOB
//#define LSM6DS3_SCL_PIN         10
//#define LSM6DS3_INT_GPIO        GPIOD
//#define LSM6DS3_INT_PIN         2

//// SPI pins - no SPI for now, but can receive only (biss) in place of uart peripheral
//#define HW_SPI_DEV				SPID3
//#define HW_SPI_GPIO_AF			GPIO_AF_SPI3
//#define HW_SPI_PORT_NSS			GPIOC
//#define HW_SPI_PIN_NSS			13
//#define HW_SPI_PORT_SCK			GPIOC
//#define HW_SPI_PIN_SCK			10
//#define HW_SPI_PORT_MOSI			GPIOC
//#define HW_SPI_PIN_MOSI			12
//#define HW_SPI_PORT_MISO			GPIOC
//#define HW_SPI_PIN_MISO			11

// Measurement macros
#define ADC_V_L1				ADC_Value[ADC_IND_SENS1]
#define ADC_V_L2				ADC_Value[ADC_IND_SENS2]
#define ADC_V_L3				ADC_Value[ADC_IND_SENS3]
#define ADC_V_ZERO				(ADC_Value[ADC_IND_VIN_SENS] / 2)

// Macros
#define READ_HALL1()			palReadPad(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1)
#define READ_HALL2()			palReadPad(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2)
#define READ_HALL3()			palReadPad(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3)

// Override dead time. See the stm32f4 reference manual for calculating this value.
#define HW_DEAD_TIME_NSEC		100.0 //todo!  need to check with oscilloscope

// Default setting overrides
#ifndef MCCONF_L_MIN_VOLTAGE
#define MCCONF_L_MIN_VOLTAGE			12.0		// Minimum input voltage
#endif
#ifndef MCCONF_L_MAX_VOLTAGE
#define MCCONF_L_MAX_VOLTAGE			78.0	// Maximum input voltage
#endif
#ifndef MCCONF_DEFAULT_MOTOR_TYPE
#define MCCONF_DEFAULT_MOTOR_TYPE		MOTOR_TYPE_FOC
#endif
#ifndef MCCONF_FOC_F_ZV
#define MCCONF_FOC_F_ZV					30000.0
#endif
#ifndef MCCONF_L_MAX_ABS_CURRENT
#define MCCONF_L_MAX_ABS_CURRENT		300.0	// The maximum absolute current above which a fault is generated
#endif
#ifndef MCCONF_FOC_SAMPLE_V0_V7
#define MCCONF_FOC_SAMPLE_V0_V7			false	// Run control loop in both v0 and v7 (requires phase shunts)
#endif
#ifndef MCCONF_L_IN_CURRENT_MAX
#define MCCONF_L_IN_CURRENT_MAX			200.0	// Input current limit in Amperes (Upper)
#endif
#ifndef MCCONF_L_IN_CURRENT_MIN
#define MCCONF_L_IN_CURRENT_MIN			-200.0	// Input current limit in Amperes (Lower)
#endif

// Setting limits
#define HW_LIM_CURRENT			-220.0, 220.0
#define HW_LIM_CURRENT_IN		-220.0, 220.0
#define HW_LIM_CURRENT_ABS		0.0, 320.0
#define HW_LIM_VIN				11.0, 80.0
#define HW_LIM_ERPM				-200e3, 200e3
#define HW_LIM_DUTY_MIN			0.0, 0.1
#define HW_LIM_DUTY_MAX			0.0, 0.95
#define HW_LIM_TEMP_FET			-40.0, 110.0

// HW-specific functions
float hw_big_rus_max_temp(void);

#endif /* HW_BIG_RUSS_H_ */
