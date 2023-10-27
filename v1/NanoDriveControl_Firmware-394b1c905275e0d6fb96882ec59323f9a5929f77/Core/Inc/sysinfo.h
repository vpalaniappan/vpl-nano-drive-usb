/*
 * SysInfo.h
 *
 *  Created on: May 15, 2023
 */

#ifndef INC_SYSINFO_H_
#define INC_SYSINFO_H_

#define UNLOCK_KEY 23982
#define IO_NUM_EXIST 0

#define BINARY_LINE_LENGTH 19	//A line of bootloader data is 18 characters long.
#define BINARY_COMMAND_LENGTH 6	//Bootloader command 6 bytes.

#define CMD_ORIGIN_USART0				( 0x00 )
#define CMD_ORIGIN_CAN1					( 0x01 )
#define CMD_ORIGIN_USART2				( 0x02 )
#define CMD_ORIGIN_USART1				( 0x03 )
#define CMD_ORIGIN_PROGRAM				( 0x04 )

#define SERIAL_BUFFER_SIZE   0x80
#define SERIAL_BUFFER_MASK ( SERIAL_BUFFER_SIZE - 0x01 )

// Conditional Statement to alert that the Buffer Size isn't a mult of 2
#if ( SERIAL_BUFFER_SIZE & SERIAL_BUFFER_MASK )
#error serial buffer size is not a power of 2
#endif

typedef struct {
  uint8_t Head;
  uint8_t Tail;
  uint8_t Data[ SERIAL_BUFFER_SIZE ];
}Que;

typedef struct __attribute__((__packed__)){
	uint16_t key;
	uint16_t ver;
	uint16_t length;
	uint16_t crc;
}ConfigVer;

typedef struct __attribute__((__packed__)){
	ConfigVer header;

	uint32_t DacRes;
	uint32_t EncoderRes;
	uint64_t Vel;
	uint64_t Accel;
	uint64_t Decel;
	uint64_t JogAccel;
	uint64_t AccelMax;
	int64_t tlp;
	int64_t tln;

}SavedParams;

//***** LED CONTROLS *****//
#define TURN_ON_RED_LED		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, 0)
#define TURN_OFF_RED_LED	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, 1)
#define TURN_ON_GREEN_LED	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, 0)
#define TURN_OFF_GREEN_LED	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, 1)
#define TURN_ON_BLUE_LED	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, 0)
#define TURN_OFF_BLUE_LED	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, 1)

//***** ADDRESS CONTROLS *****//
#define READ_ADDRESS_IN		HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)
#define SET_ADDRESS_OUT		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1)
#define CLR_ADDRESS_OUT		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0)

//***** SPI CONTROLS *****//
#define SPI2_NSS_ON			HAL_GPIO_WritePin(GPIOI, GPIO_PIN_0, 0)
#define SPI2_NSS_OFF		HAL_GPIO_WritePin(GPIOI, GPIO_PIN_0, 1)
#define SPI3_NSS1_ON		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0)
#define SPI3_NSS1_OFF		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1)
#define SPI3_NSS2_ON		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 0)
#define SPI3_NSS2_OFF		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1)
#define TW8_SS_ON			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, 0)
#define TW8_SS_OFF			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, 1)

//***** COMM CONTROLS *****//
#define RS485_TRANSMIT		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1)
#define RS485_RECEIVE		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0)

//***** DRV8962 CONTROLS *****//
#define DRV8962_MODE_OFF		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0)
#define DRV8962_MODE_ON			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1)
#define DRV8962_SLEEP_OFF		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0)
#define DRV8962_SLEEP_ON		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 1)

//***** MOTION CONTROLS *****//
#define ENCODER_IDX_IN		HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8)
#define LIMIT_POS_IN		HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14)
#define LIMIT_NEG_IN		HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15)

//***** ABSOLUTE ENCODER CONTROLS *****//
#define MB4_PORT0_SET		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 1)
#define MB4_PORT0_CLR		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 0)
#define MB4_PORT1_SET		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 1)
#define MB4_PORT1_CLR		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 0)
#define MB4_PORT2_SET		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, 1)
#define MB4_PORT2_CLR		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, 0)
#define MB4_PORT3_SET		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, 1)
#define MB4_PORT3_CLR		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, 0)
#define MB4_PORT4_SET		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, 1)
#define MB4_PORT4_CLR		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, 0)
#define MB4_PORT5_SET		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, 1)
#define MB4_PORT5_CLR		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, 0)
#define MB4_PORT6_SET		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, 1)
#define MB4_PORT6_CLR		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, 0)
#define MB4_PORT7_SET		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, 1)
#define MB4_PORT7_CLR		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, 0)

#define MB4_GETSENS_SET		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, 1)
#define MB4_GETSENS_CLR		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, 0)
#define MB4_ENABLE_SET		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, 1)
#define MB4_ENABLE_CLR		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, 0)
#define MB4_NCS_SET			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, 0)		//inverted
#define MB4_NCS_CLR			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, 1)		//inverted
#define MB4_ALE_SET			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, 1)
#define MB4_ALE_CLR			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, 0)
#define MB4_NRD_RNW_SET		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, 1)
#define MB4_NRD_RNW_CLR		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, 0)

#define RS485_DIR_PIN	GPIO_PIN_5		//GPIOB
#define LED_RED_PIN		GPIO_PIN_10		//GPIOE
#define LED_GREEN_PIN	GPIO_PIN_11		//GPIOE
#define LED_BLUE_PIN	GPIO_PIN_12		//GPIOE
#define ADDR_OUT_PIN	GPIO_PIN_12		//GPIOB
#define ADDR_IN_PIN		GPIO_PIN_13		//GPIOB
#define LSP_PIN			GPIO_PIN_14		//GPIOD
#define LSN_PIN			GPIO_PIN_15		//GPIOD

//PB5
#define SET_RS485_TRANSMIT 			GPIOB->BSRR = (uint32_t)RS485_DIR_PIN
#define SET_RS485_RECEIVE			GPIOB->BSRR = (uint32_t)RS485_DIR_PIN<<16U

#define SET_ADDR_OUT_HIGH					GPIOB->BSRR = (uint32_t)ADDR_OUT_PIN
#define SET_ADDR_OUT_LOW					GPIOB->BSRR = (uint32_t)ADDR_OUT_PIN<<16U

#define SET_LED_RED_ON 						GPIOE->BSRR = (uint32_t)LED_RED_PIN<<16U
#define SET_LED_RED_OFF 					GPIOE->BSRR = (uint32_t)LED_RED_PIN
#define SET_LED_GREEN_ON 					GPIOE->BSRR = (uint32_t)LED_GREEN_PIN<<16U
#define SET_LED_GREEN_OFF 					GPIOE->BSRR = (uint32_t)LED_GREEN_PIN
#define SET_LED_BLUE_ON 					GPIOE->BSRR = (uint32_t)LED_BLUE_PIN<<16
#define SET_LED_BLUE_OFF 					GPIOE->BSRR = (uint32_t)LED_BLUE_PIN

#define ABSOLUTE_ENCODER_SELECT 		//LPC_GPIO2->FIOSET0 |= (1<<3);
#define ABSOLUTE_ENCODER_DESELECT 		//LPC_GPIO2->FIOCLR0 |= (1<<3);

#define ENCODER_CHANNEL_DIGITAL_SELECT 	//LPC_GPIO4->FIOCLR3 |= (1<<4)				//The value here matches the EAD setting. 0 == Digital.
#define ENCODER_CHANNEL_ANALOG_SELECT 	//LPC_GPIO4->FIOSET3 |= (1<<4)

#define READ_RS485_DIR		(GPIOB->ODR & RS485_DIR_PIN)

//Parameter Default Values
#define PARDEF_FET_ONTIME 10;
#define PARDEF_AMP_DISABLE_EARLY_TIME 20
#define PARDEF_AMP_ENABLE_EARLY_TIME 100
#define DAC_RES_INIT 6000 			//6000 steps / um
#define WAVEFORM_MODE_INIT 1
#define TLP_INIT 100000000000LL		//100mm
#define TLN_INIT -100000000000LL	//-100mm
#define ACC_INIT 100000000		//100mm/s^2
#define DEC_INIT 100000000		//100mm/s^2
#define JOG_ACCEL_INIT 100000000 //
#define AMX_INIT 100000000		//0xF4240
#define VMX_INIT 20000000		//15mm/s
#define CL_CORR_VELMAX_INIT 12000000 //12mm/s
#define ENC_RES_INIT 2000
#define DBD_ENC_INIT 2			//0xA
#define DBD_TIM_INIT 0			//0x0
#define KP_WHOLE_INIT 0
#define KP_FRAC_INIT 10
#define KI_WHOLE_INIT 0
#define KI_FRAC_INIT 10
#define KD_WHOLE_INIT 0
#define KD_FRAC_INIT 10
#define KP_FRAC2_INIT 10
#define MOTOR_MODE_INIT 1
#define ENC_FILTER_MOTION_INIT 1
#define ENC_FILTER_STANDSTILL 1
#define	FPGA_FF_RESET_FREQ_INIT 15000
#define FPGA_FF_RESET_THRESHOLD_INIT 1500
#define FPGA_PHASE_OFFSET_P2_INIT 50 //50%
#define FPGA_PHASE_OFFSET_P3_INIT 25 //25%
#define FPGA_PHASE_OFFSET_P4_INIT 75 //75%
#define FPGA_WAVEFORM_AMP_INIT 100
#define VEL_INIT 2000000
#define FBK_INIT 0
#define MAX_ERROR_PID_INIT 500000
#define LIMIT_DIR_INIT 0
#define ADDR_MODE_INIT 0
#define START_PGM_INIT 0
#define MPOL_INIT 0
#define HOME_DIR_INIT 0
#define EPL_INIT 0
#define MOT_INIT 1
#define PROG_AVAIL_INIT 0
#define LOOP_PGM_INIT 0
#define LIM_POL_INIT 1
#define LIM_CONFIG_INIT 1
#define EAD_INIT 0
#define HST_INIT 10
#define LDR_INIT 0
#define HCG_INIT 1
#define HAC_INIT 100000000		//100mm/s^2
#define EDV_INIT 8000
#define DXPP_INIT 200			//0xC8
#define HDX1_INIT 200000000
#define HDX2_INIT 500000000
#define HVEL1_INIT 2000000
#define HVEL2_INIT 50000
#define SILENT_INIT 0
#define SAVE_POS_INIT 0
#define PROG_ROUTE_INIT 0
#define START_POS_INIT 0
#define IODIR0_INIT 0
#define IODIR1_INIT 0
#define IODIR2_INIT 0
#define IODIR3_INIT 0
#define	IODIR4_INIT 0
#define IODIR5_INIT 0
#define IODIR6_INIT 0
#define IODIR7_INIT 0
#define IODIR8_INIT 0
#define IODIR9_INIT 0
#define IOFUNC0_INIT 0
#define IOFUNC1_INIT 0
#define IOFUNC2_INIT 0
#define IOFUNC3_INIT 0
#define IOFUNC4_INIT 0
#define IOFUNC5_INIT 0
#define IOFUNC6_INIT 0
#define IOFUNC7_INIT 0
#define IOFUNC8_INIT 0
#define IOFUNC9_INIT 0
#define HWM_INIT 0
#define HWU_INIT 5000
#define HWS0_INIT 0
#define	HWS1_INIT 0
#define	HWS2_INIT 0
#define HWS3_INIT 0
#define HWS4_INIT 0
#define HWS5_INIT 0
#define HWS6_INIT 0
#define HWS7_INIT 0
#define HWS8_INIT 0
#define HWS9_INIT 0
#define HWR0_INIT 0
#define HWR1_INIT 1
#define HWR2_INIT 1
#define HWR3_INIT 1
#define HWR4_INIT 1
#define HWR5_INIT 1
#define HWR6_INIT 1
#define HWR7_INIT 1
#define HWR8_INIT 1
#define HWR9_INIT 1
#define PROG1_RUNTIMES 1

#define CANCOMM_INIT 0

#define IN_POS_COUNT_INIT 0
#define IN_POS_TIME_INIT 0

#endif /* INC_SYSINFO_H_ */
