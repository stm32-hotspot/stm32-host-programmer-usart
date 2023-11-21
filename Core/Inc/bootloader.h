#ifndef INC_BOOTLOADER_H_
#define INC_BOOTLOADER_H_

/* Include files */
#include "main.h"
#include "stdio.h"
#include "stdlib.h"
#include <string.h>

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TARGET_PID					0x474
#define IMAGE_ADDRESS 				0x08009000		/* Sets the first address (OF THE STM32L4R5ZI) with the image to be downloaded into target  */
#define TARGET_FLASH_START_ADDRESS	0x08000000		/* Sets the FLASH Start Address of the Target device 										*/
#define TARGET_FLASH_END_ADDRESS	0x0801ffff		/* Sets the FLASH End Address of the Target device 	 										*/
#define TARGET_OPB_START_ADDRESS	0x40022000		/* Sets the Option Bytes Start Address of the Target device									*/
/* USER CODE END PD */

/*
 * Bootloader command set
 * */
#define GET_CMD_COMMAND        0x00U
#define GET_VER_COMMAND        0x01U
#define GET_ID_COMMAND         0x02U
#define RMEM_COMMAND           0x11U
#define GO_COMMAND             0x21U
#define WMEM_COMMAND           0x31U
#define EMEM_COMMAND           0x43U
#define EX_EMEM_COMMAND        0x44U
#define SPECIAL_CMD_COMMAND    0x50U
#define EX_SPECIAL_CMD_COMMAND 0x51U
#define WP_COMMAND             0x63U
#define WU_COMMAND             0x73U
#define RP_COMMAND             0x82U
#define RU_COMMAND             0x92U
#define GET_CKSUM_COMMAND      0xA1U
#define NOT_VALID_COMMAND      0xFFU
#define STM32_CMD_GET_LENGTH	17

#define BL_UART_SOF 		   0x7F
#define BL_ACK				   0x79U
#define BL_NAK 				   0x1FU

/*
 * Extended erase special parameters
 * */
#define ERASE_ALL    		   0xFFFFU
#define ERASE_BANK1  		   0xFFFEU
#define ERASE_BANK2  		   0xFFFDU

#define DEFAULT_TIMEOUT 100

/*
 * Macros
 * */
#define stm32_port_serial_flush HAL_UART_Abort
#define stm32_port_serial_write HAL_UART_Transmit
#define stm32_port_serial_read  HAL_UART_Receive
#define stm32_port_deinit		HAL_UART_DeInit
#define stm32_delay 			HAL_Delay

#define stm32_nrst_low()        HAL_GPIO_WritePin(NRST_CTRL_PIN_GPIO_Port, NRST_CTRL_PIN_Pin, GPIO_PIN_RESET);
#define stm32_nrst_high()       HAL_GPIO_WritePin(NRST_CTRL_PIN_GPIO_Port, NRST_CTRL_PIN_Pin, GPIO_PIN_SET);

#define stm32_boot_low()        HAL_GPIO_WritePin(BOOT_CTRL_PIN_GPIO_Port, BOOT_CTRL_PIN_Pin, GPIO_PIN_RESET);
#define stm32_boot_high()       HAL_GPIO_WritePin(BOOT_CTRL_PIN_GPIO_Port, BOOT_CTRL_PIN_Pin, GPIO_PIN_SET);

/*
 * Error Typedef and enum creation
 * */
typedef enum {
	STM32_ERR_OK      = 0x00,
	STM32_ERR_UNKNOWN = 0x01,
	STM32_ERR_NACK    = 0x02,
	STM32_ERR_NO_CMD  = 0x03	/* Command not available in bootloader */
} stm32_err_t;

/*
 * Typedef struct for Bootloader command list
 */
typedef struct stm32_bl_cm{
	uint8_t get;
	uint8_t gvr;
	uint8_t gid;
	uint8_t rm;
	uint8_t go;
	uint8_t wm;
	uint8_t er; 	/* this may be extended erase */
	uint8_t sp;
	uint8_t xsp;
	uint8_t wp;
	uint8_t uw;
	uint8_t rp;
	uint8_t ur;
	uint8_t	crc;
	uint8_t	version;
	uint8_t	option1;
	uint8_t	option2;
	uint8_t bl_version;
	uint16_t pid;
}stm32_cmd ;

/*
 * Prototypes
 */
stm32_err_t stm32_reset_target(uint16_t dly);
stm32_err_t stm32_enter_boot_mode(UART_HandleTypeDef *huart);
stm32_err_t stm32_init(UART_HandleTypeDef *huart,stm32_cmd *stm);
stm32_err_t stm32_deinit(UART_HandleTypeDef *huart);
stm32_err_t stm32_send_init_seq(UART_HandleTypeDef *huart);
stm32_err_t stm32_guess_len_cmd(UART_HandleTypeDef *huart, uint8_t cmd,uint8_t *data, unsigned int len);
stm32_err_t stm32_get_ack_timeout(UART_HandleTypeDef *huart, uint16_t timeout);
stm32_err_t stm32_send_command_timeout(UART_HandleTypeDef *huart,const uint8_t cmd,uint16_t timeout);
stm32_err_t stm32_read_memory(UART_HandleTypeDef *huart, uint32_t address,uint8_t data[], unsigned int len);
stm32_err_t stm32_write_memory(UART_HandleTypeDef *huart, uint32_t address,const uint8_t data[], unsigned int len);
stm32_err_t stm32_send_init_seq(UART_HandleTypeDef *huart);
stm32_err_t stm32_erase_memory(UART_HandleTypeDef *huart, uint8_t cmd, uint8_t spage, uint8_t pages);
stm32_err_t stm32_go(UART_HandleTypeDef *huart, uint32_t address);

#endif /* INC_BOOTLOADER_H_ */



