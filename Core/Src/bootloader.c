
/* Include files */
#include "bootloader.h"
#include "usart.h"

UART_HandleTypeDef *huart_handle;

/**
 * @brief 		Reset the STM32 connected to the host.
 * @param dly	Delay [ms] to keep the STM32 under reset.
 * @retval 		stm32_err_t
 */
stm32_err_t stm32_reset_target(uint16_t dly)
{
	/* Reset the attached MCU */
	/* Set NRST Control pin LOW to Reset Target */
	stm32_nrst_low();

	/* Delay of "dly" ms */
	stm32_delay(dly);

	/* Set NRST Control pin HIGH to wake up Target */
	stm32_nrst_high();

	/* Returns Status "OK" */
	return STM32_ERR_OK;
}

/**
 * @brief 		Initialize the STM32 connected to the host in boot mode
 * 	   			using the selected serial interface as the interface.
 * @param huart UART handle.
 * @retval 		stm32_err_t
 */
stm32_err_t stm32_enter_boot_mode(UART_HandleTypeDef *huart)
{
	/* Register UART Handler in huart_handle variable */
	huart_handle = huart;

	/* Print debug message */
	printf("Enter in Boot mode: begin\r\n");

	/* Set the MCU in Boot Mode */

	/* Set NRST Control pin LOW to Reset Target */
	stm32_nrst_low();

	/* Set BOOT Control Pin High to initialize Target in BOOT Mode */
	stm32_boot_high();

	/* 20ms Delay */
	stm32_delay(20);

	/* Set NRST Control pin HIGH to wake up Target */
	stm32_nrst_high();

	/* 100ms Delay */
	stm32_delay(100);

	/* Send Bootloader entry code sequence 0x7F */
	/* Returns Status returned from stm32_send_init_seq Function */
	return stm32_send_init_seq(huart_handle);
}

/**
 * @brief 		 Uses the UART mode according to the specified
 *       		 parameters in the UART_InitTypeDef and send the initial commands
 *        		 to acquire the available commands for the system boot.
 * @param huart  UART handle.
 * @param stm    stm32_cmd.
 * @retval 	     stm32_err_t
 */
stm32_err_t stm32_init(UART_HandleTypeDef *huart,stm32_cmd *stm)
{

	/* Local Variables */
	uint8_t len, val, buf[257];
	int i, new_cmds;

	/* Register UART Handler in huart_handle variable */
	huart_handle = huart;

	/* Initialize the STM32 Target in boot mode */
	stm32_enter_boot_mode(huart_handle);

	/* Number of bytes in the reply */
	len = STM32_CMD_GET_LENGTH;

	/* Get Command Function to get the Target Bootloader Version and its Supported Commands   */
	/* Check returned Status: In case of Status different than "OK", returns Status "Unknown" */
	if (stm32_guess_len_cmd(huart_handle, GET_CMD_COMMAND, buf, len) != STM32_ERR_OK)
		return STM32_ERR_UNKNOWN;

	/* Registers Byte 2: Number of bytes received + 1 */
	len = buf[0] + 1;
	/* Registers Byte 3: Bootloader Version received */
	stm->bl_version = buf[1];

	/* Local variable */
	new_cmds = 0;

	/* Loop through the data received starting from buf[2] */
	for (i = 1; i < len; i++) {

		val = buf[i + 1];

		switch (val) {
		case GET_CMD_COMMAND:
			stm->get = val; break;

		case GET_VER_COMMAND:
			stm->gvr = val; break;

		case GET_ID_COMMAND:
			stm->gid = val; break;

		case RMEM_COMMAND:
			stm->rm = val; break;

		case GO_COMMAND:
			stm->go = val; break;

		case WMEM_COMMAND:
			stm->wm = val;	break;

		case EMEM_COMMAND:
		case EX_EMEM_COMMAND:
			stm->er = val; break;

		case SPECIAL_CMD_COMMAND:
			stm->sp = val; break;

		case EX_SPECIAL_CMD_COMMAND:
			stm->xsp = val; break;

		case WP_COMMAND:
			stm->wp = val; break;

		case WU_COMMAND:
			stm->uw = val;	break;

		case RP_COMMAND:
			stm->rp = val;	break;

		case RU_COMMAND:
			stm->ur = val;	break;
		case GET_CKSUM_COMMAND:
			stm->crc = val; break;

		default:
			/* In case of a different command other than the mapped ones */
			/* Check if this is the first time the if statement is being executed by incrementing new_cmds and comparing it to 0 */
			if (new_cmds++ == 0)
				/* Print debug message with the first unknown command */
				printf("GET returns unknown commands (0x%2x", val);
			else
				/* If this is not the first time the if statement is being executed, continues to print the following unknown commands */
				printf(", 0x%2x", val);
		}
	}

	/* If new_cmds variable is different than 0 */
		if (new_cmds)
			/* Print debug message to close the parenthesis of the unknown commands printed */
			printf(")\n");

		/* Receive ACK/NACK byte within a given time */
		/* Check returned Status */
		if (stm32_get_ack_timeout(huart_handle,DEFAULT_TIMEOUT) != STM32_ERR_OK) {
			return STM32_ERR_UNKNOWN;
		}

		/* If GET, GET Version or GET ID command values received from Target are not a valid command */
		if (stm->get == NOT_VALID_COMMAND
				|| stm->gvr == NOT_VALID_COMMAND
				|| stm->gid == NOT_VALID_COMMAND) {
			/* Print debug message */
			printf("Error: bootloader did not return correct information from GET command\n");
			/* Returns Status "Unknown" */
			return STM32_ERR_UNKNOWN;
		}

		/*
		 * Get Command Function to get the Target Bootloader Version and its Supported Commands.
		 *
		 * The Get ID command is used to get the version of the chip ID (identification).
		 * When the bootloader receives the command, it transmits the product ID to the host.
		 *
		 * The STM32 device sends the bytes for the GET ID command as follows:
		 * Byte 1	: ACK
		 * Byte 2	: N = the number of bytes – 1 (N = 1 for STM32), except for current byte and ACKs.
		 * Bytes 3-4: PID - byte 3 = 0x04, byte 4 = 0xXX
		 * Byte 5	: ACK
		 *
		 * This function will send the GET ID command and check the returned Status
		 * */
		if (stm32_guess_len_cmd(huart_handle, stm->gid, buf, 1) != STM32_ERR_OK) {
			return STM32_ERR_UNKNOWN;
		}

		/* Register the number of bytes received (+1) */
		len = buf[0] + 1;


		/* len MUST be 2 for STM32 devices */
		if (len < 2) {
			/* Print debug message if len < 2 */
			printf("Only %d bytes sent in the PID, unknown/unsupported device\n", len);
			/* Returns Status "Unknown" */
			return STM32_ERR_UNKNOWN;
		}

		/* As Product ID comes in 2 bytes, this line combines buf[1] and buf[2] values, both 8 bits, into a single 16 bits value */
		/* The PID value is registered */
		stm->pid = (buf[1] << 8) | buf[2];

		/* If len > 2, extra bytes are identified */
		if (len > 2) {
			/* Print debug message */
			printf("This bootloader returns %d extra bytes in PID:", len);
			/* For loop to print the values received */
			for (i = 2; i <= len ; i++)
				printf(" %02x", buf[i]);
			printf("\n");
		}
		/* Receives ACK/NACK byte within a given time */
		/* Check returned Status */
		if (stm32_get_ack_timeout(huart_handle,DEFAULT_TIMEOUT) != STM32_ERR_OK) {
			return STM32_ERR_UNKNOWN;
		}

		/*
		 * The Get Version & Read Protection Status command is used to get the bootloader version
		 * and the read protection status. After receiving the command the bootloader transmits the
		 * version, the read protection and number of times it has been enabled and disabled to the
		 * host
		 *
		 * This function sends GET Version Command and checks the returned Status
		 * */
		if (stm32_send_command_timeout(huart_handle, GET_VER_COMMAND, DEFAULT_TIMEOUT) != STM32_ERR_OK) {
			printf("Error to get version and read protection status\r\n");
			return STM32_ERR_UNKNOWN;
		}

		/*
		 * The STM32 sends the bytes as follows:
		 * Byte 1: ACK - already received from the previous command sent
		 * Byte 2: Bootloader version (0 < version <= 255), example: 0x10 = version 1.0
		 * Byte 3: Option byte 1: 0x00 to keep the compatibility with generic bootloader protocol
		 * Byte 4: Option byte 2: 0x00 to keep the compatibility with generic bootloader protocol
		 * Byte 5: ACK
		 * */
		len = 3;
		/* Reads the bytes received over UART and checks the returned Status */
		if( stm32_port_serial_read(huart_handle, buf, len, DEFAULT_TIMEOUT) != HAL_OK)
			return STM32_ERR_UNKNOWN;

		/* Registers Bootloader version */
		stm->version = buf[0];

		/* Registers Read Protection Status (Option Byte 1 and 2): for legacy compatibility, both bytes are 0 */
		stm->option1 = buf[1];
		stm->option2 = buf[2];

		/* Receives ACK/NACK byte within a given time and checks the returned Status */
		if (stm32_get_ack_timeout(huart_handle,DEFAULT_TIMEOUT) != STM32_ERR_OK) {
			return STM32_ERR_UNKNOWN;
		}

		/* Verifies if Product ID returned from Target matches the specified PID */
		if (stm->pid != TARGET_PID);

		/* Returns Status OK */
		return STM32_ERR_OK;
	}

/**
 * @brief		 Sends Commands, Reads ACK bytes and Registers the Number of Bytes to be
 * 				 received from Target Board.
 * @param huart  UART handle and stm32_cmd, pointer and length.
 * @param cmd	 Get CMD command.
 * @param data	 Buffer to allocate the supported command codes.
 * @param len	 Number of bytes in the reply.
 * @retval 	 	 stm32_err_t
 */
stm32_err_t stm32_guess_len_cmd(UART_HandleTypeDef *huart, uint8_t cmd,
		uint8_t *data, unsigned int len)
{
	huart_handle = huart;
	HAL_StatusTypeDef  p_err;

	/* Send cmd command with a given timeout */
	/* Check returned status */
	if (stm32_send_command_timeout(huart_handle, cmd, DEFAULT_TIMEOUT) != STM32_ERR_OK)
		return STM32_ERR_UNKNOWN;

	/* Reads the Number of Bytes to be received and its ACK/NACK byte */
	p_err = stm32_port_serial_read(huart_handle, data, 1, DEFAULT_TIMEOUT);

	/* In case of NACK, returns Status "Unknown" */
	if (p_err != HAL_OK)
		return STM32_ERR_UNKNOWN;

	/* Registers the Number of Bytes to be received */
	len = data[0];

	/* Receives Commands Values to follow */
	/* Receives ACK/NACK byte */
	p_err = stm32_port_serial_read(huart_handle, data + 1, len + 1, DEFAULT_TIMEOUT);

	/* In case of NACK, returns Status "Unknown" */
	if (p_err != HAL_OK)
		return STM32_ERR_UNKNOWN;

	/* In case of ACK, returns Status "OK" */
	return STM32_ERR_OK;
}

/**
 * @brief 			Receive ACK/NACK byte within a given time.
 * @param huart		UART Handle.
 * @param timeout	Timeout value.
 * @retval			stm32_err_t
 */
stm32_err_t stm32_get_ack_timeout(UART_HandleTypeDef *huart, uint16_t timeout)
{
	/* Local Variables */
	uint8_t byte;
	/* Register UART Handler in huart_handle variable */
	huart_handle = huart;

	/* Loop until receives an ACK or encounter an error */
	do {
		/* Receives ACK/NACK Byte with a defined Timeout */
		if( stm32_port_serial_read(huart_handle, (uint8_t*)&byte, 1, timeout) == HAL_TIMEOUT)
		{
			/* Print a debug message in case of fail to read ACK byte */
			printf( "Failed to read ACK byte\r\n");
			return STM32_ERR_UNKNOWN;
		}

		if (byte == BL_ACK)
			return STM32_ERR_OK;

		else if (byte == BL_NAK)
			return STM32_ERR_NACK;

		/* In case of different byte received, returns Status "Unknown" */
		else {
			/* Prints a debug message of the received byte */
			printf("Got byte 0x%02x instead of ACK\r\n", byte);
			return STM32_ERR_UNKNOWN;
		}
	} while (1);
}

/**
 * @brief 			Send a command with a given timeout.
 * @param huart 	UART handle.
 * @param cmd 	 	Command to be sent.
 * @param timeout	Timeout value.
 * @retval 			stm32_err_t
 */
stm32_err_t stm32_send_command_timeout(UART_HandleTypeDef *huart,
		const uint8_t cmd,
		uint16_t timeout)
{
	/* Local Variables */
	huart_handle = huart;
	stm32_err_t s_err;
	uint8_t cmd_frame[2];

	/* Sends cmd command */
	cmd_frame[0] = cmd;

	/* Sends bitwise XOR of cmd and 0xFF - Simple error-checking mechanism */
	cmd_frame[1] = cmd ^ 0xFF;

	/* Flush Anything Previously in the UART */
	stm32_port_serial_flush(huart_handle);

	/* Sends cmd frame over UART */
	stm32_port_serial_write(huart_handle, cmd_frame, 2U, 1000U);

	/* Receives ACK/NACK byte within a given time and registers it */
	s_err = stm32_get_ack_timeout(huart_handle, timeout);

	/* In case of ACK, returns Status OK */
	if (s_err == STM32_ERR_OK)
		return STM32_ERR_OK;

	/* Prints debug messages in case of Status different than OK */
	if (s_err == STM32_ERR_NACK)
	{
		printf("Got NACK from device on command 0x%02x\r\n", cmd);
	}
	else
	{
		printf("Unexpected reply from device on command 0x%02x\r\n", cmd);
	}
	return STM32_ERR_UNKNOWN;
}

/**
 * @brief 			Reads up to 256Bytes of memory, it can be from FLASH or RAM.
 * 					CAUTION: there is not boundary check, so make sure the address is valid.
 * @param huart 	UART handle.
 * @param address	Memory start address to be read.
 * @param buffer 	Buffer to register read data.
 * @param length	Memory length to be read.
 * @retval 			stm32_err_t.
 */
stm32_err_t stm32_read_memory(UART_HandleTypeDef *huart, uint32_t address,
		uint8_t data[], unsigned int len)
{
	/* Local Variables */
	huart_handle = huart;
	uint8_t buf[5];

	/* If len equals to 0, returns Status OK and ends function execution */
	if (!len)
		return STM32_ERR_OK;

	/* If len > 256, prints debug message with the length limit and returns Status Unknown */
	if (len > 256) {
		printf("Error: READ length limit at 256 bytes\r\n");
		return STM32_ERR_UNKNOWN;
	}

	if (stm32_send_command_timeout(huart_handle, RMEM_COMMAND, DEFAULT_TIMEOUT) != STM32_ERR_OK)
		return STM32_ERR_UNKNOWN;

	/* Sends the address to be read */

	/* Byte 3: MSB */
	buf[0] = address >> 24;
	/* Byte 4 */
	buf[1] = (address >> 16) & 0xFF;
	/* Byte 5 */
	buf[2] = (address >> 8) & 0xFF;
	/* Byte 6: LSB */
	buf[3] = address & 0xFF;

	/* Performs checksum by performing a bitwise XOR between bytes 3 - 6*/
	buf[4] = buf[0] ^ buf[1] ^ buf[2] ^ buf[3];

	/* Sends the above address over UART and checks the returned Status */
	if (stm32_port_serial_write(huart_handle, buf, 5, DEFAULT_TIMEOUT) != HAL_OK)
		return STM32_ERR_UNKNOWN;

	/* Receives ACK/NACK byte within a given time and checks the returned Status */
	if (stm32_get_ack_timeout(huart_handle, DEFAULT_TIMEOUT) != STM32_ERR_OK)
		return STM32_ERR_UNKNOWN;

	/* Sends the number of bytes to be read - 1 and checks the returned Status */
	if (stm32_send_command_timeout(huart_handle, len - 1,DEFAULT_TIMEOUT) != STM32_ERR_OK)
		return STM32_ERR_UNKNOWN;

	/* Receives the data over UART and checks the returned Status */
	if( stm32_port_serial_read(huart_handle, data, len, DEFAULT_TIMEOUT) != HAL_OK)
		return STM32_ERR_UNKNOWN;

	return STM32_ERR_OK;
}

/**
 * @brief 			Writes up to 256Bytes of memory, it can be from FLASH or RAM.
 * 					CAUTION: there is not boundary check, bee sure of the address
 * @param huart 	UART handle.
 * @param address	Memory start address to be read.
 * @param buffer 	Buffer to register read data.
 * @param length	Memory length to be read.
 * @retval 			stm32_err_t.
 */
stm32_err_t stm32_write_memory(UART_HandleTypeDef *huart, uint32_t address,
		const uint8_t data[], unsigned int len)
{
	/* Local Variables */
	huart_handle = huart;
	uint8_t cs, buf[256 + 2];
	unsigned int i, aligned_len;
	stm32_err_t s_err;

	/* If len equals to 0, returns Status OK and ends function execution */
	if (!len)
		return STM32_ERR_OK;

	/* If len > 256, prints debug message with the length limit and returns Status Unknown */
	if (len > 256) {
		printf("Error: WRITE length limit at 256 bytes\r\n");
		return STM32_ERR_UNKNOWN;
	}

	/* All write operations must be Word-aligned (32bit aligned) */
	/* Checks if address or len are correctly aligned */
	if (address & 0x3 || len & 0x3) {
		printf("Error: WRITE address and length must be 4 byte aligned\r\n");
		return STM32_ERR_UNKNOWN;
	}

	if (stm32_send_command_timeout(huart_handle, WMEM_COMMAND,DEFAULT_TIMEOUT) != STM32_ERR_OK)
		return STM32_ERR_UNKNOWN;

	/* Sends the address to be written */
	/* The & 0xFF operation masks the address value with 0xFF - Taking only the least significant 8 bits */

	/* Byte 3: MSB */
	buf[0] = address >> 24;
	/* Byte 4 */
	buf[1] = (address >> 16) & 0xFF;
	/* Byte 5 */
	buf[2] = (address >> 8) & 0xFF;
	/* Byte 6: LSB */
	buf[3] = address & 0xFF;

	/* Performs checksum by performing a bitwise XOR between bytes 3 - 6*/
	buf[4] = buf[0] ^ buf[1] ^ buf[2] ^ buf[3];

	/* Sends the above address over UART and checks the returned Status */
	if (stm32_port_serial_write(huart_handle, buf, 5, DEFAULT_TIMEOUT) != HAL_OK)
		return STM32_ERR_UNKNOWN;

	/* Receives ACK/NACK byte within a given time and checks the returned Status */
	if (stm32_get_ack_timeout(huart_handle,DEFAULT_TIMEOUT) != STM32_ERR_OK)
		return STM32_ERR_UNKNOWN;

	/* Calculates the aligned length of the data to write */
	aligned_len = (len + 3) & ~3;

	/* Calculates checksum value */
	cs = aligned_len - 1;

	/* Sets the first byte of the buf array to the checksum value calculated previously */
	buf[0] = aligned_len - 1;

	/* Loop that iterates over the lenght len  */
	for (i = 0; i < len; i++) {

		/* Calculates the checksum for the current data by performing a bitwise XOR operation between the current byte of data and the current value of the checksum */
		cs ^= data[i];
		buf[i + 1] = data[i];
	}

	/* Padding data - It iterates over the remaining bytes of data */
	for (i = len; i < aligned_len; i++) {
		cs ^= 0xFF;
		buf[i + 1] = 0xFF;
	}

	/* Registers checksum value */
	buf[aligned_len + 1] = cs;

	/* Sends the number of aligned bytes to be written and checks the returned Status */
	if (stm32_port_serial_write(huart_handle, buf, aligned_len + 2,DEFAULT_TIMEOUT) != HAL_OK)
		return STM32_ERR_UNKNOWN;

	/* Receives ACK/NACK byte within a given timeout and checks the returned Status */
	s_err = stm32_get_ack_timeout(huart_handle, DEFAULT_TIMEOUT);

	/* In case of Status different than OK, it returns Status "Unknown" */
	if (s_err != STM32_ERR_OK) {
		return STM32_ERR_UNKNOWN;
	}

	/* Returns Status OK */
	return STM32_ERR_OK;
}

/**
 * @brief 			Send the bootloader entry code sequence 0x7F using the chosen USART
 * @param huart 	UART handle
 * @retval stm32_	err_t
 */
stm32_err_t stm32_send_init_seq(UART_HandleTypeDef *huart)
{
	/* Local Variables */
	huart_handle = huart;
	stm32_err_t s_err;
	uint8_t cmd = BL_UART_SOF;

	/* Flush Anything Previously in the UART */
	stm32_port_serial_flush(huart_handle);

	/* Send BL_UART_SOF Command */
	/* Check Returned Status */
	if (stm32_port_serial_write(huart_handle, &cmd, 1,DEFAULT_TIMEOUT) != HAL_OK)
	{
		/* In case of Status different than "OK", print a debug message */
		printf("Failed to send init the device\r\n");
		return STM32_ERR_UNKNOWN;
	}

	/* Receives ACK/NACK byte within a given timeout and checks the returned Status */
	s_err = stm32_get_ack_timeout(huart_handle, DEFAULT_TIMEOUT);

	/* Check Returned Status */
	if (s_err != STM32_ERR_OK) {
		return STM32_ERR_UNKNOWN;
	}

	printf("Success in init the device\r\n");

	/* Returns Status "OK" */
	return STM32_ERR_OK;
}

/**
 * @brief 			The Erase Memory command allows the host to erase Flash memory pages (regular mode).
 * 					The Extended Erase Memory command allows the host to erase Flash memory pages using two bytes addressing mode.
 * @param huart 	UART handle.
 * @param cmd		Can be either EMEM_COMMAND or EX_EMEM_COMMAND.
 * @param spage		Page size.
 * @param pages		Number of pages.
 * @retval 			stm32_err_t
 */
stm32_err_t stm32_erase_memory(UART_HandleTypeDef *huart, uint8_t cmd, uint8_t spage, uint8_t pages)
{
	/* Local Variables */
	huart_handle = huart;
	stm32_err_t s_err;
	HAL_StatusTypeDef  p_err;

	/* If pages equals to 0, returns Status OK and ends function execution */
	if (!pages)
		return STM32_ERR_OK;

	/* This function sends the Erase Memory Command with a given timeout and checks Status returned */
	if (stm32_send_command_timeout(huart_handle, EX_EMEM_COMMAND,DEFAULT_TIMEOUT) != STM32_ERR_OK) {
		printf( "Can't initiate chip erase!\r\n");
		return STM32_ERR_UNKNOWN;
	}

	/* Executes this statement in case of cmd = 0x44 */
	if (cmd == EX_EMEM_COMMAND) {
		if (pages == 0xFF) {
			uint8_t buf[3];

			/* 0xFFFF - To use Global Mass Erase */
			buf[0] = 0xFF;
			buf[1] = 0xFF;
			/* Checksum byte */
			buf[2] = 0x00;

			/* Sends the above bytes over UART and checks the returned Status */
			if (stm32_port_serial_write(huart_handle, buf, 3, DEFAULT_TIMEOUT) != HAL_OK) {
				printf( "Mass erase error.\r\n");
				return STM32_ERR_UNKNOWN;
			}

			/* Receives ACK/NACK byte within a given timeout and checks the returned Status */
			s_err = stm32_get_ack_timeout(huart_handle, DEFAULT_TIMEOUT);

			/* Prints debug message in case of Status different than OK received and returns Status Unknown */
			if (s_err != STM32_ERR_OK) {
				printf( "Mass erase failed. Try specifying the number of pages to be erased.\r\n");
				return STM32_ERR_UNKNOWN;
			}

			return STM32_ERR_OK;
		}

		/* Local Variables */
		uint16_t pg_num;
		uint8_t pg_byte;
		uint8_t cs = 0;
		uint8_t *buf;
		int i = 0;

		buf = malloc(2 + 2 * pages + 1);
		if (!buf)
			return STM32_ERR_UNKNOWN;

		/* Number of pages to be erased - 1, on two bytes, with MSB first */
		pg_byte = (pages - 1) >> 8;
		buf[i++] = pg_byte;
		cs ^= pg_byte;
		pg_byte = (pages - 1) & 0xFF;
		buf[i++] = pg_byte;
		cs ^= pg_byte;

		for (pg_num = spage; pg_num < spage + pages; pg_num++) {
			pg_byte = pg_num >> 8;
			cs ^= pg_byte;
			buf[i++] = pg_byte;
			pg_byte = pg_num & 0xFF;
			cs ^= pg_byte;
			buf[i++] = pg_byte;
		}
		buf[i++] = cs;

		/* Sends the buffer buf over UART and checks the returned Status */
		p_err = stm32_port_serial_write(huart_handle, buf, i, DEFAULT_TIMEOUT);
		free(buf);

		/* Prints debug message in case of Status different than OK received and returns Status Unknown */
		if (p_err != HAL_OK) {
			printf( "Page-by-page erase error.\r\n");
			return STM32_ERR_UNKNOWN;
		}

		/* Receives ACK/NACK byte within a given timeout and checks the returned Status */
		s_err = stm32_get_ack_timeout(huart_handle, DEFAULT_TIMEOUT);

		/* Prints debug message in case of Status different than OK received and returns Status Unknown */
		if (s_err != STM32_ERR_OK) {
			printf( "Page-by-page erase failed. Check the maximum pages your device supports.\r\n");
			return STM32_ERR_UNKNOWN;
		}

		return STM32_ERR_OK;
	}

	/* Regular Erase (0x43 command) */

	if (pages == 0xFF) {
		s_err = stm32_send_command_timeout(huart_handle, 0xFF, DEFAULT_TIMEOUT);
		if (s_err != STM32_ERR_OK) {
			return STM32_ERR_UNKNOWN;
		}
		return STM32_ERR_OK;
	} else {
		uint8_t cs = 0;
		uint8_t pg_num;
		uint8_t *buf;
		int i = 0;

		buf = malloc(1 + pages + 1);
		if (!buf)
			return STM32_ERR_UNKNOWN;

		buf[i++] = pages - 1;
		cs ^= (pages-1);
		for (pg_num = spage; pg_num < (pages + spage); pg_num++) {
			buf[i++] = pg_num;
			cs ^= pg_num;
		}
		buf[i++] = cs;

		/* Sends the buffer buf over UART and checks the returned Status */
		p_err = stm32_port_serial_write(huart_handle, buf, i, DEFAULT_TIMEOUT);

		free(buf);

		/* Prints debug message in case of Status different than OK received and returns Status Unknown */
		if (p_err != HAL_OK) {
			printf( "Erase failed.\r\n");
			return STM32_ERR_UNKNOWN;
		}

		/* Receives ACK/NACK byte within a given timeout and checks the returned Status */
		s_err = stm32_get_ack_timeout(huart_handle, DEFAULT_TIMEOUT);

		/* Prints debug message in case of Status different than OK received and returns Status Unknown */
		if (s_err != STM32_ERR_OK) {
			return STM32_ERR_UNKNOWN;
		}

		return STM32_ERR_OK;
	}
}

/*
* @brief 	The Go command is used to execute the downloaded code or any other code by branching to an address specified by the application.
 * @param huart 	UART handle.
 * @param address	Address to start.
 * 			CAUTION: there is not check if the address is correct.
 * @retval 		stm32_err_t
 */
stm32_err_t stm32_go(UART_HandleTypeDef *huart, uint32_t address)
{
	/* Local Variable */
	huart_handle = huart;
	uint8_t buf[5];

	if (stm32_send_command_timeout(huart_handle, GO_COMMAND, DEFAULT_TIMEOUT) != STM32_ERR_OK)
		return STM32_ERR_UNKNOWN;
	/* Byte 3: MSB */
	buf[0] = address >> 24;
	/* Byte 4 */
	buf[1] = (address >> 16) & 0xFF;
	/* Byte 5 */
	buf[2] = (address >> 8) & 0xFF;
	/* Byte 6: LSB */
	buf[3] = address & 0xFF;

	/* Performs checksum by performing a bitwise XOR between bytes 3 - 6*/
	buf[4] = buf[0] ^ buf[1] ^ buf[2] ^ buf[3];

	/* Sends the above address over UART and checks the returned Status */
	if (stm32_port_serial_write(huart_handle, buf, 5,DEFAULT_TIMEOUT) != HAL_OK)
		return STM32_ERR_UNKNOWN;

	/* Receives ACK/NACK byte within a given timeout and checks the returned Status */
	if (stm32_get_ack_timeout(huart_handle, DEFAULT_TIMEOUT) != STM32_ERR_OK)
		return STM32_ERR_UNKNOWN;

	/* Set BOOT Control Pin LOW to exit Target from BOOT Mode */
	stm32_boot_low();

	printf("\r\nTarget Successfully Programmed!\r\n");

	/* Returns Status OK */
	return STM32_ERR_OK;
}

