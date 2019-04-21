/**
 * @file    TaskDice_Test.c
 * @brief   Application entry point.
 */

/**************************
 * BOARD RELATED INCLUDES
 **************************/
#include "MK64F12.h"
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "fsl_i2c_cmsis.h"
#include "Driver_I2C.h"
#include "fsl_port.h"

/**************************
 * OTHER INCLUDES
 **************************/
#include <stdio.h>

/****************
 * DEFINITIONS
 ****************/
#define I2C_MASTER Driver_I2C0

/* FXOS8700 and MMA8451 have the same who_am_i register address */
#define FXOS8700_WHOAMI 0xC7U
#define MMA8451_WHOAMI 0x1AU
#define ACCEL_WHOAMI_REG 0x0DU

/*  FXOS8700 and MMA8451 device address */
const uint8_t g_accel_address[] = {0x1CU, 0x1DU, 0x1EU, 0x1FU};

#define ACCEL_STATUS 0x00U
#define ACCEL_XYZ_DATA_CFG 0x0EU
#define ACCEL_CTRL_REG1 0x2AU

#define INPUT kGPIO_DigitalInput
#define OUTPUT kGPIO_DigitalOutput
#define HIGH 1U
#define LOW 0U

/***************
 * DECLARATIONS
 ***************/
uint32_t I2C0_GetFreq(void);
void I2C_MasterSignalEvent_t(uint32_t event);
static bool I2C_ReadAccelWhoAmI(void);

static bool I2C_WriteAccelReg(uint8_t device_addr, uint8_t reg_addr, uint8_t value);
static bool I2C_ReadAccelRegs(uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize);


/**************
 * MAIN
 **************/

uint8_t g_accel_addr_found = 0x00;

/*
 * @brief   Application entry point.
 */
int main(void) {
	bool isThereAccel = false;
	int16_t x, y, z;


  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();

  	/* Init FSL debug console. */
    BOARD_InitDebugConsole();

    PRINTF("Hello World\n");

    /* I2C / Accelerometer */
    I2C_MASTER.Initialize(I2C_MasterSignalEvent_t);	// Initializes I2C0
    I2C_MASTER.PowerControl(ARM_POWER_FULL);		// Turns On Power
    I2C_MASTER.Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST_PLUS); /* masterConfig.baudRate_Bps = 100000U;
																		 * masterConfig.enableStopHold = false;
																		 * masterConfig.glitchFilterWidth = 0U;
																		 * masterConfig.enableMaster = true; */

    isThereAccel = I2C_ReadAccelWhoAmI();

    /*  read the accel xyz value if there is accel device on board */
	if (true == isThereAccel)
	{
		uint8_t databyte = 0;
		uint8_t write_reg = 0;
		uint8_t readBuff[7];

		uint8_t status0_value = 0;

		/*  please refer to the "example FXOS8700CQ Driver Code" in FXOS8700 datasheet. */
		/*  write 0000 0000 = 0x00 to accelerometer control register 1 */
		/*  standby */
		/*  [7-1] = 0000 000 */
		/*  [0]: active=0 */
		write_reg = ACCEL_CTRL_REG1;
		databyte = 0;
		I2C_WriteAccelReg(g_accel_addr_found, write_reg, databyte);

		/*  write 0000 0001= 0x01 to XYZ_DATA_CFG register */
		/*  [7]: reserved */
		/*  [6]: reserved */
		/*  [5]: reserved */
		/*  [4]: hpf_out=0 */
		/*  [3]: reserved */
		/*  [2]: reserved */
		/*  [1-0]: fs=01 for accelerometer range of +/-4g range with 0.488mg/LSB */
		/*  databyte = 0x01; */
		write_reg = ACCEL_XYZ_DATA_CFG;
		databyte = 0x01;
		I2C_WriteAccelReg(g_accel_addr_found, write_reg, databyte);

		/*  write 0000 1101 = 0x0D to accelerometer control register 1 */
		/*  [7-6]: aslp_rate=00 */
		/*  [5-3]: dr=001 for 200Hz data rate (when in hybrid mode) */
		/*  [2]: lnoise=1 for low noise mode */
		/*  [1]: f_read=0 for normal 16 bit reads */
		/*  [0]: active=1 to take the part out of standby and enable sampling */
		/*   databyte = 0x0D; */
		write_reg = ACCEL_CTRL_REG1;
		databyte = 0x0d;
		I2C_WriteAccelReg(g_accel_addr_found, write_reg, databyte);
		PRINTF("The accel values:\r\n");

		while (1){
			status0_value = 0;

			/*  wait for new data are ready. */
			while (status0_value != 0xff)
			{
				I2C_ReadAccelRegs(g_accel_addr_found, ACCEL_STATUS, &status0_value, 1);
			}

			/*  Multiple-byte Read from STATUS (0x00) register */
			I2C_ReadAccelRegs(g_accel_addr_found, ACCEL_STATUS, readBuff, 7);

			status0_value = readBuff[0];
			x = ((int16_t)(((readBuff[1] * 256U) | readBuff[2]))) / 4U;
			y = ((int16_t)(((readBuff[3] * 256U) | readBuff[4]))) / 4U;
			z = ((int16_t)(((readBuff[5] * 256U) | readBuff[6]))) / 4U;

			PRINTF("status_reg = 0x%x , x = %5d , y = %5d , z = %5d \r\n", status0_value, x, y, z);

		}
	}

    /* Enter an infinite loop, just incrementing a counter. */
    volatile static int i = 0 ;
    while(1) {
        i++ ;
    }
    return 0 ;
}



/**********************
 * METHODS
 * ********************/

volatile bool completionFlag = false;
volatile bool nakFlag = false;

/* I2C Get Frequency
 * Needed for I2C CMSIS Drivers*/
uint32_t I2C0_GetFreq(void) {
    return CLOCK_GetFreq(I2C0_CLK_SRC);
}

/* I2C Signal Event
 * Receives Events from the I2C communication */
void I2C_MasterSignalEvent_t(uint32_t event) {
    if (event == ARM_I2C_EVENT_TRANSFER_DONE){
        completionFlag = true;
    }

    if (event == ARM_I2C_EVENT_ADDRESS_NACK) {
        nakFlag = true;
    }
}

/* I2C Who Am I
 * Polls a series of I2C addresses to check if the accelerometer is present */
static bool I2C_ReadAccelWhoAmI(void) {

    uint8_t who_am_i_reg = ACCEL_WHOAMI_REG;
    uint8_t who_am_i_value = 0x00;
    uint8_t accel_addr_array_size = 0x00;
    bool find_device = false;
    uint8_t i = 0;

    // Gets the size of the array of addresses to check
    accel_addr_array_size = sizeof(g_accel_address) / sizeof(g_accel_address[0]);

    for (i = 0; i < accel_addr_array_size; i++) {
        I2C_MASTER.MasterTransmit(g_accel_address[i], &who_am_i_reg, 1, true);

        /*  wait for transfer completed. */
        while ((!nakFlag) && (!completionFlag));

        nakFlag = false;

        if (completionFlag == true) {
            completionFlag = false;
            find_device = true;
            g_accel_addr_found = g_accel_address[i];
            break;
        }
    }

    if (find_device == true) {
        I2C_MASTER.MasterReceive(g_accel_addr_found, &who_am_i_value, 1, false);

        /*  wait for transfer completed. */
        while ((!nakFlag) && (!completionFlag));

        nakFlag = false;

        if (completionFlag == true) {
            completionFlag = false;
            if (who_am_i_value == FXOS8700_WHOAMI) {
                PRINTF("Found an FXOS8700 on board , the device address is 0x%x . \r\n", g_accel_addr_found);
                return true;
            }
            else if (who_am_i_value == MMA8451_WHOAMI) {
                PRINTF("Found an MMA8451 on board , the device address is 0x%x . \r\n", g_accel_addr_found);
                return true;
            }
            else {
                PRINTF("Found a device, the WhoAmI value is 0x%x\r\n", who_am_i_value);
                PRINTF("It's not MMA8451 or FXOS8700. \r\n");
                PRINTF("The device address is 0x%x. \r\n", g_accel_addr_found);
                return false;
            }
        } else {
            PRINTF("Not a successful i2c communication \r\n");
            return false;
        }
    }
    else
    {
        PRINTF("\r\n Do not find an accelerometer device ! \r\n");
        return false;
    }
}


/* Write Accelerometer Register
 * Writes a value to an address in the accelerometer */
static bool I2C_WriteAccelReg(uint8_t device_addr, uint8_t reg_addr, uint8_t value) {
    uint8_t writedata[2] = {reg_addr, value};

    /*  direction=write : start+device_write;cmdbuff;xBuff; */
    /*  direction=recive : start+device_write;cmdbuff;repeatStart+device_read;xBuff; */

    I2C_MASTER.MasterTransmit(device_addr, writedata, 2, false);

    /*  wait for transfer completed. */
    while ((!nakFlag) && (!completionFlag))
    {
    }

    nakFlag = false;

    if (completionFlag == true)
    {
        completionFlag = false;
        return true;
    }
    else
    {
        return false;
    }
}

/* Read Accelerometer Registers
 * Reads a series of values from an address in the accelerometer */
static bool I2C_ReadAccelRegs(uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize) {
    /*  direction=write : start+device_write;cmdbuff;xBuff; */
    /*  direction=recive : start+device_write;cmdbuff;repeatStart+device_read;xBuff; */

    I2C_MASTER.MasterTransmit(device_addr, &reg_addr, 1, false);
    while ((!nakFlag) && (!completionFlag))
    {
    }
    nakFlag = false;
    completionFlag = false;
    I2C_MASTER.MasterReceive(device_addr, rxBuff, rxSize, false);

    /*  wait for transfer completed. */
    while ((!nakFlag) && (!completionFlag))
    {
    }

    nakFlag = false;

    if (completionFlag == true)
    {
        completionFlag = false;
        return true;
    }
    else
    {
        return false;
    }
}
