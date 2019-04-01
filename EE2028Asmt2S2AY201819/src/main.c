#include <stdio.h>
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_ssp.h"
#include "LPC17xx.h"
#include "lpc17xx_uart.h"

#include "uart2.h"
#include "temp.h"
#include "joystick.h"
#include "pca9532.h"
#include "acc.h"
#include "oled.h"
#include "rgb.h"
#include "light.h"
#include "pca9532.h"

// Timer definitions
#define SBIT_TIMER0  1
#define SBIT_TIMER1  2

#define SBIT_MR0I    0
#define SBIT_MR0R    1

#define PCLK_TIMER0  2
#define PCLK_TIMER1  4

#define SBIT_CNTEN   0

#define MiliToMicroSec(x)  (x*1000)

#define NUM_LED     16
#define PERCENT_PER_LED 6.25

#define TILT_THRESHOLD 30

volatile uint32_t msTicks;
typedef enum{
    ORBIT,
    ORBIT_TO_LANDING,
    LANDING,
    EXPLORING,
    SLEEPING
} mode_type;
volatile mode_type mode;
int8_t x[1];
int8_t y[1];
int8_t z[1];
uint8_t isModeChange;
uint32_t interval;
uint8_t flag = 0;
uint8_t numLedToTurnOff;
float batteryLevel = 100.0;
uint8_t FLAG = 0;

const uint8_t CHAR_HEIGHT = 12;
const uint32_t LIGHT_THRESHOLD = 30;
const uint8_t ORBIT_MODE_MESSAGE[][40] =
	{{'O', 'r', 'b', 'i', 't', 'i', 'n', 'g', ' ', 'M', 'o', 'd', 'e', '.'},
	 {'P', 'r', 'e', 's', 's', ' ', 'S', 'W', '3', ' ', 't', 'o'},
	 {'L', 'a', 'n', 'd'},
	 {' '},
	 {' '},
	 {' '}
	};
const uint8_t ORBIT_TO_LANDING_MODE_MESSAGE[][40] =
	{{'E', 'N', 'T', 'E', 'R', 'I', 'N', 'G'},
	 {'L', 'A', 'N', 'D', 'I', 'N', 'G', ' ', 'M', 'O', 'D', 'E'},
	 {' '},
	 {' '},
	 {' '},
	 {' '}
	};
uint8_t LANDING_MODE_MESSAGE[][40] =
	{{'L', 'A', 'N', 'D', 'I', 'N', 'G'},
	 {' 1'},
	 {' 2'},
	 {' 3'},
	 {' 4'},
	 {' 5'}};
uint8_t EXPLORING_MODE_MESSAGE[][40] =
	{{'E', 'X', 'P', 'L', 'O', 'R', 'I', 'N', 'G'},
	 {' '},
	 {' '},
	 {' '},
	 {' '},
	 {' '}};
uint8_t SLEEPING_MODE_MESSAGE[][40] =
	{{'S', 'L', 'E', 'E', 'P', 'I', 'N', 'G'},
	 {' '},
	 {' '},
	 {' '},
	 {' '},
	 {' '}};
const uint8_t invertedNums[] = {
	0x24, 0x7D, 0xE0, 0x70, 0x39, 0x32, 0x23, 0x7C, 0x20, 0x38,
};

static unsigned char TILT_TRESHOLD_TXT[] = "Poor Landing Attitude \r\n";
static unsigned char TILT_CLEAR_TXT[] = "Save to Land! \r\n";
static unsigned char ORBITING_TXT[] = "Start Orbiting, waiting for Landing \r\n";
static unsigned char LANDING_TXT[] = "LANDING Mode \r\n";
static unsigned char EXPLORING_TXT[] = "EXPLORING Mode \r\n";
static unsigned char SLEEPING_TXT[] = "System is sleeping \r\n";
static unsigned char ACC_MSG[] = "";

static uint8_t ORBITING_FLAG = 1;
static uint8_t LANDING_FLAG = 1;
static uint8_t EXPLORING_FLAG = 1;
static uint8_t SLEEPING_FLAG = 1;
static uint8_t TILT_FLAG = 1;
static uint8_t TILT_CLEAR_FLAG = 0;
static uint8_t ACC_FLAG = 0;

void SysTick_Handler(void) {
	msTicks++;
}

uint32_t getTicks(void) {
	return msTicks;
}

uint8_t is_time_passed(int time_to_pass_ms, uint32_t* prev_time) {
	uint32_t now = getTicks();
	if (now - *prev_time >= time_to_pass_ms) {
		*prev_time = now;
		return 1;
	}
	return 0;
}

static void pinsel_uart3(void) {
	PINSEL_CFG_Type PinCfg;
	PinCfg.Funcnum = 2;
	PinCfg.Pinnum = 0;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 1;
	PINSEL_ConfigPin(&PinCfg);
}

void init_uart(void){
    UART_CFG_Type uartCfg;
    uartCfg.Baud_rate = 115200;
    uartCfg.Databits = UART_DATABIT_8;
    uartCfg.Parity = UART_PARITY_NONE;
    uartCfg.Stopbits = UART_STOPBIT_1;
    //pin select for uart3;
    pinsel_uart3();
    //supply power & setup working parameters for uart3
    UART_Init(LPC_UART3, &uartCfg);
    //enable transmit for uart3
    UART_TxCmd(LPC_UART3, ENABLE);
}

static void init_ssp(void) {
	SSP_CFG_Type SSP_ConfigStruct;
	PINSEL_CFG_Type PinCfg;

	/*
	 * Initialize SPI pin connect
	 * P0.7 - SCK;
	 * P0.8 - MISO
	 * P0.9 - MOSI
	 * P2.2 - SSEL - used as GPIO
	 */
	PinCfg.Funcnum = 2;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 7;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 8;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 9;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Funcnum = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 2;
	PINSEL_ConfigPin(&PinCfg);

	SSP_ConfigStructInit(&SSP_ConfigStruct);

	// Initialize SSP peripheral with parameter given in structure above
	SSP_Init(LPC_SSP1, &SSP_ConfigStruct);

	// Enable SSP peripheral
	SSP_Cmd(LPC_SSP1, ENABLE);
}

static void init_i2c(void) {
	PINSEL_CFG_Type PinCfg;

	/* Initialize I2C2 pin connect */
	PinCfg.Funcnum = 2;
	PinCfg.Pinnum = 10;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 11;
	PINSEL_ConfigPin(&PinCfg);

	// Initialize I2C peripheral
	I2C_Init(LPC_I2C2, 100000);

	/* Enable I2C1 operation */
	I2C_Cmd(LPC_I2C2, ENABLE);
}

static void init_GPIO(void) {
	// SW4: P1.31
	PINSEL_CFG_Type PinCfg;
	PinCfg.Funcnum = 0;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 1;
	PinCfg.Pinnum = 31;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(1, 1<<31, 0);

	// SW3: P2.10
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 10;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(2, 1<<10, 0);

	// Red LED: P2.0
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(2,1,1);

	// Blue LED: P0.26
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 26;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(0,(1<<26),1);

	// Green LED: p2.1
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 1;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(2,(1<<1),1);

	// Light interrupt: P2.5
//	PinCfg.Pinnum = 5;
//	PinCfg.Portnum = 2;
//	PINSEL_ConfigPin(&PinCfg);
//	GPIO_SetDir(2,(1<<5),0);
}

unsigned int getPrescalarForUs(uint8_t timerPclkBit) {
    unsigned int pclk,prescalarForUs;
    pclk = (LPC_SC->PCLKSEL0 >> timerPclkBit) & 0x03;  // get the pclk info for required timer

    switch (pclk) { // Decode the bits to determine the pclk
    case 0x00:
        pclk = SystemCoreClock/4;
        break;
    case 0x01:
        pclk = SystemCoreClock;
        break;
    case 0x02:
        pclk = SystemCoreClock/2;
        break;
    case 0x03:
        pclk = SystemCoreClock/8;
        break;
    default:
        pclk = SystemCoreClock/4;
        break;
    }
    return pclk/1000000 - 1; // Prescalar for 1us (1000000Counts/sec)
}

void interrupt_init(void) {
    LPC_GPIOINT->IO2IntEnF |= 1<<10; // Enable P2.10 interrupt
//    LPC_GPIOINT->IO2IntEnF |= 1<<5; // Enable P2.5 interrupt

    // timer1_init
    LPC_SC->PCONP |= (1<<SBIT_TIMER0); // Power ON Timer0, 1

    LPC_TIM1->MCR  = (1<<SBIT_MR0I) | (1<<SBIT_MR0R); // Clear TC on MR0 match and Generate Interrupt
    LPC_TIM1->PR   = getPrescalarForUs(PCLK_TIMER1); // Prescalar for 1us
    LPC_TIM1->MR0  = MiliToMicroSec(500); // 500ms delay
    LPC_TIM1->TCR  = (1<<SBIT_CNTEN); // Start timer by setting the Counter Enable

    NVIC_ClearPendingIRQ(EINT3_IRQn);
    NVIC_ClearPendingIRQ(TIMER1_IRQn);

    NVIC_EnableIRQ(EINT3_IRQn); // Enable EINT3 interrupt
    NVIC_EnableIRQ(TIMER1_IRQn); // Enable Timer1 interrupt

    NVIC_SetPriorityGrouping(5);
    NVIC_SetPriority(TIMER1_IRQn, 0x04);
}

void init(void) {
	SysTick_Config(SystemCoreClock/1000);
	init_ssp();
	init_i2c();
	init_GPIO();
	init_uart();
	led7seg_init();
	oled_init();
	acc_init();
	temp_init(getTicks);
	light_enable();
	light_clearIrqStatus();
	light_setLoThreshold(LIGHT_THRESHOLD);

	interrupt_init();

	setMode(ORBIT);
}

void setMode(mode_type modeToSet) {
	mode = modeToSet;
	isModeChange = 1;
}

void countdown(void) {
	int count = 5;

	while (count >= 0) {
		led7seg_setChar(invertedNums[count], 1);
		count--;
		Timer0_Wait(1000);
	}
}

void displayOledMessage(uint8_t message[][40]) {
	uint8_t y = 0;
	int i = 0;
	for (i = 0; i < sizeof(message); i++) {
		oled_putString(0, y, message[i], OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		y += CHAR_HEIGHT;
	}
}

uint8_t toggleButton(uint32_t currTicks) {
	uint32_t nextTicks = getTicks();
	uint8_t sw3 = 1;
	while (nextTicks-currTicks <= 1000) {
		sw3 = (GPIO_ReadValue(2) >> 10) & 0x01;
		if (sw3 == 0) {
			return 1;
		}
		nextTicks = getTicks();
	}
	return 0;
}

void setAccMessage() {
	acc_read(x, y, z);
	if((x[0]/63.0) > (TILT_THRESHOLD/60.0) | (y[0]/63.0) > (TILT_THRESHOLD/60.0)) {
		if (TILT_FLAG){
			oled_clearScreen(OLED_COLOR_BLACK);
			sendUARTMessage(5);
		}
		sprintf(LANDING_MODE_MESSAGE[1], "Poor");
		sprintf(LANDING_MODE_MESSAGE[2], "Landing");
		sprintf(LANDING_MODE_MESSAGE[3], "Altitude");
		TILT_CLEAR_FLAG = 1;
	}
	else {
		sprintf(LANDING_MODE_MESSAGE[1], "Acc-X: %2.2f g\r", x[0]/63.0);
		sprintf(LANDING_MODE_MESSAGE[2], "Acc-Y: %2.2f g\r", y[0]/63.0);
		sprintf(LANDING_MODE_MESSAGE[3], "Acc-Z: %2.2f g\r\n", z[0]/63.0);
		TILT_FLAG = 1;
		if (TILT_CLEAR_FLAG)
			sendUARTMessage(7);
		if (ACC_FLAG)
			//sprintf(ACC_MSG, "%s \r%s \r%s \r\n", LANDING_MODE_MESSAGE[1], LANDING_MODE_MESSAGE[2], LANDING_MODE_MESSAGE[3]);
			sendUARTMessage(6);

	}
}

void setTempMessage() {
	uint32_t temp = temp_read();
	printf("temp: %d %f\n", temp, temp/10.0);
	sprintf(EXPLORING_MODE_MESSAGE[1], "Temp: %2.1f deg   ", temp/10.0);
//	msg = "Temp %2.1f deg \r\n", temp/10.0;
//	UART_Send(LPC_UART3, (uint8_t *)msg , strlen(msg), BLOCKING);

}

void setBatteryMessage() {
	sprintf(EXPLORING_MODE_MESSAGE[1], "Batt: %2.2f%%   ", batteryLevel);
	sprintf(SLEEPING_MODE_MESSAGE[1], "Batt: %2.2f%%   ", batteryLevel);
}

void sendUARTMessage(int typeOfMsg) {
	switch (typeOfMsg){
	case 1:
		if(ORBITING_FLAG){
			UART_Send(LPC_UART3, (uint8_t *)ORBITING_TXT , strlen(ORBITING_TXT), BLOCKING);
			ORBITING_FLAG = 0;
		}
		break;
	case 2:
		if(LANDING_FLAG){
			UART_Send(LPC_UART3, (uint8_t *)LANDING_TXT , strlen(LANDING_TXT), BLOCKING);
			LANDING_FLAG = 0;
		}
		break;
	case 3:
		if(EXPLORING_FLAG){
			UART_Send(LPC_UART3, (uint8_t *)EXPLORING_TXT , strlen(EXPLORING_TXT), BLOCKING);
			EXPLORING_FLAG = 0;
		}
		break;
	case 4:
		if(SLEEPING_FLAG){
			UART_Send(LPC_UART3, (uint8_t *)SLEEPING_TXT , strlen(SLEEPING_TXT), BLOCKING);
			SLEEPING_FLAG = 0;
		}
		break;
	case 5:
		if(TILT_FLAG) {
			UART_Send(LPC_UART3, (uint8_t *)TILT_TRESHOLD_TXT , strlen(TILT_TRESHOLD_TXT), BLOCKING);
			TILT_FLAG = 0;
		}
		break;
	case 6:
		if(ACC_FLAG) {
			UART_Send(LPC_UART3, (uint8_t *)LANDING_MODE_MESSAGE[1] , strlen(LANDING_MODE_MESSAGE[1]), BLOCKING);
			UART_Send(LPC_UART3, (uint8_t *)LANDING_MODE_MESSAGE[2] , strlen(LANDING_MODE_MESSAGE[2]), BLOCKING);
			UART_Send(LPC_UART3, (uint8_t *)LANDING_MODE_MESSAGE[3] , strlen(LANDING_MODE_MESSAGE[3]), BLOCKING);
			ACC_FLAG = 0;
		}
		break;
	case 7:
		if(TILT_CLEAR_FLAG) {
			UART_Send(LPC_UART3, (uint8_t *)TILT_CLEAR_TXT , strlen(TILT_CLEAR_TXT), BLOCKING);
			TILT_CLEAR_FLAG = 0;
		}
		break;
	}
}

void checkBattery() {
	uint8_t sw4 = (GPIO_ReadValue(1) >> 31) & 0x01;
	if (sw4 == 0) {
		batteryLevel = batteryLevel+6.25 <= 100 ? batteryLevel+6.25 : 100;
		interval = 0;
		Timer0_Wait(200);
	}
	if (interval >= 10000) {
		batteryLevel = batteryLevel-12.5 >= 0 ? batteryLevel-12.5 : 0;
		interval = 0;
	}

	if (batteryLevel <= 12.5 && mode != SLEEPING) {
		setMode(SLEEPING);
		SLEEPING_FLAG = 1;
	}
	if (batteryLevel > 12.5 && mode != EXPLORING)	{
		setMode(EXPLORING);
		EXPLORING_FLAG = 1;
	}
	numLedToTurnOff = NUM_LED-batteryLevel/PERCENT_PER_LED;
}

// EINT3 Interrupt Handler
void EINT3_IRQHandler() {
	// Determine whether GPIO Interrupt P2.10 has occurred
	if ((LPC_GPIOINT->IO2IntStatF>>10)& 0x1) {
    	static uint32_t lastTicks = 0;
    	uint32_t currTicks = getTicks();
    	if (currTicks - lastTicks <= 1000 && mode == ORBIT) setMode(ORBIT_TO_LANDING);
    	lastTicks = currTicks;
        // Clear GPIO Interrupt P2.10
        LPC_GPIOINT->IO2IntClr = 1<<10;
	}
	// Determine whether GPIO Interrupt P2.5 has occurred
//	if ((LPC_GPIOINT->IO2IntStatF>>5)& 0x1) {
//		if(light_getIrqStatus()) {
//			printf("Light Lux: \n", light_read());
//			if (mode == LANDING) setMode(EXPLORING);
//			light_clearIrqStatus(); //Clear lightsensor interrupt first before clearing gpio int
//		}
//		light_setLoThreshold(LIGHT_THRESHOLD);
//		LPC_GPIOINT->IO2IntClr = 1<<5;
//    	printf("last line\n");
//	}
}

void TIMER1_IRQHandler() {
	LPC_TIM1->IR = LPC_TIM1->IR; // Clear Timer1 interrupt
	uint32_t isRed = (GPIO_ReadValue(2) >> 0) & 0x01;
	uint32_t isBlue = (GPIO_ReadValue(0) >> 26) & 0x01;
	switch (mode) {
	    case ORBIT: // BLINK_BLUE
	    	interval = 0;
			if (isBlue) GPIO_ClearValue(0, 1<<26);
			else GPIO_SetValue(0, (1<<26));
	    	break;
	    case ORBIT_TO_LANDING: // BLINK_BLUE
	    	interval = 0;
	    	if (isBlue) GPIO_ClearValue(0, 1<<26);
	    	else GPIO_SetValue(0, (1<<26));
	    	break;
	    case LANDING: // ALTERNATE_LED
	    	interval = 0;
	    	if (isBlue) {
	    		GPIO_ClearValue(0, 1<<26);
	    		GPIO_SetValue(2, 1);
	    	}	else {
	    		GPIO_SetValue(0, (1<<26));
	    		GPIO_ClearValue(2, 1<<0);
	    	}
//			if (isBlue) GPIO_ClearValue(0, 1<<26);
//			else GPIO_SetValue(0, (1<<26));
//			if (isRed) GPIO_ClearValue(2, 1<<0);
//			else GPIO_SetValue(2, 1);
	    	break;
	    case EXPLORING: // BLUE
	    	interval += 500;
	    	GPIO_SetValue(0, (1<<26));
	    	GPIO_ClearValue(2, 1<<0);
	    	break;
	    case SLEEPING: // BLUE
	    	interval += 500;
	    	GPIO_SetValue(0, (1<<26));
	    	GPIO_ClearValue(2, 1<<0);
	    	break;
	    default:
	    	interval = 0;
	    	break;
	}
}

int main() {
	init();
	static uint32_t prev_update_time = 0;

	while(1) {
		if (isModeChange == 1) {
			oled_clearScreen(OLED_COLOR_BLACK);
			isModeChange = 0;
		}

		if (mode == ORBIT) {
			displayOledMessage(ORBIT_MODE_MESSAGE);
			led7seg_setChar(0x24, 1); // Inverted O
			sendUARTMessage(1);
		} else if (mode == ORBIT_TO_LANDING) {
			displayOledMessage(ORBIT_TO_LANDING_MODE_MESSAGE);
			countdown();
			setMode(LANDING);
		} else if (mode == LANDING) {
			sendUARTMessage(2);
			if (is_time_passed(5000, &prev_update_time)) {
				ACC_FLAG = 1;
			}
			setAccMessage();
			displayOledMessage(LANDING_MODE_MESSAGE);
			led7seg_setChar(0xA7, 1); // Inverted L
			if (light_read() < LIGHT_THRESHOLD) setMode(EXPLORING);
		} else if (mode == EXPLORING) {
//			printf("temp: %d\n", temp_read());
//			setTempMessage();
			sendUARTMessage(3);
			checkBattery();
			setBatteryMessage();
			displayOledMessage(EXPLORING_MODE_MESSAGE);
			led7seg_setChar(0xA2, 1); // Inverted E
			pca9532_setLeds(0xFFFF>>numLedToTurnOff, 0xFFFF);
		} else if (mode == SLEEPING) {
			sendUARTMessage(4);
			checkBattery();
			setBatteryMessage();
			displayOledMessage(SLEEPING_MODE_MESSAGE);
			led7seg_setChar(0x32, 1);
			pca9532_setLeds(0xFFFF>>numLedToTurnOff, 0xFFFF);
		}
	}
}
