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
#define CHAR_HEIGHT 12
#define LIGHT_THRESHOLD 30

#define TEMP_COUNT 500
#define TEMP_CONSTANT 20
#define TEMP_DIVIDER 10
#define TEMP_K_TO_C_CONSTANT 2731

typedef enum {
	ORBITING,
	ORBITING_TO_LANDING,
	LANDING,
	EXPLORING,
	SLEEPING
} mode_type;
volatile uint32_t msTicks;
volatile mode_type mode;
int8_t x[1];
int8_t y[1];
int8_t z[1];
uint32_t batteryInterval; // In ms
uint32_t transmissionInterval; // In ms
uint8_t numLedToTurnOff;
float batteryLevel = 100.0;
static uint32_t t1 = 0;
static uint32_t t2 = 0;

static uint8_t CLEAR_SCREEN_FLAG;
static uint8_t ORBITING_FLAG = 1;
static uint8_t ORBITING_TO_LANDING_FLAG = 0;
static uint8_t LANDING_FLAG = 1;
static uint8_t TILT_THRESHOLD_FLAG = 0;
static uint8_t TILT_CLEAR_FLAG = 0;
static uint8_t LIGHT_THRESHOLD_FLAG = 0;
static uint8_t ACC_FLAG = 0;
static uint8_t TEMP_STATE_FLAG = 0;
static uint8_t TEMP_CHANGE_FLAG = 0;

static uint8_t ORBIT_MODE_MESSAGE[][40] = {
		"Orbiting Mode",
		"Press SW3 to",
		"land",
		""
		""
};
static uint8_t ORBITING_TO_LANDING_MODE_MESSAGE[][40] ={
		"ENTERING",
		"LANDING MODE",
		"",
		"",
		"",
		""
};
static uint8_t LANDING_MODE_MESSAGE[][40] = {
		"LANDING",
		"",
		"",
		"",
		"",
		""
};
static uint8_t EXPLORING_MODE_MESSAGE[][40] = {
		"EXPLORING",
		"",
		"",
		"",
		"",
		""
};
uint8_t SLEEPING_MODE_MESSAGE[][40] = {
		"SLEEPING",
		"",
		"",
		"",
		"",
		""
};

static unsigned char TILT_THRESHOLD_TXT[] = "Poor Landing Attitude \r\n";
static unsigned char TILT_CLEAR_TXT[] = "Safe to Land! \r\n";
static unsigned char ORBITING_TXT[] = "Start Orbiting, waiting for Landing \r\n";
static unsigned char LANDING_TXT[] = "LANDING Mode \r\n";
static unsigned char EXPLORING_TXT[] = "EXPLORING Mode \r\n";
static unsigned char SLEEPING_TXT[] = "System is sleeping \r\n";
static unsigned char LIGHT_THRESHOLD_TXT[] = "Light threshold met \r\n";
static unsigned char CLEAR_SCREEN_TXT[] = "\x1b[2J";

const uint8_t invertedNums[] = { 0x24, 0x7D, 0xE0, 0x70, 0x39, 0x32, 0x23, 0x7C,
		0x20, 0x38, };

unsigned int getPrescalarForUs(uint8_t timerPclkBit) {
	unsigned int pclk, prescalarForUs;
	pclk = (LPC_SC ->PCLKSEL0 >> timerPclkBit) & 0x03; // get the pclk info for required timer

	switch (pclk) { // Decode the bits to determine the pclk
	case 0x00:
		pclk = SystemCoreClock / 4;
		break;
	case 0x01:
		pclk = SystemCoreClock;
		break;
	case 0x02:
		pclk = SystemCoreClock / 2;
		break;
	case 0x03:
		pclk = SystemCoreClock / 8;
		break;
	default:
		pclk = SystemCoreClock / 4;
		break;
	}
	return pclk / 1000000 - 1; // Prescalar for 1us (1000000Counts/sec)
}

void SysTick_Handler(void) {
	msTicks++;
}

uint32_t getTicks(void) {
	return msTicks;
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

void init_uart(void) {
	UART_CFG_Type uartCfg;
	uartCfg.Baud_rate = 115200;
	uartCfg.Databits = UART_DATABIT_8;
	uartCfg.Parity = UART_PARITY_NONE;
	uartCfg.Stopbits = UART_STOPBIT_1;
	// Pin select for uart3
	pinsel_uart3();
	// Supply power & setup working parameters for uart3
	UART_Init(LPC_UART3, &uartCfg);
	// Enable transmit for uart3
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
	// SW3: P2.10
	PINSEL_CFG_Type PinCfg;
	PinCfg.Funcnum = 0;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 10;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(2, 1 << 10, 0);

	// SW4: P1.31
	PinCfg.Portnum = 1;
	PinCfg.Pinnum = 31;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(1, 1 << 31, 0);

	// Red LED: P2.0
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(2, 1, 1);

	// Blue LED: P0.26
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 26;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(0, (1 << 26), 1);

	// Green LED: P2.1
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 1;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(2, (1 << 1), 1);
}

void interrupt_init(void) {
	// Clear and enable SW3 interrupt
	LPC_GPIOINT ->IO2IntClr = 1 << 10;
	LPC_GPIOINT ->IO2IntEnR |= 1 << 10;

	// Clear and enable light interrupt
	LPC_GPIOINT ->IO2IntClr = 1 << 5;
	LPC_GPIOINT ->IO2IntEnF |= 1 << 5;

	LPC_GPIOINT ->IO0IntClr = 1 << 2;
	LPC_GPIOINT ->IO0IntClr = 1 << 2;
	LPC_GPIOINT ->IO0IntEnF |= 1 << 2;
	LPC_GPIOINT ->IO0IntEnR |= 1 << 2;

	// Enable Timer1 interrupt
	LPC_SC ->PCONP |= (1 << SBIT_TIMER0); // Power ON Timer0, 1

	LPC_TIM1 ->MCR = (1 << SBIT_MR0I) | (1 << SBIT_MR0R); // Clear TC on MR0 match and Generate Interrupt
	LPC_TIM1 ->PR = getPrescalarForUs(PCLK_TIMER1); // Prescalar for 1us
	LPC_TIM1 ->MR0 = MiliToMicroSec(500); // 500ms delay
	LPC_TIM1 ->TCR = (1 << SBIT_CNTEN); // Start timer by setting the Counter Enable

	NVIC_ClearPendingIRQ(EINT3_IRQn);
	NVIC_EnableIRQ(EINT3_IRQn);

	NVIC_ClearPendingIRQ(TIMER1_IRQn);
	NVIC_EnableIRQ(TIMER1_IRQn);

	NVIC_SetPriorityGrouping(5);
	NVIC_SetPriority(TIMER1_IRQn, 0x04);
}

void init(void) {
	SysTick_Config(SystemCoreClock / 1000);

	init_ssp();
	init_i2c();
	init_GPIO();

	init_uart();
	led7seg_init();
	oled_init();
	acc_init();
	temp_init(getTicks);
	light_enable();
	light_setRange(LIGHT_RANGE_4000);
	light_setLoThreshold(LIGHT_THRESHOLD);
	light_clearIrqStatus();

	interrupt_init();

	// Set default mode
	setMode(ORBITING);
	sendUARTMessage(ORBITING_TXT);
}

void setMode(mode_type modeToSet) {
	mode = modeToSet;
	CLEAR_SCREEN_FLAG = 1;
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

void checkTiltThreshold() {
	if ((x[0] / 63.0) > (TILT_THRESHOLD / 60.0)
			| (y[0] / 63.0) > (TILT_THRESHOLD / 60.0)) {
		if (!TILT_THRESHOLD_FLAG) CLEAR_SCREEN_FLAG = 1;
		TILT_THRESHOLD_FLAG = 1;
		ACC_FLAG = 0;
	} else {
		if (TILT_THRESHOLD_FLAG) TILT_CLEAR_FLAG = 1;
		TILT_THRESHOLD_FLAG = 0;
		ACC_FLAG = 1;
	}
}


void recordTempTime() {
	static uint16_t count = 0;
	if (count == TEMP_COUNT) {
		t1 = (uint32_t) t2;
		t2 = getTicks();
		count = 0;
		TEMP_CHANGE_FLAG = 1;
	} else {
		count++;
	}
}

int32_t readTemp() {
	static int32_t temp = 0;
	if (TEMP_CHANGE_FLAG) {
		uint32_t diff;
		if (t2 > t1) diff = t2 - t1;
		else diff = (0xFFFFFFFF - t1 + 1) + t2;

		temp = ((TEMP_CONSTANT * diff) / TEMP_DIVIDER
				- TEMP_K_TO_C_CONSTANT);
		TEMP_CHANGE_FLAG = 0;
	}

	return temp;
}

void setAccAndLightMessage() {
	if (TILT_THRESHOLD_FLAG) {
		sprintf(LANDING_MODE_MESSAGE[1], "Poor");
		sprintf(LANDING_MODE_MESSAGE[2], "Landing");
		sprintf(LANDING_MODE_MESSAGE[3], "Altitude");
	} else {
		sprintf(LANDING_MODE_MESSAGE[1], "Acc-X: %2.2f g\r", x[0] / 63.0);
		sprintf(LANDING_MODE_MESSAGE[2], "Acc-Y: %2.2f g\r", y[0] / 63.0);
		sprintf(LANDING_MODE_MESSAGE[3], "Acc-Z: %2.2f g\r", z[0] / 63.0);
		sprintf(LANDING_MODE_MESSAGE[4], "Light: %lu lux\r\n", (unsigned long) light_read());
	}
}

void setTempMessage() {
	sprintf(EXPLORING_MODE_MESSAGE[2], "Temp: %2.1f deg  \r\n", readTemp() / 10.0);
}

void setBatteryMessage() {
	sprintf(EXPLORING_MODE_MESSAGE[1], "Batt: %2.2f%%   ", batteryLevel);
	sprintf(SLEEPING_MODE_MESSAGE[1], "Batt: %2.2f%%   ", batteryLevel);
}

void sendUARTMessage(uint8_t *message) {
	UART_Send(LPC_UART3, (uint8_t *) message,
			strlen(message), BLOCKING);
}

void checkBattery() {
	uint8_t sw4 = (GPIO_ReadValue(1) >> 31) & 0x01;
	if (sw4 == 0) {
		batteryLevel = batteryLevel + 6.25 <= 100 ? batteryLevel + 6.25 : 100;
		batteryInterval = 0;
		Timer0_Wait(200);
	}

	if (batteryInterval >= 10000) {
		batteryLevel = batteryLevel - 12.5 >= 0 ? batteryLevel - 12.5 : 0;
		batteryInterval = 0;
	}

	if (batteryLevel <= 12.5 && mode != SLEEPING) {
		setMode(SLEEPING);
		sendUARTMessage(SLEEPING_TXT);
	}

	if (batteryLevel > 12.5 && mode != EXPLORING) {
		setMode(EXPLORING);
		sendUARTMessage(EXPLORING_TXT);
	}

	numLedToTurnOff = NUM_LED - batteryLevel / PERCENT_PER_LED;
}

void EINT3_IRQHandler() {
	// SW3 interrupt
	if ((LPC_GPIOINT ->IO2IntStatR >> 10) & 0x1) {
		static uint32_t lastTicks = 0;
		uint32_t currTicks = getTicks();
		if (currTicks - lastTicks <= 1000 && mode == ORBITING)
			ORBITING_TO_LANDING_FLAG = 1;
		lastTicks = currTicks;
		LPC_GPIOINT ->IO2IntClr = 1 << 10;
	}

	// Light interrupt
	if ((LPC_GPIOINT ->IO2IntStatF >> 5) & 0x1) {
		if(light_getIrqStatus()) {
			if (mode == LANDING) LIGHT_THRESHOLD_FLAG = 1;
			light_clearIrqStatus();
		}
		LPC_GPIOINT->IO2IntClr = 1<<5;
	}

	// Temperature interrupt
	if ((LPC_GPIOINT ->IO0IntStatR >> 2) & 0x1) {
		if (!TEMP_STATE_FLAG) {
			recordTempTime();
			TEMP_STATE_FLAG = 1;
		}
		LPC_GPIOINT ->IO0IntClr = 1 << 2;
	}

	if ((LPC_GPIOINT ->IO0IntStatF >> 2) & 0x1) {
		TEMP_STATE_FLAG = 0;
		LPC_GPIOINT ->IO0IntClr = 1 << 2;
	}
}

void TIMER1_IRQHandler() {
	LPC_TIM1 ->IR = LPC_TIM1 ->IR; // Clear Timer1 interrupt
	uint32_t isBlue = (GPIO_ReadValue(0) >> 26) & 0x01;
	switch (mode) {
	case ORBITING: // BLINK_BLUE
		batteryInterval = 0;
		transmissionInterval = 0;
		if (isBlue) GPIO_ClearValue(0, 1 << 26);
		else GPIO_SetValue(0, (1 << 26));
		break;
	case ORBITING_TO_LANDING: // BLINK_BLUE
		batteryInterval = 0;
		transmissionInterval = 0;
		if (isBlue) GPIO_ClearValue(0, 1 << 26);
		else GPIO_SetValue(0, (1 << 26));
		break;
	case LANDING: // ALTERNATE_LED
		batteryInterval = 0;
		transmissionInterval += 500;
		if (isBlue) {
			GPIO_ClearValue(0, 1 << 26);
			GPIO_SetValue(2, 1);
		} else {
			GPIO_SetValue(0, (1 << 26));
			GPIO_ClearValue(2, 1 << 0);
		}
		break;
	case EXPLORING:
		batteryInterval += 500;
		transmissionInterval += 500;
		// BLUE
		GPIO_SetValue(0, (1 << 26));
		GPIO_ClearValue(2, 1 << 0);
		break;
	case SLEEPING:
		batteryInterval += 500;
		transmissionInterval = 0;
		// BLUE
		GPIO_SetValue(0, (1 << 26));
		GPIO_ClearValue(2, 1 << 0);
		break;
	default:
		batteryInterval = 0;
		transmissionInterval = 0;
		break;
	}
}

void orbiting() {
	displayOledMessage(ORBIT_MODE_MESSAGE);
	led7seg_setChar(0x24, 1); // Inverted O
	if (ORBITING_TO_LANDING_FLAG) setMode(ORBITING_TO_LANDING);
}

void orbitingToLanding() {
	displayOledMessage(ORBITING_TO_LANDING_MODE_MESSAGE);
	countdown();
	setMode(LANDING);
	sendUARTMessage(LANDING_TXT);
}

void landing() {
	acc_read(x, y, z);
	checkTiltThreshold();
	setAccAndLightMessage();
	displayOledMessage(LANDING_MODE_MESSAGE);
	led7seg_setChar(0xA7, 1); // Inverted L

	if (TILT_THRESHOLD_FLAG) {
		sendUARTMessage(TILT_THRESHOLD_TXT);
	} else if (TILT_CLEAR_FLAG) {
		sendUARTMessage(CLEAR_SCREEN_TXT);
		sendUARTMessage(TILT_CLEAR_TXT);
		TILT_CLEAR_FLAG = 0;
	}

	if (transmissionInterval >= 5000 && !TILT_THRESHOLD_FLAG) {
		sendUARTMessage(LANDING_MODE_MESSAGE[1]);
		sendUARTMessage(LANDING_MODE_MESSAGE[2]);
		sendUARTMessage(LANDING_MODE_MESSAGE[3]);
		sendUARTMessage(LANDING_MODE_MESSAGE[4]);
		transmissionInterval = 0;
	}

	if (LIGHT_THRESHOLD_FLAG) {
		setMode(EXPLORING);
		sendUARTMessage(LIGHT_THRESHOLD_TXT);
		sendUARTMessage(EXPLORING_TXT);
	}
}

void exploring() {
	setTempMessage();
	checkBattery();
	setBatteryMessage();
	displayOledMessage(EXPLORING_MODE_MESSAGE);
	led7seg_setChar(0xA2, 1); // Inverted E
	pca9532_setLeds(0xFFFF >> numLedToTurnOff, 0xFFFF);

	if (transmissionInterval >= 5000) {
		sendUARTMessage(EXPLORING_MODE_MESSAGE[1]);
		sendUARTMessage(EXPLORING_MODE_MESSAGE[2]);
		transmissionInterval = 0;
	}
}

void sleeping() {
	checkBattery();
	setBatteryMessage();
	displayOledMessage(SLEEPING_MODE_MESSAGE);
	led7seg_setChar(0x32, 1); // Inverted S
	pca9532_setLeds(0xFFFF >> numLedToTurnOff, 0xFFFF);
}

int main() {
	init();

	while (1) {
		if (CLEAR_SCREEN_FLAG) {
			oled_clearScreen(OLED_COLOR_BLACK);
			CLEAR_SCREEN_FLAG = 0;
		}

		switch (mode) {
			case ORBITING:
				orbiting();
				break;
			case ORBITING_TO_LANDING:
				orbitingToLanding();
				break;
			case LANDING:
				landing();
				break;
			case EXPLORING:
				exploring();
				break;
			case SLEEPING:
				sleeping();
				break;
			default:
				break;
		}
	}
}
