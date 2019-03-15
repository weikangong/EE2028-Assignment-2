/*****************************************************************************
 *   GPIO Speaker & Tone Example + GPIO Interrupt
 *
 *   CK Tham, EE2024
 *
 ******************************************************************************/

#include <stdio.h>
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_timer.h"

// Function definitions
#define NOTE_PIN_HIGH() GPIO_SetValue(0, 1<<26);
#define NOTE_PIN_LOW()  GPIO_ClearValue(0, 1<<26);

// Interval in us
static uint32_t notes[] = {
        2272, // A - 440 Hz
        2024, // B - 494 Hz
        3816, // C - 262 Hz
        3401, // D - 294 Hz
        3030, // E - 330 Hz
        2865, // F - 349 Hz
        2551, // G - 392 Hz
        1136, // a - 880 Hz
        1012, // b - 988 Hz
        1912, // c - 523 Hz
        1703, // d - 587 Hz
        1517, // e - 659 Hz
        1432, // f - 698 Hz
        1275, // g - 784 Hz
};

static void playNote(uint32_t note, uint32_t durationMs)
{
    uint32_t t = 0;

    if (note > 0) {
        while (t < (durationMs*1000)) {
            NOTE_PIN_HIGH();
            Timer0_us_Wait(note/2); // us timer

            NOTE_PIN_LOW();
            Timer0_us_Wait(note/2);

            t += note;
        }
    }
    else {
    	Timer0_Wait(durationMs); // ms timer
    }
}

static uint32_t getNote(uint8_t ch)
{
    if (ch >= 'A' && ch <= 'G')
        return notes[ch - 'A'];

    if (ch >= 'a' && ch <= 'g')
        return notes[ch - 'a' + 7];

    return 0;
}

static uint32_t getDuration(uint8_t ch)
{
    if (ch < '0' || ch > '9')
        return 400;
    /* number of ms */
    return (ch - '0') * 200;
}

static uint32_t getPause(uint8_t ch)
{
    switch (ch) {
    case '+':
        return 0;
    case ',':
        return 5;
    case '.':
        return 20;
    case '_':
        return 30;
    default:
        return 5;
    }
}

static void playSong(uint8_t *song) {
    uint32_t note = 0;
    uint32_t dur  = 0;
    uint32_t pause = 0;

    /*
     * A song is a collection of tones where each tone is
     * a note, duration and pause, e.g.
     *
     * "E2,F4,"
     */

    while(*song != '\0') {
        note = getNote(*song++);
        if (*song == '\0')
            break;
        dur  = getDuration(*song++);
        if (*song == '\0')
            break;
        pause = getPause(*song++);

        playNote(note, dur);
        Timer0_Wait(pause);
    }
}

static uint8_t * song = (uint8_t*)"C1.C1,D2,C2,F2,E4,";

static void init_GPIO(void)
{
	// Initialize button SW4 (not really necessary since default configuration)
	PINSEL_CFG_Type PinCfg;
	PinCfg.Funcnum = 0;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 1;
	PinCfg.Pinnum = 31;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(1, 1<<31, 0);

    /* ---- Speaker ------> */
    GPIO_SetDir(2, 1<<0, 1);
    GPIO_SetDir(2, 1<<1, 1);

    GPIO_SetDir(0, 1<<27, 1);
    GPIO_SetDir(0, 1<<28, 1);
    GPIO_SetDir(2, 1<<13, 1);

    // Main tone signal : P0.26
    GPIO_SetDir(0, 1<<26, 1);

    GPIO_ClearValue(0, 1<<27); //LM4811-clk
    GPIO_ClearValue(0, 1<<28); //LM4811-up/dn
    GPIO_ClearValue(2, 1<<13); //LM4811-shutdn
    /* <---- Speaker ------ */
}

// EINT3 Interrupt Handler
void EINT3_IRQHandler(void)
{
//	int i;
	// Determine whether GPIO Interrupt P2.10 has occurred
	if ((LPC_GPIOINT->IO2IntStatF>>10)& 0x1)
	{
        printf("GPIO Interrupt 2.10\n");
//      for (i=0;i<9999999;i++);
        // Clear GPIO Interrupt P2.10
        LPC_GPIOINT->IO2IntClr = 1<<10;
	}
}

int main (void) {

    uint8_t btn1 = 1;

    init_GPIO();

    // Enable GPIO Interrupt P2.10
    LPC_GPIOINT->IO2IntEnF |= 1<<10;

    // Enable EINT3 interrupt
    NVIC_EnableIRQ(EINT3_IRQn);

    while (1)
    {
    	btn1 = (GPIO_ReadValue(1) >> 31) & 0x01;

        if (btn1 == 0)
        {
            playSong(song);
        }
    }
}
