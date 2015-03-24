#include "ch.h"
#include "hal.h"

/*
 * Application entry point.
 */
int main(void) {

	halInit();
	chSysInit();

	for (;;) {
		palTogglePad(GPIOC, GPIOC_LED);
		chThdSleepMilliseconds(500);
	}

	return CH_SUCCESS;
}
