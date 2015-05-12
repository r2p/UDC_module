#include "ch.h"
#include "hal.h"

/*
 * Application entry point.
 */
int main(void) {

	halInit();
	chSysInit();

	for (;;) {
		palTogglePad(LED_GPIO, LED_PIN);
		chThdSleepMilliseconds(500);
	}

	return CH_SUCCESS;
}
