#include <cstdio>

#include "ch.h"
#include "hal.h"

#include "qei.h"

#include <r2p/Middleware.hpp>
#include <r2p/node/led.hpp>
#include <r2p/msg/std_msgs.hpp>
#include <pid.hpp>
#include <r2p/msg/motor.hpp>

#ifndef R2P_MODULE_NAME
#define R2P_MODULE_NAME "uDC"
#endif

#define _TICKS 48.0f
#define _RATIO 30.0f
#define _PI 3.14159265359f

#define R2T(r) ((r / (2 * _PI)) * (_TICKS * _RATIO))
#define T2R(t) ((t / (_TICKS * _RATIO)) * (2 * _PI))

static WORKING_AREA(wa_info, 1024);
static r2p::RTCANTransport rtcantra(RTCAND1);

RTCANConfig rtcan_config = { 1000000, 100, 60 };

r2p::Middleware r2p::Middleware::instance(R2P_MODULE_NAME, "BOOT_"R2P_MODULE_NAME);

PID speed_pid;
int16_t pwm = 100;

/*
 * PWM cyclic callback.
 */
static void pwmcb(PWMDriver *pwmp) {

	(void) pwmp;
	chSysLockFromIsr()
	;
	if (pwm >= 0) {
		pwm_lld_enable_channel(&PWMD1, 0, pwm);
		pwm_lld_enable_channel(&PWMD1, 1, 0);
	} else {
		pwm_lld_enable_channel(&PWMD1, 0, 0);
		pwm_lld_enable_channel(&PWMD1, 1, -pwm);
	}
	chSysUnlockFromIsr();
}

/*
 * PWM configuration.
 */
static PWMConfig pwmcfg = { 36000000, /* 36MHz PWM clock.   */
4096, /* 12-bit PWM, 9KHz frequency. */
pwmcb, { { PWM_OUTPUT_ACTIVE_HIGH, NULL }, { PWM_OUTPUT_ACTIVE_HIGH, NULL }, { PWM_OUTPUT_DISABLED, NULL }, {
	PWM_OUTPUT_DISABLED, NULL } }, 0, };


/*
 * PWM subscriber node
 */

msg_t pwm_node(void * arg) {
	uint8_t index = *(reinterpret_cast<uint8_t *>(arg));
	r2p::Node node("pwm2sub");
	r2p::Subscriber<r2p::PWM2Msg, 5> pwm_sub;
	r2p::PWM2Msg * msgp;

	(void) arg;

	chRegSetThreadName("pwm_node");

	/* Enable the h-bridge. */
	palSetPad(GPIOB, GPIOB_MOTOR_ENABLE); palClearPad(GPIOA, GPIOA_MOTOR_D1);
	chThdSleepMilliseconds(500);
	pwmStart(&PWM_DRIVER, &pwmcfg);

	node.subscribe(pwm_sub, "pwm");

	for (;;) {
		if (node.spin(r2p::Time::ms(1000))) {
			if (pwm_sub.fetch(msgp)) {
				pwm = msgp->value[index];
				chSysLock()
				;
				if (pwm >= 0) {
					pwm_lld_enable_channel(&PWMD1, 0, msgp->value[index]);
					pwm_lld_enable_channel(&PWMD1, 1, 0);
				} else {
					pwm_lld_enable_channel(&PWMD1, 0, 0);
					pwm_lld_enable_channel(&PWMD1, 1, -msgp->value[index]);
				}
				chSysUnlock();
				pwm_sub.release(*msgp);
			}
		} else {
			// Stop motor if no messages for 1000 ms
			pwm_lld_disable_channel(&PWM_DRIVER, 0);
			pwm_lld_disable_channel(&PWM_DRIVER, 1);
		}
	}
	return CH_SUCCESS;
}

/*
 * PID node
 */

struct pid_conf {
	float k;
	float ti;
	float td;
	float ts;
};

bool enc_callback(const r2p::EncoderMsg &msg) {

	pwm = speed_pid.update(msg.delta);
	chSysLock()
	;

	if (pwm > 0) {
		pwm_lld_enable_channel(&PWM_DRIVER, 1, pwm);
		pwm_lld_enable_channel(&PWM_DRIVER, 0, 0);
	} else {
		pwm_lld_enable_channel(&PWM_DRIVER, 1, 0);
		pwm_lld_enable_channel(&PWM_DRIVER, 0, -pwm);
	}
	chSysUnlock();

	return true;
}

msg_t pid_node(void * arg) {
	pid_conf * conf = reinterpret_cast<pid_conf *>(arg);
	r2p::Node node("pid");
	r2p::Subscriber<r2p::SpeedMsg, 5> speed_sub;
	r2p::Subscriber<r2p::EncoderMsg, 5> enc_sub(enc_callback);
	r2p::SpeedMsg * msgp;
	r2p::Time last_setpoint(0);

	(void) arg;
	chRegSetThreadName("pid");

	speed_pid.config(conf->k, conf->ti, conf->td, conf->ts, -4095.0, 4095.0);

	/* Enable the h-bridge. */
	palSetPad(GPIOB, GPIOB_MOTOR_ENABLE); palClearPad(GPIOA, GPIOA_MOTOR_D1);
	chThdSleepMilliseconds(500);
	pwmStart(&PWM_DRIVER, &pwmcfg);

	node.subscribe(speed_sub, "speed");
	node.subscribe(enc_sub, "encoder");

	for (;;) {
		if (node.spin(r2p::Time::ms(1000))) {
			if (speed_sub.fetch(msgp)) {
				speed_pid.set(msgp->value);
				last_setpoint = r2p::Time::now();
				speed_sub.release(*msgp);
			} else if (r2p::Time::now() - last_setpoint > r2p::Time::ms(1000)) {
				speed_pid.set(0);
			}
		} else {
			// Stop motor if no messages for 1000 ms
			pwm_lld_disable_channel(&PWM_DRIVER, 0);
			pwm_lld_disable_channel(&PWM_DRIVER, 1);
		}
	}

	return CH_SUCCESS;
}

/*
 * QEI publisher node
 */

QEIConfig qeicfg = { QEI_MODE_QUADRATURE, QEI_BOTH_EDGES, QEI_DIRINV_FALSE, };

msg_t encoder_node(void *arg) {
	r2p::Node node("encoder");
	r2p::Publisher<r2p::EncoderMsg> enc_pub;
	systime_t time;
	qeidelta_t delta;
	r2p::EncoderMsg *msgp;

	(void) arg;
	chRegSetThreadName("encoder");

	/* Enable the QEI driver. */
	qeiInit();
	qeiStart(&QEI_DRIVER, &qeicfg);
	qeiEnable (&QEI_DRIVER);

	node.advertise(enc_pub, "encoder");

	for (;;) {
		time = chTimeNow();
		delta = qeiUpdate(&QEI_DRIVER);

		if (enc_pub.alloc(msgp)) {
			msgp->delta = T2R(delta);
			enc_pub.publish(*msgp);
		}

		time += MS2ST(50);
		chThdSleepUntil(time);
	}

	return CH_SUCCESS;
}

/*
 * Test node
 */
msg_t test_pub_node(void *arg) {
	r2p::Node node("test_pub");
	r2p::Publisher<r2p::String64Msg> pub;
	r2p::String64Msg * msgp;
	uint16_t * uuid = (uint16_t *)0x1FFFF7AC;

	(void) arg;
	chRegSetThreadName("test_pub");

	node.advertise(pub, "test");

	while (!pub.alloc(msgp)) chThdSleepMilliseconds(1000);

	sprintf(msgp->data, "\n\n"R2P_MODULE_NAME" module [0x%x 0x%x 0x%x 0x%x 0x%x 0x%x]", uuid[0], uuid[1], uuid[2], uuid[3], uuid[4], uuid[5]);
	pub.publish(msgp);
	chThdSleepMilliseconds(100);

	return CH_SUCCESS;
}

/*
 * Application entry point.
 */
extern "C" {
int main(void) {

	halInit();
	chSysInit();

	r2p::Middleware::instance.initialize(wa_info, sizeof(wa_info), r2p::Thread::LOWEST);
	rtcantra.initialize(rtcan_config);
	r2p::Middleware::instance.start();

	r2p::ledsub_conf ledsub_conf = { "led" };
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(512), NORMALPRIO, r2p::ledsub_node, &ledsub_conf);

	r2p::Thread::create_heap(NULL, THD_WA_SIZE(2048), NORMALPRIO + 1, test_pub_node, NULL);

	chThdSleepMilliseconds(2000);

	pid_conf pid_conf = {100, 0, 0, 0.01};
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(2048), NORMALPRIO + 1, pid_node, &pid_conf);

	r2p::Thread::create_heap(NULL, THD_WA_SIZE(2048), NORMALPRIO + 2, encoder_node, NULL);

	for (;;) {
		r2p::Thread::sleep(r2p::Time::ms(500));
	}

	return CH_SUCCESS;
}
}
