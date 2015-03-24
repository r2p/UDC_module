#ifndef _CONFIG_H_
#define _CONFIG_H_

#include "qei.h"

namespace r2p {
/*
 * PWM configuration.
 */
PWMConfig pwmcfg = { STM32_SYSCLK, /* 72MHz PWM clock frequency.   */
4096, /* 12-bit PWM, 17KHz frequency. */
NULL, {
                { PWM_OUTPUT_ACTIVE_HIGH | PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH, NULL },
                { PWM_OUTPUT_ACTIVE_HIGH | PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH, NULL },
                { PWM_OUTPUT_DISABLED, NULL }, { PWM_OUTPUT_DISABLED, NULL } }, 0,
#if STM32_PWM_USE_ADVANCED
                72, /* XXX 1uS deadtime insertion   */
#endif
                };

QEIConfig qeicfg = {
        QEI_MODE_QUADRATURE,
        QEI_BOTH_EDGES,
        QEI_DIRINV_FALSE,
};

}

#endif /* _CONFIG_H_ */
