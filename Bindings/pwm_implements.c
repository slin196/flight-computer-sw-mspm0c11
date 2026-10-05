

#include <stdint.h>
#include "../Core/inc/sys_settings.h"

#ifdef MSPM0C110x

#define ti_dl_dl_timera__include
#include <ti/driverlib/dl_timer.h>
#include <ti\devices\msp\peripherals\hw_gptimer.h>

#define PWM_0_INST ((GPTIMER_Regs *)(0x40090000U))     /*!< Base address of module TIMG8 */
#define PWM_1_INST ((GPTIMER_Regs *)(0x40084000U))     /*!< Base address of module TIMG14 */

void set_duty_cycle(uint8_t motor, uint8_t duty_cylce) {
    switch(motor) {
        case 1: DL_Timer_setCaptureCompareValue(PWM_0_INST, duty_cylce, 0); break;
        case 2: DL_Timer_setCaptureCompareValue(PWM_0_INST, duty_cylce, 1); break;
        case 3: DL_Timer_setCaptureCompareValue(PWM_1_INST, duty_cylce, 0); break;
        case 4: DL_Timer_setCaptureCompareValue(PWM_1_INST, duty_cylce, 1); break;
        //TODO: vll. noch errorcode zurückgeben falls hier keiner der cases ausgelöst wird
    }
}


#endif
