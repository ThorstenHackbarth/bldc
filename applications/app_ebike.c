/*
	Copyright 2018 Thorsten Hackbarth thortsen.hackbarth@gmx.de

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "app.h"

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "mc_interface.h"
#include "timeout.h"
#include "utils.h"
#include "comm_can.h"
#include "hw.h"
#include "commands.h"
#include <math.h>

// Settings
#define MAX_CAN_AGE						0.1
#define MIN_MS_WITHOUT_POWER			500
#define FILTER_SAMPLES					5
#define RPM_FILTER_SAMPLES				8

#define EBIKE_RPM_MIN (900.0f)
//
typedef struct {
	uint32_t update_rate_hz;
} ebike_config;


// Threads
static THD_FUNCTION(ebike_thread, arg);
static THD_WORKING_AREA(ebike_thread_wa, 1024);

// Private variables
static app_configuration *config;

static volatile float setpoint_rpm = 0; // start at zero!
static volatile float setpoint_current = 0.2f;
static volatile float mean_rpm = 0.0f;

static volatile int pas_status = 0;

#define EBIKE_MAIN_UPDATE_RATE_Hz 250 // Hz
#define EBIKE_MAIN_UPDATE_DELAY_ms (1000/EBIKE_MAIN_UPDATE_RATE_Hz)
#define PAS_NOT_CHANGED_MAX_ms 250
static volatile int pas_not_changed_ms = 0;
#define PAS_TICKS_PER_ROT 12
static volatile float pas_speed_rps = 0;


#define PAS_NOT_CHANGED_TICKS_MAX  15 //simple
static volatile int pas_changed_ticks = 0;

static volatile bool stop_now = true;
static volatile bool is_running = false;

static volatile float config_current_limit = 0.0f;

static int loop_cnt = 0;


/* local function declarations -------------------------------------------------------------------------------------*/
void app_ebike_start(void);
void app_ebike_stop(void);
void app_ebike_configure(app_configuration *conf);

/* custom application interface ------------------------------------------------------------------------------------*/
void app_custom_start(void)
{
	app_ebike_start();
}
void app_custom_stop()
{
	app_ebike_stop();
}
void app_custom_configure(app_configuration *conf)
{
	app_ebike_configure(conf);
}


/* implementation --------------------------------------------------------------------------------------------------*/

void app_ebike_configure(app_configuration *conf)
{
	config = conf;
    commands_printf("app_ebike_configure\n");
}

void app_ebike_start(void)
{
    commands_printf("app_ebike_start\n");

	stop_now = false;
	chThdCreateStatic(ebike_thread_wa, sizeof(ebike_thread_wa), NORMALPRIO, ebike_thread, NULL);
    commands_printf("app_ebike_start: done\n");
}

void app_ebike_stop(void)
{
    commands_printf("app_ebike_stop\n");

	stop_now = true;
	while (is_running)
	{
		chThdSleepMilliseconds(1);
	}
}


static void control_loop_simple_current_setter(void)
{
    for(;;)
	{
        loop_cnt++;
		// Sleep for a time according to the specified rate
		/*systime_t sleep_time = CH_CFG_ST_FREQUENCY / EBIKE_MAIN_UPDATE_RATE;// Hz //config.update_rate_hz;

		// At least one tick should be slept to not block the other threads
		if (sleep_time == 0) {
			sleep_time = 10;
		}
		chThdSleep(sleep_time);*/
        chThdSleepMilliseconds(10);

		if (stop_now)
		{
			is_running = false;
            mc_interface_release_motor();
			return;
		}

		float rpm = mc_interface_get_rpm();
		setpoint_rpm = 0.995f * setpoint_rpm + 0.005f * rpm ;
        if (setpoint_rpm < EBIKE_RPM_MIN)  setpoint_rpm = EBIKE_RPM_MIN;

        float err_rpm = setpoint_rpm - rpm;
        float new_setpoint_current = err_rpm / 2000.0f; //0...1000
        setpoint_current = new_setpoint_current;

        // minimum
        float min_current = 0.005f * setpoint_rpm / 1000.0f;

        if (setpoint_current < min_current)
           setpoint_current = min_current;


		if (setpoint_current > 1.0f)
            setpoint_current = 1.0f;

		// PAS
		// 12*2ticks/rot 3rot/s => max 72 changes/s 100Hz is fine
		// 12*2ticks/rot 0.2rot/s => min 4.8 ticks/s => 5Hz => 200ms =>20 cycle
		const bool pas_input = (palReadPad(HW_ICU_GPIO, HW_ICU_PIN) != PAL_LOW);
		if (pas_input != pas_status)
		{
			pas_status = pas_input;
            if ( pas_changed_ticks < (2 * PAS_NOT_CHANGED_TICKS_MAX) )
            {
                pas_changed_ticks += PAS_NOT_CHANGED_TICKS_MAX;
            }
            commands_printf("in %u", pas_changed_ticks);

        }
        else
        {
            if (pas_changed_ticks > 0)
            {
                pas_changed_ticks--;
                commands_printf("  %u", pas_changed_ticks);
            }
        }

        if (pas_changed_ticks <= PAS_NOT_CHANGED_TICKS_MAX)
        {
            setpoint_current = 0;
            setpoint_rpm = 0;
        }
        else
        {
            commands_printf("go %f %f %f %f", setpoint_rpm, rpm,  err_rpm, setpoint_current);
        }

		// TODO: Bremse
        if ( loop_cnt > 100 ) // wait 0.5 s
        {
            if (setpoint_current < 0.001f)
            {
                mc_interface_release_motor();
            }
            else
            {
                mc_interface_set_current_rel(setpoint_current);
            }
        }
        else
        {
            //commands_printf("wait");
        }

		timeout_reset();
	} // end for ever
}

static inline void pas_process(void)
{
    // PAS
    // 12*2ticks/rot 3rot/s => max 72 changes/s 100Hz is fine
    // 12*2ticks/rot 0.2rot/s => min 4.8 ticks/s => 5Hz => 200ms =>20 cycle
    const bool pas_input = (palReadPad(HW_ICU_GPIO, HW_ICU_PIN) != PAL_LOW);
    if (pas_input != pas_status)
    {
        pas_status = pas_input;
        if (pas_not_changed_ms != 0)
        {
            float speed_rps = (1000.0f/(PAS_TICKS_PER_ROT*2)) / pas_not_changed_ms;
            pas_speed_rps = 0.5f * pas_speed_rps + 0.5f * speed_rps;
        }
        pas_not_changed_ms = 0;

        commands_printf("[%u] pas speed %f", loop_cnt, pas_speed_rps);
    }
    else
    {
        pas_not_changed_ms += EBIKE_MAIN_UPDATE_DELAY_ms;
    }

    if (pas_not_changed_ms >= PAS_NOT_CHANGED_MAX_ms)
    {
        if (pas_speed_rps != 0.0f)
        {
            commands_printf("[%u] PAS stop",loop_cnt);
        }
        pas_speed_rps = 0.0f;
    }
}


static void control_loop_current(void)
{
    // 10% per/ s
    static const float RPM_MEAN_FACTOR_IN  = 1.0f / EBIKE_MAIN_UPDATE_RATE_Hz;
    static const float RPM_MEAN_FACTOR_OLD = (1.0f - RPM_MEAN_FACTOR_IN);

    for(;;)
	{
        loop_cnt++;
        chThdSleepMilliseconds(1000/EBIKE_MAIN_UPDATE_RATE_Hz);

		if (stop_now)
		{
			is_running = false;
            mc_interface_release_motor();
			return;
		}

        float pas_speed_old = pas_speed_rps;
        pas_process();
#ifdef PAS_KICKDOWN
        float pas_div = pas_speed_rps - pas_speed_old;
        //pas_div *= 0.5f;
        if (pas_div < 0.0f)
        {
            pas_div *= 0.5f;
        }
        setpoint_rpm = setpoint_rpm + ( setpoint_rpm * pas_div/pas_speed_rps);
#endif // PAS_KICKDOWN

		float rpm = mc_interface_get_rpm();
		setpoint_rpm = RPM_MEAN_FACTOR_OLD * setpoint_rpm + RPM_MEAN_FACTOR_IN * rpm;

        if (setpoint_rpm < EBIKE_RPM_MIN)
        {
            setpoint_rpm = EBIKE_RPM_MIN;
        }

        float err_rpm = setpoint_rpm - rpm;
        float new_setpoint_current = err_rpm / 2000.0f; //0...1000
        // low pass a bit
        setpoint_current = 0.01f* new_setpoint_current + 0.99f * setpoint_current;

        // minimum
        float min_current = 0.0025f * (1.0f + (setpoint_rpm / 1000.0f));

        if (setpoint_current < min_current)
        {
            setpoint_current = min_current;
            //commands_printf("current min %f",setpoint_current );
        }

		if (setpoint_current > 1.0f)
        {
            setpoint_current = 1.0f;
            commands_printf("current max");
        }

        if (pas_speed_rps < 0.2f) // 0.4 rotation/s
        {
            setpoint_current = 0.0f;
            setpoint_rpm = 0.0f;
        }

		// TODO: Bremse

		if ( loop_cnt > 100 ) // wait 0.5 s
        {
            if (setpoint_current < 0.001f)
            {
                mc_interface_release_motor();
            }
            else
            {
                if ((loop_cnt % 10 ) == 0)
                {
                    commands_printf("[%u] %f MOTOR current control rpm %02f current %02f ", loop_cnt, rpm ,setpoint_rpm, setpoint_current);
                }
                mc_interface_set_current_rel(setpoint_current);
            }
        }
        else
        {
            //commands_printf("wait");
        }

		timeout_reset();
	} // end for ever
}


static inline void control_loop_speed_pid_current_limit(void)
{
    //current limit in pid mode https://vesc-project.com/node/199
    mc_configuration *conf = (mc_configuration*)mc_interface_get_configuration();
    float l_in_current_max  = conf->l_in_current_max;
    float lo_in_current_max = conf->lo_in_current_max;
int debug_state=0;
    for(;;)
	{
        loop_cnt++;
        chThdSleepMilliseconds(1000/EBIKE_MAIN_UPDATE_RATE_Hz);

		if (stop_now)
		{
			is_running = false;
            mc_interface_release_motor();
            conf->l_in_current_max  = l_in_current_max;
            conf->lo_in_current_max = lo_in_current_max;
			return;
		}

		float rpm = mc_interface_get_rpm();

        float pas_speed_old = pas_speed_rps;
        pas_process();
        float pas_div = pas_speed_rps - pas_speed_old;
        if (pas_div < 0.0f)
        {
            pas_div *= 0.5f;
        }
        setpoint_rpm = setpoint_rpm + ( setpoint_rpm * pas_div/pas_speed_rps);

		setpoint_rpm = 0.995f * setpoint_rpm + 0.005f * rpm ;
        if (setpoint_rpm < EBIKE_RPM_MIN)
        {
            setpoint_rpm = EBIKE_RPM_MIN;// TODO STARTUP_RPM
        }

        float err_rpm = setpoint_rpm - rpm;
        float new_setpoint_current = err_rpm / 2000.0f; //0...1000
        setpoint_current = new_setpoint_current;

        // minimum
        float min_current = 0.005 + 0.005f * setpoint_rpm / EBIKE_RPM_MIN;

        if (setpoint_current < min_current)
        {
            setpoint_current = min_current;
        }

		if (setpoint_current > 1.0f)
        {
            setpoint_current = 1.0f;
        }
		// TODO
		// Drehmoment Sensor-> current

//#ifdef ADC_IND_EXT2
//		float adc_input = (float)ADC_Value[ADC_IND_EXT2];
//		adc_input /= 4095.0f;
//		adc_input *= V_REG;
//      setpoint_current = 0.2f;
//#endif


        if (pas_speed_rps < 0.4f) // 0.4 rotation/s
        {
            setpoint_current = 0.0f;
            setpoint_rpm = 0.0f;
        }


        //commands_printf("go %f %f %f %f", setpoint_rpm, rpm,  err_rpm, setpoint_current);

		// TODO: Bremshebelschalter

        // set motor output after startup delay
        if ( loop_cnt >= EBIKE_MAIN_UPDATE_RATE_Hz ) // wait 1s for controller startup
        {
            if (setpoint_current < 0.001f)
            {
                if (debug_state != 0)
                {
                    commands_printf("[%u] MOTOR stop",loop_cnt);
                    debug_state = 0;
                }
                mc_interface_release_motor();
            }
            /*else if (setpoint_rpm < EBIKE_RPM_MIN) // startup in current cmode
            {
                //if (debug_state != 1)
                {
                    commands_printf("[%u] %f MOTOR current control rpm %02f current %02f ", loop_cnt, rpm ,setpoint_rpm, setpoint_current);
                    debug_state = 1;
                }
                conf->l_in_current_max = l_in_current_max;
                conf->lo_in_current_max = lo_in_current_max;
                mc_interface_set_current_rel(setpoint_current);
            }*/
            else
            {
                //if (debug_state != 2)
                {
                    commands_printf("[%u] %f MOTOR speed control rpm %02f current %02f ", loop_cnt, rpm, setpoint_rpm, setpoint_current);
                    debug_state = 2;
                }
                // TODO: limit wattage
                conf->l_in_current_max  = l_in_current_max  * setpoint_current;
                conf->lo_in_current_max = lo_in_current_max * setpoint_current;
                mc_interface_set_pid_speed(setpoint_rpm);
            }
        }
    	timeout_reset();
	} // end for ever
}

static THD_FUNCTION(ebike_thread, arg)
{
    (void)arg;

    commands_printf("ebike_thread\n");
	chRegSetThreadName("APP_EBIKE");
	is_running = true;

	//mc_interface_set_current_rel(1.0f);
	//PAS pedal assist sensor input on servo port
	palSetPadMode(HW_ICU_GPIO, HW_ICU_PIN, PAL_MODE_INPUT_PULLUP);

	// UART RX port as input for break sensor
	palSetPadMode(HW_UART_RX_PORT, HW_UART_RX_PIN, PAL_MODE_INPUT_PULLUP);

    //control_loop_speed_pid_current_limit();
    control_loop_current();
    // control_loop_simple_current_setter();

}

