/*
 * Copyright (c) 2024-present LAAS-CNRS
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 2.1 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @brief  This file deploys the code for discussing with a python script for
 *         hardware in the loop applications. Please check its documentation on
 *         the readme file or at: https://docs.owntech.org/
 *
 * @author Clément Foucher <clement.foucher@laas.fr>
 * @author Luiz Villa <luiz.villa@laas.fr>
 */

/* --------------OWNTECH APIs---------------------------------- */
#include "SpinAPI.h"
#include "TaskAPI.h"
#include "ShieldAPI.h"
#include "pid.h"
#include "comm_protocol.h"

/* Number of point to record */
#define RECORD_SIZE 128


/* --------------SETUP FUNCTIONS DECLARATION------------------- */
/* Setups the hardware and software of the system */
void setup_routine();

/* --------------LOOP FUNCTIONS DECLARATION-------------------- */
/* Code to be executed in the background task */
void loop_application_task();
/* Code to be executed in the background task */
void loop_communication_task();
/* Code to be executed in real time in the critical task */
void loop_control_task();


/* -------------USER VARIABLES DECLARATIONS-------------------- */

/* [us] period of the control task */
static uint32_t control_task_period = 100;
/* [bool] state of the PWM (ctrl task) */
static bool pwm_enable_leg_1 = false;
/* [bool] state of the PWM (ctrl task) */
static bool pwm_enable_leg_2 = false;

/* Measurement  variables */

float32_t V1_low_value;
float32_t V2_low_value;
float32_t I1_low_value;
float32_t I2_low_value;
float32_t I_high_value;
float32_t V_high_value;

float32_t T1_value;
float32_t T2_value;

float32_t delta_V1;
float32_t V1_max = 0.0;
float32_t V1_min = 0.0;

float32_t delta_V2;
float32_t V2_max = 0.0;
float32_t V2_min = 0.0;

uint16_t dead_time_max;
uint16_t dead_time_min;
int16_t phase_shift_max;
int16_t phase_shift_min;


int8_t AppTask_num, CommTask_num;

static float32_t acquisition_moment = 0.06;

/* Temporary storage for measured value (ctrl task) */
static float meas_data;

float32_t starting_duty_cycle = 0.1;

static float32_t kp = 0.000215;
static float32_t Ti = 7.5175e-5;
static float32_t Td = 0.0;
static float32_t N = 0.0;
static float32_t upper_bound = 1.0F;
static float32_t lower_bound = 0.0F;
static float32_t Ts = control_task_period * 1e-6;
static PidParams pid_params(Ts, kp, Ti, Td, N, lower_bound, upper_bound);

static Pid pid1;
static Pid pid2;

#ifdef CONFIG_SHIELD_OWNVERTER
/* [bool] state of the PWM (ctrl task) */
static bool pwm_enable_leg_3 = false;
float32_t V3_low_value;
float32_t I3_low_value;
float32_t T3_value;

float32_t delta_V3;
float32_t V3_max = 0.0;
float32_t V3_min = 0.0;
static Pid pid3;
#endif

static uint32_t counter = 0;
static uint32_t temp_meas_internal = 10;

static float32_t local_analog_value=0;

/* ---------------SETUP FUNCTIONS---------------------------------- */

void setup_routine()
{

#ifdef CONFIG_SHIELD_OWNVERTER
    shield.sensors.enableDefaultOwnverterSensors();
#endif

#ifdef CONFIG_SHIELD_TWIST
    shield.sensors.enableDefaultTwistSensors();
#endif

    shield.power.initBuck(LEG1);
    shield.power.initBuck(LEG2);

#ifdef CONFIG_SHIELD_OWNVERTER
    shield.power.initBuck(LEG3);
#endif

    AppTask_num = task.createBackground(loop_application_task);
    CommTask_num = task.createBackground(loop_communication_task);
    task.createCritical(&loop_control_task, control_task_period);

    pid1.init(pid_params);
    pid2.init(pid_params);
#ifdef CONFIG_SHIELD_OWNVERTER
    pid3.init(pid_params);
#endif

    task.startBackground(AppTask_num);
    task.startBackground(CommTask_num);
    task.startCritical();

}

/* ---------------LOOP FUNCTIONS---------------------------------- */

void loop_communication_task()
{
    received_char = console_getchar();
    initial_handle(received_char);
}

void loop_application_task()
{
    switch(mode)
    {
        case IDLE:
            /* IDLE MODE - turns data emission off */
            spin.led.turnOff();
            if(!print_done) {
                printk("IDLE \n");
                print_done = true;
            }
            break;
        case POWER_OFF:
            /* POWER_OFF MODE - turns the power off but broadcasts
             * the system state data */
            spin.led.toggle();
            if(!print_done) {
                printk("POWER OFF \n");
                print_done = true;
            }
            frame_POWER_OFF();
            break;
        case POWER_ON:
            /* POWER_ON MODE - turns the system on and broadcasts
             * measurement from the physical variables */
            spin.led.turnOn();
            if(!print_done) {
                printk("POWER ON \n");
                print_done = true;
            }

#ifdef CONFIG_SHIELD_OWNVERTER
            meas_data = shield.sensors.getLatestValue(TEMP_SENSOR);

            counter++;
            if(counter == temp_meas_internal){
                shield.sensors.setOwnverterTempMeas(TEMP_1);
                if (meas_data != NO_VALUE) T3_value = meas_data;
            } else if(counter == 2*temp_meas_internal){
                shield.sensors.setOwnverterTempMeas(TEMP_2);
                if (meas_data != NO_VALUE) T1_value = meas_data;
            } else if(counter == 3*temp_meas_internal){
                shield.sensors.setOwnverterTempMeas(TEMP_3);
                if (meas_data != NO_VALUE) T2_value = meas_data;
                counter = 0;
            }
#endif

#ifdef CONFIG_SHIELD_TWIST

            counter++;
            if(counter == temp_meas_internal){
                shield.sensors.triggerTwistTempMeas(TEMP_SENSOR_1);
                meas_data = shield.sensors.getLatestValue(TEMP_SENSOR_2);
                if (meas_data != NO_VALUE) T2_value = meas_data;
            } else if(counter == 2*temp_meas_internal){
                shield.sensors.triggerTwistTempMeas(TEMP_SENSOR_2);
                meas_data = shield.sensors.getLatestValue(TEMP_SENSOR_1);
                if (meas_data != NO_VALUE) T1_value = meas_data;
                counter = 0;
            }
#endif
            frame_POWER_ON();
            break;
        default:
            break;
    }

     task.suspendBackgroundMs(100);
}


void loop_control_task()
{
    /* ------------- GET SENSOR MEASUREMENTS --------------------- */
    meas_data = shield.sensors.getLatestValue(V1_LOW);
    if (meas_data != NO_VALUE)
        V1_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(V2_LOW);
    if (meas_data != NO_VALUE)
        V2_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(V_HIGH);
    if (meas_data != NO_VALUE)
        V_high_value = meas_data;

    meas_data = shield.sensors.getLatestValue(I1_LOW);
    if (meas_data != NO_VALUE)
        I1_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(I2_LOW);
    if (meas_data != NO_VALUE)
        I2_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(I_HIGH);
    if (meas_data != NO_VALUE)
        I_high_value = meas_data;

#ifdef CONFIG_SHIELD_OWNVERTER
    meas_data = shield.sensors.getLatestValue(V3_LOW);
    if (meas_data != NO_VALUE)
        V3_low_value = meas_data;

    meas_data = shield.sensors.getLatestValue(I3_LOW);
    if (meas_data != NO_VALUE)
        I3_low_value = meas_data;
#endif


    /* ----------- DEPLOYS MODES---------------- */
    switch(mode){
        /* IDLE and POWER_OFF modes turn the power off */
        case IDLE:
        case POWER_OFF:
            shield.power.stop(LEG1);
            pwm_enable_leg_1 = false;
            V1_max  = 0;

            shield.power.stop(LEG2);
            pwm_enable_leg_2 = false;
            V2_max  = 0;

#ifdef CONFIG_SHIELD_OWNVERTER
            shield.power.stop(LEG3);
            pwm_enable_leg_3 = false;
            V3_max  = 0;
#endif
            break;

        case POWER_ON:
            /* POWER_ON mode turns the power ON */

            /* Tests if the legs were turned off and does it only once */
            if(!pwm_enable_leg_1 &&
                power_leg_settings[LEG1].settings[BOOL_LEG])
            {
                shield.power.start(LEG1);
                pwm_enable_leg_1 = true;
            }
            if(!pwm_enable_leg_2 &&
                power_leg_settings[LEG2].settings[BOOL_LEG])
            {
                shield.power.start(LEG2);
                pwm_enable_leg_2 = true;
            }

#ifdef CONFIG_SHIELD_OWNVERTER
            if(!pwm_enable_leg_3
                && power_leg_settings[LEG3].settings[BOOL_LEG])
            {
                shield.power.start(LEG3);
                pwm_enable_leg_3 = true;
            }
#endif
            /* Tests if the legs were turned on and does it only once */
            if(pwm_enable_leg_1 &&
               !power_leg_settings[LEG1].settings[BOOL_LEG])
            {
                shield.power.stop(LEG1);
                pwm_enable_leg_1 = false;
            }

            if(pwm_enable_leg_2 &&
               !power_leg_settings[LEG2].settings[BOOL_LEG])
            {
                shield.power.stop(LEG2);
                pwm_enable_leg_2 = false;
            }

#ifdef CONFIG_SHIELD_OWNVERTER
            if(pwm_enable_leg_3 &&
               !power_leg_settings[LEG3].settings[BOOL_LEG])
            {
                shield.power.stop(LEG3);
                pwm_enable_leg_3 = false;
            }
#endif

            /* Calls the pid calculation if the converter in either
             * in mode buck or boost for a given dynamically
             * set reference value */

            if(power_leg_settings[LEG1].settings[BOOL_BUCK] ||
               power_leg_settings[LEG1].settings[BOOL_BOOST])
            {
                power_leg_settings[LEG1].duty_cycle =
                    pid1.calculateWithReturn(
                        power_leg_settings[LEG1].reference_value,
                        *power_leg_settings[LEG1].tracking_variable
                    );
            }

            if(power_leg_settings[LEG2].settings[BOOL_BUCK] ||
               power_leg_settings[LEG2].settings[BOOL_BOOST])
            {
                power_leg_settings[LEG2].duty_cycle =
                    pid2.calculateWithReturn(
                        power_leg_settings[LEG2].reference_value ,
                        *power_leg_settings[LEG2].tracking_variable
                    );
            }

#ifdef CONFIG_SHIELD_OWNVERTER
            if(power_leg_settings[LEG3].settings[BOOL_BUCK] ||
               power_leg_settings[LEG3].settings[BOOL_BOOST])
            {
            power_leg_settings[LEG3].duty_cycle =
                pid3.calculateWithReturn(
                    power_leg_settings[LEG3].reference_value ,
                    *power_leg_settings[LEG3].tracking_variable
                );
            }
#endif

            if(power_leg_settings[LEG1].settings[BOOL_LEG])
            {
                if(power_leg_settings[LEG1].settings[BOOL_BOOST])
                {
                    /* Inverses the convention of the leg in case
                     * of changing from buck to boost */
                    shield.power.setDutyCycle(
                        LEG1,
                        (1-power_leg_settings[LEG1].duty_cycle)
                    );
                }
                else
                {
                    /* Uses the normal convention by default */
                    shield.power.setDutyCycle(
                        LEG1,
                        power_leg_settings[LEG1].duty_cycle
                    );
                }
            }

            if(power_leg_settings[LEG2].settings[BOOL_LEG])
            {
                if(power_leg_settings[LEG2].settings[BOOL_BOOST])
                {
                    /* Inverses the convention of the leg in case
                     * of changing from buck to boost */
                    shield.power.setDutyCycle(
                        LEG2,
                        (1-power_leg_settings[LEG2].duty_cycle)
                    );
                }
                else
                {
                    /* Uses the normal convention by default */
                    shield.power.setDutyCycle(
                        LEG2,
                        power_leg_settings[LEG2].duty_cycle
                    );
                }
            }

#ifdef CONFIG_SHIELD_OWNVERTER
            if(power_leg_settings[LEG3].settings[BOOL_LEG])
            {
                if(power_leg_settings[LEG3].settings[BOOL_BOOST])
                {
                    /* Inverses the convention of the leg in case of
                     * changing from buck to boost */
                    shield.power.setDutyCycle(
                        LEG3,
                        (1-power_leg_settings[LEG3].duty_cycle)
                    );
                }
                else
                {
                    /* Uses the normal convention by default */
                    shield.power.setDutyCycle(
                        LEG3,
                        power_leg_settings[LEG3].duty_cycle
                    );
                }
            }
#endif

            if(V1_low_value > V1_max){
                /* Gets the maximum V1 voltage value.
                 * This is used for the capacitor test */
                V1_max = V1_low_value;
            }
            if(V2_low_value > V2_max){
                /* Gets the maximum V2 voltage value.
                 * This is used for the capacitor test */
                V2_max = V2_low_value;
            }

#ifdef CONFIG_SHIELD_OWNVERTER
            if(V3_low_value > V3_max){
                /* Gets the maximum V3 voltage value.
                 * This is used for the capacitor test */
                V3_max = V3_low_value;
            }
#endif

            break;
        default:
            break;
    }
}

/**
 * This is the main function of this example
 * This function is generic and does not need editing.
 */

int main(void)
{
    setup_routine();

    return 0;
}