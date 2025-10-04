/*
 * Copyright (c) 2023 Juan Manuel Cruz <jcruz@fi.uba.ar> <jcruz@frba.utn.edu.ar>.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @file   : task_c.c
 * @date   : Set 26, 2023
 * @author : Juan Manuel Cruz <jcruz@fi.uba.ar> <jcruz@frba.utn.edu.ar>
 * @version	v1.0.0
 */

/********************** inclusions *******************************************/
/* Project includes */
#include "main.h"

/* Demo includes */
#include "logger.h"
#include "dwt.h"

/* Application & Tasks includes */
#include "board.h"
#include "app.h"

#include <float.h>
#include <math.h>

/********************** macros and definitions *******************************/


/********************** internal data declaration ****************************/

/********************** internal functions declaration ***********************/
float calcular_pid(bool ojos[]);
float calcular_error(bool ojos[]);
float calcular_dt(void);

/********************** internal data definition *****************************/
const char *p_task_c 		= "Task Cerebro";
static float acum_integral_error = 0; /* acumulo la integral del error */
static float prev_error = 0; /* guardo el último error */
static float prev_t;
static uint16_t contador_impresion = 0;
static float prev_pid = 0;
/********************** external data declaration ****************************/


/********************** external functions definition ************************/
void task_cerebro_init(void *parameters) {
	/* Print out: Task Initialized */
	LOGGER_INFO(" ");
	LOGGER_INFO("  %s is running - %s", GET_NAME(task_c_init), p_task_c);

	shared_data_type * shared_data = (shared_data_type *) parameters;

	shared_data->estado_piernas = STATE_STOP;
	shared_data->actualizar_piernas = false;

    prev_t = cycle_counter_get_time_us();
}

void task_cerebro_update(void *parameters) {
	shared_data_type * shared_data = (shared_data_type *) parameters;

	bool * ojos = shared_data->ojos;
	piernas_state_t * estado_piernas = &shared_data->estado_piernas;
	bool * actualizar_piernas = &shared_data->actualizar_piernas;

	float valor_pid = calcular_pid(ojos);
/*
	if (valor_pid != prev_pid) {
		int32_t pid_entero = (int32_t) valor_pid;
		int32_t pid_decimal = (int32_t) ((valor_pid - pid_entero) * 100);
		LOGGER_INFO("cambio de PID = %li.%li", pid_entero, pid_decimal);
	}
*/
	prev_pid = valor_pid;
/*
	if (contador_impresion == 200) {
		int32_t pid_entero = (int32_t) valor_pid;
		int32_t pid_decimal = (int32_t) ((valor_pid - pid_entero) * 100);
		LOGGER_INFO("PID = %li.%li", pid_entero, pid_decimal);

		contador_impresion = 0;
	}
	contador_impresion++;
*/
	if (valor_pid < 0) {
		*estado_piernas = STATE_TURN_LEFT;
		*actualizar_piernas = true;
	} else if (valor_pid == 0) {
		*estado_piernas = STATE_FORWARD;
		*actualizar_piernas = true;
	} else if (valor_pid > 0) {
		*estado_piernas = STATE_TURN_RIGHT;
		*actualizar_piernas = true;
	}
}

float calcular_pid(bool ojos[]) {
    const float k_prop = 1,
          k_inte = 0.1,
          k_dif = 0.1;

    float dt = calcular_dt();

    float error = calcular_error(ojos);

    /* Calculo la parte proporcional */
	float proporcional = k_prop * error;

    /* Calculo la parte integral */
    acum_integral_error += error * dt;
    /* Me aseguro de que el número no explote */
    acum_integral_error = fmin(acum_integral_error, 1e3);
    acum_integral_error = fmax(acum_integral_error, -1e3);
	float integral = k_inte * acum_integral_error;

    /* Calculo la parte diferencial */
    float de_dt = (error - prev_error) / dt;
	float diferencial = k_dif * de_dt;

    prev_error = error;

    return proporcional + integral + diferencial;
}

float calcular_error(bool ojos[]) {
    /* Tomo 0 como setpoint, valores negativos son izquierda y positivos
     * derecha */

    float error = 0;

	float sensores[] = {
			-2 * ojos[0],
			-1 * ojos[1],
			 1 * ojos[2],
			 2 * ojos[3]
	};
	uint8_t tam_sensores = sizeof(sensores) / sizeof(*sensores);

    /* Calculo el error como el promedio del valor de todos los sensores.
     * Si hay sensores contiguos activos, se promedian. Asumo que  no se van a
     * activar sensores que no sean contiguos pero no importa. */
    uint8_t sensores_activados = 0;
    for (uint8_t i = 0; i < tam_sensores; i++) {
        if (!sensores[i])
            continue;

        error += sensores[i];
        sensores_activados++;
    }

    /* si no se encontraron sensores activos, devuelvo el error de la última
     * lectura */
    if (!sensores_activados)
        return prev_error;

    return error/sensores_activados;
}

float calcular_dt(void) {
    float actual_t = cycle_counter_get_time_us();

    float max_t = UINT32_MAX / (SystemCoreClock * 1e-6);
    if (actual_t < prev_t) /* hubo overflow */
            actual_t += max_t;

    float dt = (actual_t - prev_t) * 1e6;
    dt = fabs(dt) + FLT_MIN;
    prev_t = remainder(actual_t, max_t);

    return dt;
}

/********************** end of file ******************************************/
