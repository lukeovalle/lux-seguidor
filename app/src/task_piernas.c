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
 * @file   : task_b.c
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

/********************** macros and definitions *******************************/
#define G_TASK_B_CNT_INI	0ul

#define TASK_B_CNT_INI		0ul
#define TASK_B_CNT_MAX		50ul

#define TASK_B_DEL_INI		0ul
#define TASK_B_DEL_MAX		500ul

/********************** internal data declaration ****************************/
typedef struct {
	GPIO_TypeDef * puerto1, * puerto2;
	uint16_t pin1, pin2;
} motor_t;

typedef enum { LEFT = 0, RIGHT = 1 } direction_t;

/********************** internal functions declaration ***********************/
void pierna_girar_reloj(direction_t dir);
void pierna_girar_contrareloj(direction_t dir);
void pierna_parar(direction_t dir);

/********************** internal data definition *****************************/
const char *p_task_b 		= "Task B - Non-Blocking Code";
motor_t piernas[] = {
		{.puerto1 = motor1_1_GPIO_Port, .puerto2 = motor1_2_GPIO_Port,
				.pin1 = motor1_1_Pin, .pin2 = motor1_2_Pin},
		{.puerto1 = motor2_1_GPIO_Port, .puerto2 = motor2_2_GPIO_Port,
				.pin1 = motor2_1_Pin, .pin2 = motor2_2_Pin}
};

#define PIERNAS_QTY (sizeof(piernas) / sizeof(motor_t))

/********************** external data declaration ****************************/
uint32_t g_task_b_cnt;

/********************** external functions definition ************************/
void task_piernas_init(void *parameters)
{
	/* Print out: Task Initialized */
	LOGGER_INFO(" ");
	LOGGER_INFO("  %s is running - %s", GET_NAME(task_b_init), p_task_b);

	/* Init & Print out: Task execution counter */
	g_task_b_cnt = G_TASK_B_CNT_INI;
	LOGGER_INFO("   %s = %lu", GET_NAME(g_task_b_cnt), g_task_b_cnt);

	shared_data_type * parametros = (shared_data_type *) parameters;

	parametros->actualizar_piernas = true;
	parametros->estado_piernas = STATE_FORWARD;

}

void task_piernas_update(void *parameters)
{
	shared_data_type * parametros = (shared_data_type *) parameters;

	if (!parametros->actualizar_piernas)
		return;

	parametros->actualizar_piernas = false;

	switch (parametros->estado_piernas) {
		case STATE_FORWARD:
			pierna_girar_contrareloj(LEFT);
			pierna_girar_contrareloj(RIGHT);
			break;
		case STATE_BACKWARDS:
			pierna_girar_reloj(LEFT);
			pierna_girar_reloj(RIGHT);
			break;
		case STATE_TURN_LEFT:
			pierna_parar(LEFT);
			pierna_girar_contrareloj(RIGHT);
			break;
		case STATE_TURN_RIGHT:
			pierna_girar_contrareloj(LEFT);
			pierna_parar(RIGHT);
			break;
		case STATE_STOP:
			pierna_parar(LEFT);
			pierna_parar(RIGHT);
			break;
		default:
			break;
	}
}

void pierna_girar_reloj(direction_t dir) {
	HAL_GPIO_WritePin(piernas[dir].puerto1, piernas[dir].pin1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(piernas[dir].puerto2, piernas[dir].pin2, GPIO_PIN_SET);
}

void pierna_girar_contrareloj(direction_t dir) {
	HAL_GPIO_WritePin(piernas[dir].puerto1, piernas[dir].pin1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(piernas[dir].puerto2, piernas[dir].pin2, GPIO_PIN_RESET);
}

void pierna_parar(direction_t dir) {
	HAL_GPIO_WritePin(piernas[dir].puerto1, piernas[dir].pin1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(piernas[dir].puerto2, piernas[dir].pin2, GPIO_PIN_RESET);
}

/********************** end of file ******************************************/
