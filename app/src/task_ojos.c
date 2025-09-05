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
 * @file   : task_adc.c
 * @date   : Set 26, 2023
 * @author : Juan Manuel Cruz <jcruz@fi.uba.ar> <jcruz@frba.utn.edu.ar>
 * @version	v1.0.0
 */

/********************** inclusions *******************************************/
/* Project includes. */
#include "main.h"

/* Demo includes. */
#include "logger.h"
#include "dwt.h"

/* Application & Tasks includes. */
#include "board.h"
#include "app.h"
#include "task_ojos.h"

/********************** macros and definitions *******************************/


/********************** internal data declaration ****************************/
typedef struct {
	GPIO_TypeDef * puerto;
	uint16_t pin;
} ojo_t;

/********************** internal functions declaration ***********************/


/********************** internal data definition *****************************/
const char *p_task_ojos	= "Leer ojos";
const ojo_t ojos[] = {
		{.puerto = ojo_izq_ext_GPIO_Port, .pin = ojo_izq_ext_Pin},
		{.puerto = ojo_izq_int_GPIO_Port, .pin = ojo_izq_int_Pin},
		{.puerto = ojo_der_int_GPIO_Port, .pin = ojo_der_int_Pin},
		{.puerto = ojo_der_ext_GPIO_Port, .pin = ojo_der_ext_Pin}
};

#define OJOS_QTY (sizeof(ojos) / sizeof(ojo_t))


/********************** external data declaration *****************************/

/********************** external functions definition ************************/
void task_ojos_init(void *parameters)
{
	shared_data_type *shared_data = (shared_data_type *) parameters;

	for (size_t i = 0; i < OJOS_MAX; i++)
		shared_data->ojos[i] = false;

	/* Print out: Task Initialized */
	LOGGER_LOG("  %s is running - %s\r\n", GET_NAME(task_adc_init), p_task_ojos);

}

void task_ojos_update(void *parameters)
{

	shared_data_type *shared_data = (shared_data_type *) parameters;

	for (int i = 0; i < OJOS_QTY; i++) {
		bool readed = (bool) HAL_GPIO_ReadPin(ojos[i].puerto, ojos[i].pin);
		shared_data->ojos[i] = readed;
	}

}






/********************** end of file ******************************************/
