/**************************************************************
     Koala crowler periperials handlers on STM32F303K8 
**************************************************************/
/*

Copyright (C): Alessandro De Crescenzo - ale-dc@live.it

See COPYING file for copyrights details.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#ifndef __KOALA_PERIPH_H__
#define __KOALA_PERIPH_H__

// Includes for the Canfestival
#include "canfestival.h"
#include "canopen_timer.h"
#include "koala_OD.h"
#include "D6T_8L_09.h"
#include "mbed.h"

void init_uart(void);

void init_servos(void);
void init_outputs(void);
void init_thermal(void);

void thermal_measurement_fun(void);



#ifdef __cplusplus
extern "C" {
#endif


unsigned char servos_handler(CO_Data* d);
unsigned char outputs_handler(CO_Data* d);
unsigned char thermal_handler(CO_Data* d);  //TODO remove , basta vedere in measurement fun la variabile

#ifdef __cplusplus
}
#endif

#endif //__KOALA_PERIPH_H__
