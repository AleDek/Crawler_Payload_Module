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
/* File WAS generated by gen_cfile.py. Should not be modified. */

#ifndef MBED_SLAVE_H
#define MBED_SLAVE_H

#include "data.h"

/* Prototypes of function provided by object dictionnary */
UNS32 mbed_slave_valueRangeTest (UNS8 typeValue, void * value);
const indextable * mbed_slave_scanIndexOD (UNS16 wIndex, UNS32 * errorCode, ODCallback_t **callbacks);

/* Master node data struct */
extern CO_Data mbed_slave_Data;
extern UNS32 Probe_height_32_bit[1];	/* Mapped at index 0x6202, subindex 0x01 - 0x01 */
extern UNS32 Brush_speed_32_bit[1];	    /* Mapped at index 0x6202, subindex 0x01 - 0x02 */
extern UNS32 Brush_height_32_bit[1];	/* Mapped at index 0x6202, subindex 0x02 - 0x03 */
extern UNS8 Led_state_8_Bit[1];		    /* Mapped at index 0x6207, subindex 0x01 - 0x01 */
extern UNS8 Laser_state_8_Bit[1];		/* Mapped at index 0x6207, subindex 0x01 - 0x02 */
extern UNS8 Fan_state_8_Bit[1];		    /* Mapped at index 0x6207, subindex 0x01 - 0x03 */
extern UNS8 Thermal_enable_8_Bit[1];	/* Mapped at index 0x6207, subindex 0x01 - 0x04 */
// extern UNS16 Tof_1_16_bit[1];		    /* Mapped at index 0x6207, subindex 0x01 - 0x01 */
// extern UNS16 Tof_2_16_bit[1];		    /* Mapped at index 0x6207, subindex 0x02 - 0x02 */
// extern UNS16 Tof_3_16_bit[1];		    /* Mapped at index 0x6207, subindex 0x03 - 0x03 */
// extern UNS16 Tof_4_16_bit[1];		    /* Mapped at index 0x6207, subindex 0x04 - 0x04 */
// extern UNS32 Force_deadzone_32_bit[1];  /* Mapped at index 0x6202, subindex 0x01 - 0x01 */
// extern UNS32 Force_Kp_32_bit[1];        /* Mapped at index 0x6202, subindex 0x02 - 0x02 */
// extern UNS32 Force_Ki_32_bit[1];        /* Mapped at index 0x6202, subindex 0x03 - 0x03 */
// extern UNS32 Force_Kd_32_bit[1];        /* Mapped at index 0x6202, subindex 0x04 - 0x04 */

/* // old stuff from the example of the library
extern UNS8 Read_Inputs_8_Bit[1];		
extern UNS8 Polarity_Input_8_Bit[1];		
extern UNS8 Filter_Constant_Input_8_Bit[1];		
extern UNS8 Global_Interrupt_Enable_Digital;		
extern UNS8 Interrupt_Mask_Any_Change_8_Bit[1];		
extern UNS8 Interrupt_Mask_Low_to_High_8_Bit[1];		
extern UNS8 Interrupt_Mask_High_to_Low_8_Bit[1];		
extern UNS8 Write_Outputs_8_Bit[1];		
extern UNS8 Change_Polarity_Outputs_8_Bit[1];		
extern UNS8 Error_Mode_Outputs_8_Bit[1];		
extern UNS8 Error_Value_Outputs_8_Bit[1];		
*/
#endif // MBED_SLAVE_H