/**************************************************************
     Koala crowler periperials handlers on STM32L432KC 
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

#include "koalaPeriph.h"
#include "DigitalOut.h"
#include <cstdint>

/* Servos*/
#define PROBE_LIFT_SERVO_PIN PA_6
#define BRUSH_LIFT_SERVO_PIN PA_1 //conflitto uart2
#define BRUSH_SERVO_PIN      PA_3 //conflitto uart2 ?
/* Digital outputs */
#define LIGHT_PIN PB_1
#define LASER_PIN PB_0 // TODO verificare !
#define FAN_PIN   PA_8
/* thermal sensor*/
#define THERM_1_SDA PB_7
#define THERM_1_SCL PB_6
#define THERM_2_SDA PB_4
#define THERM_2_SCL PA_7

// make the system speed available for timer configuration
extern uint32_t SystemCoreClock;

//servos homing values

const float probe_lift_home =0.0; //pu
const float probe_lift_up = 1000;
const float probe_lift_dn = 2000;

const float brush_lift_home =0.0; //pu
const float brush_lift_up = 1000;
const float brush_lift_dn = 2000;

const float brush_home = 0.0; //pu
const float brush_max = 1000;
const float brush_min = 2000;

// only for debug TODO remove
float old_probe_hgt_sp ;
float old_brush_hgt_sp ;
float old_brush_spd_sp ;
uint8_t old_led_sp ;
uint8_t old_laser_sp ;
uint8_t old_fan_sp ;
uint8_t old_thermal_ena;

/* hw objects*/
Serial main_pc(USBTX, USBRX);

PwmOut servo_probe_lift(PROBE_LIFT_SERVO_PIN);
PwmOut servo_brush_lift(BRUSH_LIFT_SERVO_PIN);
PwmOut servo_brush(BRUSH_SERVO_PIN);

DigitalOut light(LIGHT_PIN);
DigitalOut laser(LASER_PIN);
DigitalOut fan(FAN_PIN);

uint8_t therm_buffer[19];
uint8_t serial_buffer[21];
uint8_t j = 0;      //fake buffer TODO remove

/*utility*/
float clamp(float & x , float min, float max){
    if(x > max ) x = max;
    else if (x <min ) x= min;
    return x;
}
float map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
/*debug*/
void printSYSCLK(void){
    //printf("system clock: %d \n",SystemCoreClock);
}
	
/*
*  PERIPHERIAL FUNCTIONS 
*/ 
void init_servos(void){
	Probe_height_32_bit[0] = *((uint32_t*)&probe_lift_home);
    Brush_height_32_bit[0] = *((uint32_t*)&brush_lift_home);
    Brush_speed_32_bit[0] = *((uint32_t*)&brush_home);

    old_probe_hgt_sp = 0.0;
    old_brush_hgt_sp = 0.0;
    old_brush_spd_sp = 0.0;

    servo_probe_lift.pulsewidth_us((int)map(probe_lift_home, 0.0, 1.0, probe_lift_up, probe_lift_dn));
    servo_brush_lift.pulsewidth_us((int)map(brush_lift_home, 0.0, 1.0, brush_lift_up, brush_lift_dn));
    servo_brush.pulsewidth_us((int)map(brush_home, -1.0, 1.0, brush_min, brush_max));
}
void init_outputs(void){
    old_led_sp =0;
    old_laser_sp =0;
    old_fan_sp =0;
    old_thermal_ena =0;

    light = 0;
    laser = 0;
    fan = 0;
}
void init_thermal(void){
    main_pc.baud(115200);

    for(int i =0;i<19;i++){
        therm_buffer[i] = i+j;
    }
}

void thermal_measurement_fun(void){
    if(Thermal_enable_8_Bit[0]){
        printf("[");
        for(int i =0;i<19;i++){
            therm_buffer[i] = i+j;
            printf("%u ", therm_buffer[i]);
        }
        printf("]\n");
        j++;
        j = j%19;
        serial_buffer[0] = 0xFF;
        serial_buffer[1] = 0xFF;
        for(int i =0; i<19;i++) serial_buffer[2+i] =therm_buffer[i];
        main_pc.write(serial_buffer,21,(const event_callback_t)NULL);
        //printf("Thermal data demo [  ] \n");
    }
}



unsigned char servos_handler(CO_Data* d){
    //UNS32 pos = Servo_position_32_bit[0];
	float probe_hgt_sp = *((float*)&Probe_height_32_bit[0]);
    float brush_hgt_sp = *((float*)&Brush_height_32_bit[0]);
    float brush_spd_sp = *((float*)&Brush_speed_32_bit[0]);
    
    if(probe_hgt_sp != old_probe_hgt_sp) printf("Probe lift= %f\n",probe_hgt_sp); //TODO remove
    if(brush_hgt_sp != old_brush_hgt_sp) printf("Brush lift= %f\n",brush_hgt_sp); //TODO remove
    if(brush_spd_sp != old_brush_spd_sp) printf("Brush speed= %f\n",brush_spd_sp); //TODO remove
    
    old_probe_hgt_sp = probe_hgt_sp;  //TODO remove
    old_brush_hgt_sp = brush_hgt_sp;
    old_brush_spd_sp = brush_spd_sp; 

    clamp(probe_hgt_sp, 0.0, 1.0);
    clamp(brush_hgt_sp, 0.0, 1.0);
    clamp(brush_spd_sp, -1.0, 1.0);
    servo_probe_lift.pulsewidth_us((int)map(probe_hgt_sp, 0.0, 1.0, probe_lift_up, probe_lift_dn));
    servo_brush_lift.pulsewidth_us((int)map(brush_hgt_sp, 0.0, 1.0, brush_lift_up, brush_lift_dn));
    servo_brush.pulsewidth_us((int)map(brush_spd_sp, -1.0, 1.0, brush_min, brush_max));

    return 1;
}
unsigned char outputs_handler(CO_Data* d){
    uint8_t led_sp = Led_state_8_Bit[0];
    uint8_t laser_sp = Laser_state_8_Bit[0];
    uint8_t fan_sp = Fan_state_8_Bit[0];

    if(old_led_sp != led_sp) printf("Light= %u\n",led_sp); //TODO remove
    if(old_laser_sp != laser_sp) printf("Laser= %u\n",laser_sp); //TODO remove
    if(old_fan_sp != fan_sp) printf("Fan= %u\n",fan_sp); //TODO remove
    old_led_sp = led_sp;   //TODO remove
    old_laser_sp = laser_sp;
    old_fan_sp = fan_sp;

    light = bool(led_sp);
    laser = bool(laser_sp);
    fan = bool(fan_sp);

    return 1;
}
unsigned char thermal_handler(CO_Data* d){     //TODO remove , basta vedere in measurement_fun la variabile
    uint8_t thermal_ena = Thermal_enable_8_Bit[0];

    if(old_thermal_ena != thermal_ena){ //TODO remove
        if(thermal_ena) printf("Thermal=  enabled\n");
        else printf("Thermal=  disabled\n");
    }
    old_thermal_ena = thermal_ena; //TODO remove
    return 1;
}


