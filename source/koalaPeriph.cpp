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

#define DEBUG_PRINT 0
/* Servos*/
#define PROBE_LIFT_SERVO_PIN PA_6
#define BRUSH_LIFT_SERVO_PIN PA_3 //conflitto uart2
#define BRUSH_SERVO_PIN      PA_1 //conflitto uart2 ?
/* Digital outputs */
#define LIGHT_PIN PB_1
#define LASER_PIN PB_0 // TODO verificare !
#define FAN_PIN   PA_8
/* thermal sensor*/
#define THERM_1_SDA PB_7 //front thermal
#define THERM_1_SCL PB_6
#define THERM_2_SDA PB_4 //rear thermal
#define THERM_2_SCL PA_7

// make the system speed available for timer configuration
extern uint32_t SystemCoreClock;

//servos homing values

const float probe_lift_home =0.0; //pu
const float probe_lift_up = 1900;
const float probe_lift_dn = 1000;

const float brush_lift_home =0.0; //pu
const float brush_lift_up = 2000;
const float brush_lift_dn = 1000;

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

//thermal sensors
D6T_8L_09 tmp_fwd(THERM_1_SDA, THERM_1_SCL); // SDA, SCL
//D6T_8L_09 tmp_bwd(THERM_2_SDA, THERM_2_SCL); // SDA, SCL
char raw_buff_fwd[19];
char raw_buff_bwd[19];
uint8_t serial_buffer[21]; //TBD 40 2 +19+19

float floatbuff[8]; //TODO OVE test only
float chiptemp; //TODO OVE test only

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
void init_uart(void){
    main_pc.baud(115200);
}
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
    
    //tmp_fwd.read_chip_temp();
    //tmp_bwd.read_chip_temp();
}

void thermal_measurement_fun(void){
    if(Thermal_enable_8_Bit[0]){
    //// test only
    
        tmp_fwd.read_float_data(floatbuff);
        chiptemp = tmp_fwd.read_chip_temp();
        tmp_fwd.get_raw_buffer(raw_buff_fwd);
        #if DEBUG_PRINT
        printf("ptat: %.1f temp: [",chiptemp);
        for(int i=0;i<8;i++){
            printf("%.1f, ",floatbuff[i]);
        }
        printf("]\n");
        #endif
        
    
        /// final implem
        //tmp_fwd.read_raw_buffer(raw_buff_fwd); //TBD
        //tmp_bwd.read_raw_buffer(raw_buff_bwd); //TBD
    
        serial_buffer[0] = 0xFF;
        serial_buffer[1] = 0xFF;
        for(int i =0; i<19;i++) serial_buffer[2+i] =raw_buff_fwd[i];
        //for(int i =0; i<19;i++) serial_buffer[21+i] =raw_buff_bwd[i]; //TBD
        main_pc.write(serial_buffer,21,(const event_callback_t)NULL);
   
    }
}



unsigned char servos_handler(CO_Data* d){
    //UNS32 pos = Servo_position_32_bit[0];
	float probe_hgt_sp = *((float*)&Probe_height_32_bit[0]);
    float brush_hgt_sp = *((float*)&Brush_height_32_bit[0]);
    float brush_spd_sp = *((float*)&Brush_speed_32_bit[0]);
    
    #if DEBUG_PRINT
        if(probe_hgt_sp != old_probe_hgt_sp) printf("Probe lift= %f\n",probe_hgt_sp); //TODO remove
        if(brush_hgt_sp != old_brush_hgt_sp) printf("Brush lift= %f\n",brush_hgt_sp); //TODO remove
        if(brush_spd_sp != old_brush_spd_sp) printf("Brush speed= %f\n",brush_spd_sp); //TODO remove
    #endif
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
    #if DEBUG_PRINT
        if(old_led_sp != led_sp) printf("Light= %u\n",led_sp); //TODO remove
        if(old_laser_sp != laser_sp) printf("Laser= %u\n",laser_sp); //TODO remove
        if(old_fan_sp != fan_sp) printf("Fan= %u\n",fan_sp); //TODO remove
    #endif
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
    #if DEBUG_PRINT
        if(old_thermal_ena != thermal_ena){ //TODO remove
            if(thermal_ena) printf("Thermal=  enabled\n");
            else printf("Thermal=  disabled\n");
        }
    #endif
    old_thermal_ena = thermal_ena; //TODO remove
    return 1;
}


