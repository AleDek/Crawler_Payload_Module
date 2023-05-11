/*
 * mbed library program
 *  D6T_8L_09: D6T MEMS Thermal Sensors
 *  --- High Sensitivity Enables Detection of Stationary Human Presence
 *  https://www.components.omron.com/web/en/product-detail?partNumber=D6T
 *
 * Copyright (c) 2020 Kenji Arai / JH1PJL
 *  http://www7b.biglobe.ne.jp/~kenjia/
 *  https://os.mbed.com/users/kenjiArai/
 *      Created:    February  16th, 2020
 *      Revised:    February  18th, 2020
 */

/*
    Refrence documentation:
    https://omronfs.omron.com/ja_JP/ecb/products/pdf/CDSC-019.pdf

    Tested on:
        GR-MANGO(beta version) with 2JCIE-EV01 board
 */

#include "D6T_8L_09.h"

#define DEBUG_PRINT             0

// Data structure
#define NUMBER_OF_ARRAY         8 
// chip temp(2)+16position(2*16)+PEC/CRC(2)+Termination(1)
#define NUMBER_OF_DATA          19
// chip temperature data position
#define POSITION_CHIP_TEMP      2
// CRC data position
#define POSITION_PEC            18

D6T_8L_09::D6T_8L_09(PinName p_sda, PinName p_scl) :
  _i2c_p(new I2C(p_sda, p_scl)), _i2c(*_i2c_p)
{
    initialize();
}

D6T_8L_09::D6T_8L_09 (I2C& p_i2c) : _i2c(p_i2c)
{
    initialize();
}

void D6T_8L_09::initialize()
{
    _i2c.frequency(100000);     // Set I2C clock frequency
    _addr = D6T_8L_09_ADDR;    // set address
    _chip_temp = -100.0f;
}

bool D6T_8L_09::read_float_data(float *dt)
{
    float *p = dt;
    bool result = read_data();
    for(uint32_t i = 0; i < NUMBER_OF_ARRAY; i++) {
        *p++ = (float)_arry[i] / 10.0f;
    }
    return result;
}

bool D6T_8L_09::read_16bit_data(int16_t *dt)
{
    int16_t *p = dt;
    bool result = read_data();
    for(uint32_t i = 0; i < NUMBER_OF_ARRAY; i++) {
        *p++ = _arry[i];
    }
    return result;
}

bool D6T_8L_09::read_raw_buffer(char *dt){

    char *p = dt;
    bool result = read_data();
    for(uint32_t i = 0; i < NUMBER_OF_DATA; i++) {
        *p++ = _dt[i];
    }
    return result;

}

bool D6T_8L_09::get_raw_buffer(char *dt){

    char *p = dt;
    bool result = 1;
    for(uint32_t i = 0; i < NUMBER_OF_DATA; i++) {
        *p++ = _dt[i];
    }
    return result;

}

float D6T_8L_09::read_chip_temp()
{
    if (_chip_temp == -100.0f) {
        read_data();
    }
    return _chip_temp;
}

bool D6T_8L_09::read_data()
{
    _dt[0] = D6T_8L_09_CMD;
    _i2c.write(_addr, _dt, 1, true);
    _i2c.read(_addr, _dt, NUMBER_OF_DATA, false);
    bool result = check_crc();
#if DEBUG_PRINT
    printf("CIAO STRONZO\r\n");
    for (uint32_t i = 0; i < NUMBER_OF_DATA; i++) {
        printf("%3d, %6d\r\n", i, _dt[i]);
    }
    if (result == false) {
        printf("CRC error\r\n");
    } else {
        printf("CRC okay\r\n");   
    }
#endif
    _chip_temp = (float)((_dt[1] << 8U) | _dt[0]) / 10.0f;
    for (uint32_t i = 2, n = 0; n < NUMBER_OF_ARRAY; i += 2, n++) {
        _arry[n] = (int16_t)((_dt[i+1] << 8U) | _dt[i]);
    }
    return result;
}

uint8_t D6T_8L_09::calc_crc(uint8_t dt)
{
    for(uint32_t i = 0; i < 8; i++) {
        uint8_t temp = dt;
        dt <<= 1U;
        if (temp & 0x80) {
            dt ^= 0x07;
        }
    }
    return dt;
}

bool D6T_8L_09::check_crc()
{
    uint8_t crc = calc_crc(D6T_8L_09_ADDR + 0U);   // +0 = write
    crc = calc_crc(D6T_8L_09_CMD ^ crc );
    crc = calc_crc((D6T_8L_09_ADDR + 1U) ^ crc );  // +1 = Read
    for(uint32_t i = 0; i < POSITION_PEC; i++) {
        crc = calc_crc( _dt[i] ^ crc );
    }
    if (crc == _dt[POSITION_PEC]) {
        return true;
    } else {
        return false;
    }
}
