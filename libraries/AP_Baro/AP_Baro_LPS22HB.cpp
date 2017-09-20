/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
    driver by Radiolink ljwang
 */
#include "AP_Baro_LPS22HB.h"

#include <utility>
#include <stdio.h>
#include <assert.h>

#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL &hal;

#define ID_WHO_AM_I         0xB1
#define ADDR_THS_P_L			0x0C
#define ADDR_THS_P_H		0x0D
#define ADDR_WHO_AM_I		0x0F
#define ADDR_CTRL_REG1		0x10
#define ADDR_CTRL_REG2		0x11
#define ADDR_CTRL_REG3		0x12
#define ADDR_REF_P_XL			0x15
#define ADDR_REF_P_L				0x16
#define ADDR_REF_P_H			0x17
#define ADDR_RPDS_L				0x18
#define ADDR_RPDS_H				0x19
#define ADDR_INT_CFG			0x0B
#define ADDR_INT_SOURCE		0x25

#define ADDR_STATUS_REG		0x27
#define ADDR_P_OUT_XL		0x28
#define ADDR_P_OUT_L		0x29
#define ADDR_P_OUT_H		0x2A
#define ADDR_TEMP_OUT_L		0x2B
#define ADDR_TEMP_OUT_H		0x2C

#define ADDR_FIFO_CTRL		0x2E
#define ADDR_FIFO_STATUS	0x2F

#define CTRL_REG1_SIM		(1 << 0)
#define CTRL_REG1_BDU		(1 << 1)
#define CTRL_REG1_LPFP_CFG	(1 << 2)
#define CTRL_REG1_EN_LPFP	(1 << 3)
#define CTRL_REG1_PD		(0 << 4)
#define CTRL_REG1_ODR_1HZ	(1 << 4)
#define CTRL_REG1_ODR_10HZ	(2 << 4)
#define CTRL_REG1_ODR_25HZ	(3 << 4)
#define CTRL_REG1_ODR_50HZ	(4 << 4)
#define CTRL_REG1_ODR_75HZ	(5 << 4)

#define CTRL_REG2_ONE_SHOT	0x01
#define CTRL_REG2_SWRESET	(1 << 2)
#define CTRL_REG2_I2C_DIS	(1 << 3)
#define CTRL_REG2_IF_ADD_INC	(1 << 4)
#define CTRL_REG2_STOP_ON_FTH	(1 << 5)
#define CTRL_REG2_FIFO_EN	(1 << 6)
#define CTRL_REG2_BOOT		(1 << 7)

#define CTRL_REG3_INT1_S_DATA	0x0
#define CTRL_REG3_INT1_S_P_HIGH	0x1
#define CTRL_REG3_INT1_S_P_LOW	0x2
#define CTRL_REG3_INT1_S_P_LIM	0x3
#define CTRL_REG3_DRDY		(1 << 2)

#define INTERRUPT_CFG_PH_E	(1 << 0)
#define INTERRUPT_CFG_PL_E	(1 << 1)
#define INTERRUPT_CFG_LIR	(1 << 2)

#define INT_SOURCE_PH		(1 << 0)
#define INT_SOURCE_PL		(1 << 1)
#define INT_SOURCE_IA		(1 << 2)

#define STATUS_REG_T_DA		(1 << 0)
#define STATUS_REG_P_DA		(1 << 1)
#define STATUS_REG_T_OR		(1 << 4)
#define STATUS_REG_P_OR		(1 << 5)

/*
  use an OSR of 1024 to reduce the self-heating effect of the
  sensor. Information from MS tells us that some individual sensors
  are quite sensitive to this effect and that reducing the OSR can
  make a big difference
 */

/*
  constructor
 */
AP_Baro_LPS22HB::AP_Baro_LPS22HB(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev)
		: AP_Baro_Backend(baro)
		, _dev(std::move(dev))
{
}

AP_Baro_Backend *AP_Baro_LPS22HB::probe(AP_Baro &baro,
                                       AP_HAL::OwnPtr<AP_HAL::Device> dev)
{
    if (!dev) {
        return nullptr;
    }
    AP_Baro_LPS22HB *sensor = new AP_Baro_LPS22HB(baro, std::move(dev));
    if (!sensor || !sensor->_init()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

bool AP_Baro_LPS22HB::_init()
{
    if (!_dev) {
        return false;
    }

    _has_sample = false;

    if (!_dev->get_semaphore()->take(0)) {
        AP_HAL::panic("PANIC: AP_Baro_LPS22HB: failed to take serial semaphore for init");
        return false;
    }

    if (_dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI) {
        _dev->set_read_flag(0x80);
   }

    if (_register_read(ADDR_WHO_AM_I) )
    		if(_register_read(ADDR_WHO_AM_I)!= ID_WHO_AM_I) {
    	_dev->get_semaphore()->give();
    	    return false;
    }

    _dev->setup_checked_registers(3);

    _register_write(ADDR_CTRL_REG1,CTRL_REG1_ODR_75HZ|CTRL_REG1_BDU|CTRL_REG1_EN_LPFP|CTRL_REG1_LPFP_CFG);
  //  _register_write(ADDR_CTRL_REG1,CTRL_REG1_ODR_75HZ|CTRL_REG1_BDU);
    _register_write(ADDR_CTRL_REG2,0x18);

    _instance = _frontend.register_sensor();
#if 0
    _dump_registers();
#endif
    _dev->get_semaphore()->give();

    //75HZ
    _dev->register_periodic_callback(1000000/75,
                                        FUNCTOR_BIND_MEMBER(&AP_Baro_LPS22HB::_timer, void));

    return true;
}

uint8_t AP_Baro_LPS22HB::_register_read(uint8_t reg)
{
	uint8_t val = 0;

	_dev->read_registers(reg,&val,1);

	return val;
}

void AP_Baro_LPS22HB::_register_write(uint8_t reg, uint8_t val)
{
	_dev->write_register(reg, val);
}

void AP_Baro_LPS22HB::_register_modify(uint8_t reg,uint8_t clearbits,uint8_t setbits)
{
		uint8_t val;

	    val = _register_read(reg);
	    val &= ~clearbits;
	    val |= setbits;
	    _register_write(reg, val);
}

void AP_Baro_LPS22HB::_timer(void)
{
	struct PACKED {
        uint8_t press_xl;
        uint8_t press_l;
        uint8_t press_h;
        uint8_t temp_l;
        uint8_t temp_h;
    } data;
    uint32_t raw;
    uint16_t t_raw;
	uint8_t readreg = ADDR_P_OUT_XL | 0x80;

//	_register_modify(ADDR_CTRL_REG2,0x01,CTRL_REG2_ONE_SHOT); //read signe;

	if(!(_register_read(ADDR_STATUS_REG) & 0x03))
	{
		accum_count++;
		goto check_registers;
	}

    if(!_dev->read_registers(readreg, (uint8_t *)&data,sizeof(data)))
    {
    	goto check_registers;
    }

#if 0
	printf("xl:%x\n",data.press_xl);
	printf("l:%x\n",data.press_l);
	printf("h:%x\n",data.press_h);
#endif

    raw = data.press_xl+(data.press_l<<8)+(data.press_h<<16);
    t_raw = data.temp_l+(data.temp_h<<8);

#if 0
    printf("p:%.3f\n", ((float)raw)/4096);
#endif

    if (_sem->take(0)) {
            _temperature = ((float)t_raw) / 100;
       //     _pressure =    barofilter.apply(((float)raw) / 4096);
            _pressure =    ((float)raw) / 4096;
            _has_sample = true;
            _sem->give();

        }


check_registers:
        	_dev->check_next_register();
}

void AP_Baro_LPS22HB::update()
{
    if (_sem->take_nonblocking()) {
        if (!_has_sample) {
            _sem->give();
            return;
        }
#if 0
    // debugging code for sample rate
    static uint32_t lastt;
    static uint32_t total;
    total += accum_count;
    uint32_t now = AP_HAL::micros();
    float dt = (now - lastt) * 1.0e-6;
    if (dt > 1) {
        printf("lps22hb:%u samples\n", total);
        lastt = now;
        total = 0;
    }
    accum_count = 0;
#endif
        _copy_to_frontend(_instance, _pressure, _temperature);
        _sem->give();
    }
}

void AP_Baro_LPS22HB::_dump_registers()
{
	printf("LPS22HB registers\n");
	for(uint8_t reg =ADDR_WHO_AM_I;reg <= 0x33 ;reg++)
	{
		 uint8_t v = _register_read(reg);
		 printf("%#x::%#x\n",(unsigned)reg,(unsigned)v);
	}
}
