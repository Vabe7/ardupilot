/*
 * Copyright (C) 2016  Intel Corporation. All rights reserved.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "AP_Compass_BMM150.h"

#if AP_COMPASS_BMM150_ENABLED

#include <AP_HAL/AP_HAL.h>

#include <utility>

#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/AP_Math.h>
#include <stdio.h>

#if USE_BMM_SENSOR_API
#include "bmm150_defs.h"
#include "bmm150.h"
#endif  // USE_BMM_SENSOR_API

#define CHIP_ID_REG 0x40
#define CHIP_ID_VAL 0x32

#define POWER_AND_OPERATIONS_REG 0x4B
#define POWER_CONTROL_VAL (1 << 0)
#define SOFT_RESET (1 << 7 | 1 << 1)

#define OP_MODE_SELF_TEST_ODR_REG 0x4C
#define NORMAL_MODE (0 << 1)
#define ODR_30HZ (1 << 3 | 1 << 4 | 1 << 5)
#define ODR_20HZ (1 << 3 | 0 << 4 | 1 << 5)

#define DATA_X_LSB_REG 0x42

#define REPETITIONS_XY_REG 0x51
#define REPETITIONS_Z_REG 0X52

/* Trim registers */
#define DIG_X1_REG 0x5D
#define DIG_Y1_REG 0x5E
#define DIG_Z4_LSB_REG 0x62
#define DIG_Z4_MSB_REG 0x63
#define DIG_X2_REG 0x64
#define DIG_Y2_REG 0x65
#define DIG_Z2_LSB_REG 0x68
#define DIG_Z2_MSB_REG 0x69
#define DIG_Z1_LSB_REG 0x6A
#define DIG_Z1_MSB_REG 0x6B
#define DIG_XYZ1_LSB_REG 0x6C
#define DIG_XYZ1_MSB_REG 0x6D
#define DIG_Z3_LSB_REG 0x6E
#define DIG_Z3_MSB_REG 0x6F
#define DIG_XY2_REG 0x70
#define DIG_XY1_REG 0x71

#define MEASURE_TIME_USEC 16667

extern const AP_HAL::HAL &hal;

#if USE_BMM_SENSOR_API
BMM150_INTF_RET_TYPE AP_Compass_BMM150::bmm150_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    // printf("BMM150: read %ld bytes from %#04x\n", length, reg_addr);
    AP_Compass_BMM150* bmm_150 = (AP_Compass_BMM150*)intf_ptr;
    bool result = bmm_150->_dev->read_registers(reg_addr, reg_data, length);
    return result ? 0: -1;
}

BMM150_INTF_RET_TYPE AP_Compass_BMM150::bmm150_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    // printf("BMM150: write %ld bytes to %#04x\n", length, reg_addr);
    AP_Compass_BMM150* bmm_150 = (AP_Compass_BMM150*)intf_ptr;
    const uint8_t one_byte = 0x01;
    uint8_t reg = reg_addr;
    for (int i=0; i<length; ++i) {
        reg += one_byte;
        uint8_t val = reg_data[i];
        bool result = bmm_150->_dev->write_register(reg, val);
        if (!result) {
            return -1;
        }
    }
    return 0;
}

void AP_Compass_BMM150::bmm150_delay_us(uint32_t period, void */*intf_ptr*/)
{
    hal.scheduler->delay_microseconds(period);
}

void bmm150_error_codes_print_result(const char api_name[], int8_t rslt)
{
    if (rslt != BMM150_OK)
    {
        printf("%s\t", api_name);

        switch (rslt)
        {
            case BMM150_E_NULL_PTR:
                printf("Error [%d] : Null pointer error.", rslt);
                printf(
                    "It occurs when the user tries to assign value (not address) to a pointer, which has been initialized to NULL.\r\n");
                break;

            case BMM150_E_COM_FAIL:
                printf("Error [%d] : Communication failure error.", rslt);
                printf(
                    "It occurs due to read/write operation failure and also due to power failure during communication\r\n");
                break;

            case BMM150_E_DEV_NOT_FOUND:
                printf("Error [%d] : Device not found error. It occurs when the device chip id is incorrectly read\r\n",
                       rslt);
                break;

            case BMM150_E_INVALID_CONFIG:
                printf("Error [%d] : Invalid sensor configuration.", rslt);
                printf(" It occurs when there is a mismatch in the requested feature with the available one\r\n");
                break;

            default:
                printf("Error [%d] : Unknown error code\r\n", rslt);
                break;
        }
    }
}

#endif  // USE_BMM_SENSOR_API

AP_Compass_Backend *AP_Compass_BMM150::probe(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev, bool force_external, enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    AP_Compass_BMM150 *sensor = NEW_NOTHROW AP_Compass_BMM150(std::move(dev), force_external, rotation);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

AP_Compass_BMM150::AP_Compass_BMM150(AP_HAL::OwnPtr<AP_HAL::Device> dev, bool force_external, enum Rotation rotation)
    : _dev(std::move(dev)), _rotation(rotation), _force_external(force_external)
{
}

bool AP_Compass_BMM150::_load_trim_values()
{
    struct {
        int8_t dig_x1;
        int8_t dig_y1;
        uint8_t rsv[3];
        le16_t dig_z4;
        int8_t dig_x2;
        int8_t dig_y2;
        uint8_t rsv2[2];
        le16_t dig_z2;
        le16_t dig_z1;
        le16_t dig_xyz1;
        le16_t dig_z3;
        int8_t dig_xy2;
        uint8_t dig_xy1;
    } PACKED trim_registers, trim_registers2;

    // read the registers twice to confirm we have the right
    // values. There is no CRC in the registers and these values are
    // vital to correct operation
    int8_t tries = 4;
    while (tries--) {
        if (!_dev->read_registers(DIG_X1_REG, (uint8_t *)&trim_registers,
                                  sizeof(trim_registers))) {
            continue;
        }
        if (!_dev->read_registers(DIG_X1_REG, (uint8_t *)&trim_registers2,
                                  sizeof(trim_registers))) {
            continue;
        }
        if (memcmp(&trim_registers, &trim_registers2, sizeof(trim_registers)) == 0) {
            break;
        }
    }
    if (-1 == tries) {
        DEV_PRINTF("BMM150: Failed to load trim registers\n");
        return false;
    }

    _dig.x1 = trim_registers.dig_x1;
    _dig.x2 = trim_registers.dig_x2;
    _dig.xy1 = trim_registers.dig_xy1;
    _dig.xy2 = trim_registers.dig_xy2;
    _dig.xyz1 = le16toh(trim_registers.dig_xyz1);
    _dig.y1 = trim_registers.dig_y1;
    _dig.y2 = trim_registers.dig_y2;
    _dig.z1 = le16toh(trim_registers.dig_z1);
    _dig.z2 = le16toh(trim_registers.dig_z2);
    _dig.z3 = le16toh(trim_registers.dig_z3);
    _dig.z4 = le16toh(trim_registers.dig_z4);

    return true;
}

bool AP_Compass_BMM150::init()
{
    printf("BMM150: init\n");

    // https://github.com/boschsensortec/BMM150_SensorAPI
    int8_t rslt;

    // interface selection
    printf("BMM150: select interface\n");
    magdev.read = AP_Compass_BMM150::bmm150_read;
    magdev.write = AP_Compass_BMM150::bmm150_write;
    magdev.intf = BMM150_I2C_INTF;

    magdev.delay_us = AP_Compass_BMM150::bmm150_delay_us;
    magdev.intf_ptr = this;

#if 0 //USE_BMM_SENSOR_API
    _dev->get_semaphore()->take_blocking();

    // 10 retries for init
    _dev->set_retries(10);

    // use checked registers to cope with bus errors
    _dev->setup_checked_registers(4);

    // hold status of sensor api calls 
    // int8_t rslt;
    struct bmm150_settings settings;
    int8_t boot_tries = 4;

    // initialise
    // printf("BMM150: initialise driver\n");
    // rslt = bmm150_init(&magdev);
    // bmm150_error_codes_print_result("bmm150_init", rslt);
    // if (rslt != BMM150_OK) {
    //     goto bus_error;
    // }

    // replace bmm150_init, as it may fail on first attempt (invalid chip id)
    while (boot_tries--) {
        uint8_t reg_data;

        printf("BMM150: soft reset\n");
        // rslt = bmm150_soft_reset(&magdev);
        reg_data = BMM150_SET_SOFT_RESET;
        rslt = bmm150_set_regs(BMM150_REG_POWER_CONTROL, &reg_data, 1, &magdev);
        bmm150_error_codes_print_result("bmm150_soft_reset", rslt);
        hal.scheduler->delay(2);
        if (rslt != BMM150_OK) {
            continue;
        }

        printf("BMM150: set power control enable\n");
        reg_data = BMM150_POWER_CNTRL_ENABLE;
        rslt = bmm150_set_regs(BMM150_REG_POWER_CONTROL, &reg_data, 1, &magdev);
        bmm150_error_codes_print_result("bmm150_set_power_control_enable", rslt);
        hal.scheduler->delay(2);
        if (rslt != BMM150_OK) {
            continue;
        }

        printf("BMM150: get chip ID\n");
        rslt = bmm150_get_regs(BMM150_REG_CHIP_ID, &(magdev.chip_id), 1, &magdev);
        bmm150_error_codes_print_result("bmm150_get_regs", rslt);
        if (rslt != BMM150_OK) {
            continue;
        }

        if (magdev.chip_id == BMM150_CHIP_ID) {
            printf("BMM150: chip ID %#02x\n", magdev.chip_id);
            break;
        }
        if (0 == boot_tries) {
            printf("BMM150: wrong chip ID %#02x should be %#02x\n", magdev.chip_id, BMM150_CHIP_ID);
        }
    }
    if (-1 == boot_tries) {
        goto bus_error;
    }

    // perform self test
    // printf("BMM150: self test\n");
    // rslt = bmm150_perform_self_test(BMM150_SELF_TEST_NORMAL, &magdev);
    // hal.scheduler->delay(2);
    // if (rslt != BMM150_OK) {
    //     printf("BMM150: self test failed (%d)\n", rslt);
    // }

    // perform advanced self test
    // printf("BMM150: advanced self test\n");
    // rslt = bmm150_perform_self_test(BMM150_SELF_TEST_ADVANCED, &magdev);
    // hal.scheduler->delay(2);
    // if (rslt != BMM150_OK) {
    //     printf("BMM150: self test failed (%d)\n", rslt);
    // }

    //! @todo use the internal bmm150 function...
    printf("BMM150: load trim values\n");


    printf("BMM150: set sensor settings\n");
    // // settings.xyz_axes_control;
    // settings.pwr_mode = BMM150_POWERMODE_NORMAL;
    // // Table 3 p13 recommneded OBR is 20Hz 
    // settings.data_rate = 20;
    // // §5.8 p30  nXY = 1 + 2 * REPXY 
    // settings.xy_rep = (47 - 1) / 2;
    // // §5.8 p31  nZ = 1 + REPZ 
    // settings.z_rep = 83 - 1;
    // settings.preset_mode = BMM150_PRESETMODE_REGULAR;
    // // settings.int_settings;
    // rslt = bmm150_set_sensor_settings(BMM150_SEL_DATA_RATE | BMM150_SEL_XY_REP | BMM150_SEL_Z_REP, &settings, &magdev);
    // bmm150_error_codes_print_result("bmm150_set_sensor_settings", rslt);
    // if (rslt != BMM150_OK) {
    //     goto bus_error;
    // }
    printf("BMM150: set sensor preset mode\n");
    settings.preset_mode = BMM150_PRESETMODE_REGULAR;
    rslt = bmm150_set_presetmode(&settings, &magdev);
    bmm150_error_codes_print_result("bmm150_set_presetmode", rslt);
    if (rslt != BMM150_OK) {
        goto bus_error;
    }

    printf("BMM150: change operation mode to normal\n");
    settings.pwr_mode = BMM150_POWERMODE_NORMAL;
    rslt = bmm150_set_op_mode(&settings, &magdev);
    bmm150_error_codes_print_result("bmm150_set_op_mode_normal", rslt);
    if (rslt != BMM150_OK) {
        goto bus_error;
    }

    _dev->get_semaphore()->give();

    /* register the compass instance in the frontend */
    _dev->set_device_type(DEVTYPE_BMM150);
    if (!register_compass(_dev->get_bus_id(), _compass_instance)) {
        return false;
    }
    set_dev_id(_compass_instance, _dev->get_bus_id());

    set_rotation(_compass_instance, _rotation);

    if (_force_external) {
        set_external(_compass_instance, true);
    }

    // 2 retries for run
    _dev->set_retries(2);
    
    _dev->register_periodic_callback(MEASURE_TIME_USEC,
            FUNCTOR_BIND_MEMBER(&AP_Compass_BMM150::_update, void));

    _last_read_ms = AP_HAL::millis();
    
    printf("BMM150: initialised\n");
    return true;

bus_error:
    _dev->get_semaphore()->give();
    return false;

//! @todo restore
#else
    uint8_t val = 0;
    bool ret;

    _dev->get_semaphore()->take_blocking();

    // 10 retries for init
    _dev->set_retries(10);

    // use checked registers to cope with bus errors
    _dev->setup_checked_registers(4);
    
    int8_t boot_tries = 4;
    while (boot_tries--) {
        /* Do a soft reset */
        ret = _dev->write_register(POWER_AND_OPERATIONS_REG, SOFT_RESET);
        hal.scheduler->delay(2);
        if (!ret) {
            continue;
        }

        /* Change power state from suspend mode to sleep mode */
        ret = _dev->write_register(POWER_AND_OPERATIONS_REG, POWER_CONTROL_VAL, true);
        hal.scheduler->delay(2);
        if (!ret) {
            continue;
        }

        ret = _dev->read_registers(CHIP_ID_REG, &val, 1);
        if (!ret) {
            continue;
        }
        if (val == CHIP_ID_VAL) {
            // found it
            break;
        }
        if (boot_tries == 0) {
            DEV_PRINTF("BMM150: Wrong chip ID 0x%02x should be 0x%02x\n", val, CHIP_ID_VAL);
        }
    }
    if (-1 == boot_tries) {
        goto bus_error;
    }

    ret = _load_trim_values();
    if (!ret) {
        goto bus_error;
    }

    //! @todo - move
    rslt = read_trim_registers(&magdev);
    if (rslt != BMM150_OK) {
        goto bus_error;
    }

    /*
     * Recommended preset for high accuracy:
     * - Rep X/Y = 47
     * - Rep Z = 83
     * - ODR = 20
     * But we are going to use 30Hz of ODR
     */
    ret = _dev->write_register(REPETITIONS_XY_REG, (47 - 1) / 2, true);
    if (!ret) {
        goto bus_error;
    }
    ret = _dev->write_register(REPETITIONS_Z_REG, 83 - 1, true);
    if (!ret) {
        goto bus_error;
    }
    /* Change operation mode from sleep to normal and set ODR */
    ret = _dev->write_register(OP_MODE_SELF_TEST_ODR_REG, NORMAL_MODE | ODR_30HZ, true);
    if (!ret) {
        goto bus_error;
    }

    _dev->get_semaphore()->give();

    /* register the compass instance in the frontend */
    _dev->set_device_type(DEVTYPE_BMM150);
    if (!register_compass(_dev->get_bus_id(), _compass_instance)) {
        return false;
    }
    set_dev_id(_compass_instance, _dev->get_bus_id());

    set_rotation(_compass_instance, _rotation);

    if (_force_external) {
        set_external(_compass_instance, true);
    }

    // 2 retries for run
    _dev->set_retries(2);
    
    _dev->register_periodic_callback(MEASURE_TIME_USEC,
            FUNCTOR_BIND_MEMBER(&AP_Compass_BMM150::_update, void));

    _last_read_ms = AP_HAL::millis();
    
    return true;

bus_error:
    _dev->get_semaphore()->give();
    return false;
#endif  // USE_BMM_SENSOR_API
}

/*
 * Compensation algorithm got from https://github.com/BoschSensortec/BMM050_driver
 * this is not explained in datasheet.
 */
int16_t AP_Compass_BMM150::_compensate_xy(int16_t xy, uint32_t rhall, int32_t txy1, int32_t txy2) const
{
    int32_t inter = ((int32_t)_dig.xyz1) << 14;
    inter /= rhall;
    inter -= 0x4000;

    int32_t val = _dig.xy2 * ((inter * inter) >> 7);
    val += (inter * (((uint32_t)_dig.xy1) << 7));
    val >>= 9;
    val += 0x100000;
    val *= (txy2 + 0xA0);
    val >>= 12;
    val *= xy;
    val >>= 13;
    val += (txy1 << 3);

    return val;
}

int16_t AP_Compass_BMM150::_compensate_z(int16_t z, uint32_t rhall) const
{
    int32_t dividend = int32_t(z - _dig.z4) << 15;
    int32_t dividend2 = dividend - ((_dig.z3 * (int32_t(rhall) - int32_t(_dig.xyz1))) >> 2);

    int32_t divisor = int32_t(_dig.z1) * (rhall << 1);
    divisor += 0x8000;
    divisor >>= 16;
    divisor += _dig.z2;

    int16_t ret = constrain_int32(dividend2 / divisor, -0x8000, 0x8000);
#if 0
    static uint8_t counter;
    if (counter++ == 0) {
        printf("ret=%d z=%d rhall=%u z1=%d z2=%d z3=%d z4=%d xyz1=%d dividend=%d dividend2=%d divisor=%d\n",
               ret, z, rhall, _dig.z1, _dig.z2, _dig.z3, _dig.z4, _dig.xyz1, dividend, dividend2, divisor);
    }
#endif
    return ret;
}

void AP_Compass_BMM150::_update()
{
#if USE_BMM_SENSOR_API
    // bmm150_read_mag_data
    int8_t rslt;
    int16_t msb_data;
    uint8_t reg_data[BMM150_LEN_XYZR_DATA] = { 0 };
    struct bmm150_mag_data mag_data;
    struct bmm150_raw_mag_data raw_mag_data;

    rslt = bmm150_get_regs(BMM150_REG_DATA_X_LSB, reg_data, BMM150_LEN_XYZR_DATA, &magdev);

    // check data ready status
    if (rslt != BMM150_OK || !(reg_data[6] & 0x1)) {
        _dev->check_next_register();
        uint32_t now = AP_HAL::millis();
        if (now - _last_read_ms > 250) {
            // printf("BMM150: resetting\n");
            // cope with power cycle to sensor
            _last_read_ms = now;

            uint8_t reg_data;
            reg_data = BMM150_SET_SOFT_RESET;
            rslt = bmm150_set_regs(BMM150_REG_POWER_CONTROL, &reg_data, 1, &magdev);

            reg_data = BMM150_POWER_CNTRL_ENABLE;
            rslt = bmm150_set_regs(BMM150_REG_POWER_CONTROL, &reg_data, 1, &magdev);
        }
        return;
    }

    /* Mag X axis data */
    reg_data[0] = BMM150_GET_BITS(reg_data[0], BMM150_DATA_X);

    /* Shift the MSB data to left by 5 bits */
    /* Multiply by 32 to get the shift left by 5 value */
    msb_data = ((int16_t)((int8_t)reg_data[1])) * 32;

    /* Raw mag X axis data */
    raw_mag_data.raw_datax = (int16_t)(msb_data | reg_data[0]);

    /* Mag Y axis data */
    reg_data[2] = BMM150_GET_BITS(reg_data[2], BMM150_DATA_Y);

    /* Shift the MSB data to left by 5 bits */
    /* Multiply by 32 to get the shift left by 5 value */
    msb_data = ((int16_t)((int8_t)reg_data[3])) * 32;

    /* Raw mag Y axis data */
    raw_mag_data.raw_datay = (int16_t)(msb_data | reg_data[2]);

    /* Mag Z axis data */
    reg_data[4] = BMM150_GET_BITS(reg_data[4], BMM150_DATA_Z);

    /* Shift the MSB data to left by 7 bits */
    /* Multiply by 128 to get the shift left by 7 value */
    msb_data = ((int16_t)((int8_t)reg_data[5])) * 128;

    /* Raw mag Z axis data */
    raw_mag_data.raw_dataz = (int16_t)(msb_data | reg_data[4]);

    /* Mag R-HALL data */
    reg_data[6] = BMM150_GET_BITS(reg_data[6], BMM150_DATA_RHALL);
    raw_mag_data.raw_data_r = (uint16_t)(((uint16_t)reg_data[7] << 6) | reg_data[6]);

    // /* Compensated Mag X data in int16_t format */
    mag_data.x = compensate_x(raw_mag_data.raw_datax, raw_mag_data.raw_data_r, &magdev);

    // /* Compensated Mag Y data in int16_t format */
    mag_data.y = compensate_y(raw_mag_data.raw_datay, raw_mag_data.raw_data_r, &magdev);

    // /* Compensated Mag Z data in int16_t format */
    mag_data.z = compensate_z(raw_mag_data.raw_dataz, raw_mag_data.raw_data_r, &magdev);

    Vector3f raw_field = Vector3f{
        (float)mag_data.x,
        (float)mag_data.y,
        (float)mag_data.z
    };

    /* apply sensitivity scale 16 LSB/uT */
    raw_field /= 16;
    /* convert uT to milligauss */
    raw_field *= 10;

    // printf("raw_mag_data [%d, %d, %d]\n", raw_mag_data.raw_datax, raw_mag_data.raw_datay, raw_mag_data.raw_dataz);
    // printf("int_mag_data [%d, %d, %d]\n", mag_data.x, mag_data.y, mag_data.z);
    // printf("mag_data     [%f, %f, %f]\n", raw_field.x, raw_field.y, raw_field.z);

    _last_read_ms = AP_HAL::millis();

    accumulate_sample(raw_field, _compass_instance);
    _dev->check_next_register();
#else
    le16_t data[4];
    bool ret = _dev->read_registers(DATA_X_LSB_REG, (uint8_t *) &data, sizeof(data));

    /* Checking data ready status */
    if (!ret || !(data[3] & 0x1)) {
        // printf("mag_data not ready\n");
        _dev->check_next_register();
        uint32_t now = AP_HAL::millis();
        if (now - _last_read_ms > 250) {
            printf("BMM150: resetting\n");
            // cope with power cycle to sensor
            _last_read_ms = now;
            _dev->write_register(POWER_AND_OPERATIONS_REG, SOFT_RESET);
            _dev->write_register(POWER_AND_OPERATIONS_REG, POWER_CONTROL_VAL, true);
        }
        return;
    }

    const uint16_t rhall = le16toh(data[3]) >> 2;

    const uint16_t data_x = ((int16_t)le16toh(data[0])) >> 3;
    const uint16_t data_y = ((int16_t)le16toh(data[1])) >> 3;
    const uint16_t data_z = ((int16_t)le16toh(data[2])) >> 1;

    printf("mag_data [%d, %d, %d]\n", data_x, data_y, data_z);


    Vector3f raw_field = Vector3f{
        (float) _compensate_xy(((int16_t)le16toh(data[0])) >> 3,
                               rhall, _dig.x1, _dig.x2),
        (float) _compensate_xy(((int16_t)le16toh(data[1])) >> 3,
                               rhall, _dig.y1, _dig.y2),
        (float) _compensate_z(((int16_t)le16toh(data[2])) >> 1, rhall)};

    /* apply sensitivity scale 16 LSB/uT */
    raw_field /= 16;
    /* convert uT to milligauss */
    raw_field *= 10;

    _last_read_ms = AP_HAL::millis();

    accumulate_sample(raw_field, _compass_instance);
    _dev->check_next_register();
#endif  // USE_BMM_SENSOR_API
}

void AP_Compass_BMM150::read()
{
    drain_accumulated_samples(_compass_instance);
}


#endif  // AP_COMPASS_BMM150_ENABLED
