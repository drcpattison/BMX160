/*!
 * @file DPEng_BMX160_AK09916.h
 *
 * This header file is part of DPEng's BMX160 driver for the Arduino platform.  It is
 * designed specifically to work with the DPEng BMX160 breakout board:
 * [AMAZON & EBAY BMX160 BREAKOUT BOARD PRODUCT LINKS ARE COMING SOON!!!]
 *
 * These sensors use I2C to communicate, 2 pins (SCX+SDX) are required
 * to interface with the breakout. SPI is also possible with <2 MBIT/s speeds.
 *
 * DP Engineering invests time and resources providing this open source code,
 * please support DPEng by purchasing this breakout board from DPEng
 *
 * Written by David Pattison for DP Engineering Ltd.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
/** \file DPEng_BMX160.h */
#ifndef __DPEng_BMX160_H__
#define __DPEng_BMX160_H__

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Adafruit_Sensor.h>
#include <Wire.h>

/*=========================================================================
    I2C ADDRESS/BITS AND SETTINGS
    -----------------------------------------------------------------------*/
    /** 7-bit I2C address for this sensor */
    #define BMX160_ADDRESS           (0x68)     // 0110 1001
    /** Device ID for this sensor (used as sanity check during init) */
    #define BMX160_ID                (0xD8)     // 1110 1010
	
	/** Macro for mg per LSB at +/- 2g sensitivity (1 LSB = 0.000061035mg) */
	#define ACCEL_MG_LSB_2G (0.000061035F)
	/** Macro for mg per LSB at +/- 4g sensitivity (1 LSB = 0.000122070mg) */
	#define ACCEL_MG_LSB_4G (0.000122070F)
	/** Macro for mg per LSB at +/- 8g sensitivity (1 LSB = 0.000244141mg) */
	#define ACCEL_MG_LSB_8G (0.000244141F)
	/** Macro for mg per LSB at +/- 16g sensitivity (1 LSB = 0.000488281mg) */
	#define ACCEL_MG_LSB_16G (0.000488281F)
	
	/** Gyroscope sensitivity at 125dps */
    #define GYRO_SENSITIVITY_125DPS  (0.0038110F) // Table 1 of datasheet
	/** Gyroscope sensitivity at 250dps */
    #define GYRO_SENSITIVITY_250DPS  (0.0076220F) // Table 1 of datasheet
    /** Gyroscope sensitivity at 500dps */
    #define GYRO_SENSITIVITY_500DPS  (0.0152439F)
    /** Gyroscope sensitivity at 1000dps */
    #define GYRO_SENSITIVITY_1000DPS (0.0304878F)
    /** Gyroscope sensitivity at 2000dps */
    #define GYRO_SENSITIVITY_2000DPS (0.0609756F)
	
	/** Macro for micro tesla (uT) per LSB (1 LSB = 0.1uT) */
	#define MAG_UT_LSB      (0.3F)
	
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    /*!
        Raw register addresses used to communicate with the sensor.
    */
    typedef enum
    {
		CHIP_ID  		= 0x00, // Should return 0xEA
		ERR_REG         = 0x02,  // Bit 7 enable DMP, bit 3 reset DMP
		PMU_STATUS      = 0x03,  // Bit 7 enable DMP, bit 3 reset DMP
		DATA_0   		= 0x04,
		DATA_1   		= 0x05,
		DATA_2   		= 0x06,
		DATA_3   		= 0x07,
		DATA_4   		= 0x08,
		DATA_5   		= 0x09,
		DATA_6   		= 0x0A,
		DATA_7   		= 0x0B,
		DATA_8   		= 0x0C,
		DATA_9   		= 0x0D,
		DATA_10   		= 0x0E,
		DATA_11   		= 0x0F,
		DATA_12   		= 0x10,
		DATA_13   		= 0x11,
		DATA_14   		= 0x12,
		DATA_15   		= 0x13,
		DATA_16  		= 0x14,
		DATA_17  		= 0x15,
		DATA_18   		= 0x16,
		DATA_19   		= 0x17,
		SENSOERTIME_0	= 0x18, 
		SENSOERTIME_1	= 0x19, 
		SENSOERTIME_2	= 0x1A, 
		STATUS	   		= 0x1B, 
		INT_STATUS_0	= 0x1C, 
		INT_STATUS_1	= 0x1D, 
		INT_STATUS_2	= 0x1E, 
		INT_STATUS_3	= 0x1F,
		TEMPERATURE_0	= 0x20,
		TEMPERATURE_1	= 0x21,
		FIFO_LENGTH_0	= 0x22,
		FIFO_LENGTH_1	= 0x23,
		FIFO_DATA		= 0x24,
		ACC_CONF		= 0x40,
		ACC_RANGE		= 0x41,
		GYR_CONF		= 0x42,
		GYR_RANGE		= 0x43,
		MAG_CONF		= 0x44,
		FIFO_DOWNS		= 0x45,	
		FIFO_CONFIG_0	= 0x46,	
		FIFO_CONFIG_1	= 0x47,	
		MAG_IF_0		= 0x4C,	
		MAG_IF_1		= 0x4D,	
		MAG_IF_2		= 0x4E,	
		MAG_IF_3		= 0x4F,
		INT_EN_0		= 0x50,
		INT_EN_1		= 0x51,
		INT_EN_2		= 0x52,
		INT_OUT_CTRL	= 0x53,
		INT_LATCH		= 0x54,
		INT_MAP_0		= 0x55,
		INT_MAP_1		= 0x56,
		INT_MAP_2		= 0x57,
		INT_DATA_0		= 0x58,
		INT_DATA_1		= 0x59,
		INT_LOWHIGH_0	= 0x5A,
		INT_LOWHIGH_1	= 0x5B,
		INT_LOWHIGH_2	= 0x5C,
		INT_LOWHIGH_3	= 0x5D,
		INT_LOWHIGH_4	= 0x5E,
		INT_MOTION_0	= 0x5F,
		INT_MOTION_1	= 0x60,
		INT_MOTION_2	= 0x61,
		INT_MOTION_3	= 0x62,
		INT_TAP_0		= 0x63,
		INT_TAP_1		= 0x64,
		INT_ORIENT_0	= 0x65,
		INT_ORIENT_1	= 0x66,
		INT_FLAT_0		= 0x67,
		INT_FLAT_1		= 0x68,
		FOC_COMF		= 0x69,
		COMF			= 0x6A,
		IF_CONF			= 0x6B,
		PMU_TRIGGER		= 0x6C,
		SELF_TEST		= 0x6D,
		NV_CONF			= 0x70,
		OFFSET_0		= 0x71,
		OFFSET_1		= 0x72,
		OFFSET_2		= 0x73,
		OFFSET_3		= 0x74,
		OFFSET_4		= 0x75,
		OFFSET_5		= 0x76,
		OFFSET_6		= 0x77,
		STEP_CNT_0		= 0x78,
		STEP_CNT_1		= 0x79,
		STEP_CONF_0		= 0x7A,
		STEP_CONF_1		= 0x7B,
		CMD				= 0x7E
    } bmx160Registers_t;
	
/*=========================================================================*/

/*=========================================================================
    OPTIONAL SPEED SETTINGS
    -----------------------------------------------------------------------*/
    /*!
        Range settings for the accelerometer sensor.
    */
    typedef enum
    {
      BMX160_ACCELRANGE_2G                = (0b0011),
      BMX160_ACCELRANGE_4G                = (0b0101),
      BMX160_ACCELRANGE_8G                = (0b1000),
	  BMX160_ACCELRANGE_16G               = (0b1100)
    } bmx160AccelRange_t;
	
	/*!
        Enum to define valid gyroscope range values
    */
    typedef enum
    {
	  GYRO_RANGE_125DPS  = 125,     /**< 125dps */
      GYRO_RANGE_250DPS  = 250,     /**< 250dps */
      GYRO_RANGE_500DPS  = 500,     /**< 500dps */
      GYRO_RANGE_1000DPS = 1000,    /**< 1000dps */
      GYRO_RANGE_2000DPS = 2000     /**< 2000dps */
    } bmx160GyroRange_t;
/*=========================================================================*/

/*=========================================================================
    RAW GYROSCOPE DATA TYPE
    -----------------------------------------------------------------------*/
    /*!
        @brief  Raw (integer) values from the gyroscope sensor.
    */
    typedef struct
    {
      int16_t x;    /**< Raw int16_t value from the x axis */
      int16_t y;    /**< Raw int16_t value from the y axis */
      int16_t z;    /**< Raw int16_t value from the z axis */
    } bmx160RawData_t;
/*=========================================================================*/

/**************************************************************************/
/*!
    @brief  Unified sensor driver for the DPEng BMX160 breakout.
*/
/**************************************************************************/
class DPEng_BMX160 : public Adafruit_Sensor
{
  public:
    DPEng_BMX160(int32_t accelSensorID = -1, int32_t gyroSensorID = -1, int32_t magSensorID = -1);

    bool begin           ( bmx160AccelRange_t rngAccel = BMX160_ACCELRANGE_2G, bmx160GyroRange_t rngGyro = GYRO_RANGE_250DPS );
    bool getEvent        ( sensors_event_t* accel );
    void getSensor       ( sensor_t* accel );
    bool getEvent        ( sensors_event_t* accel, sensors_event_t* gyro, sensors_event_t* mag );
    void getSensor       ( sensor_t* accel, sensor_t* gyro, sensor_t* mag );
    void standby         ( boolean standby );

    /*! Raw accelerometer values from last sucsessful sensor read */
    bmx160RawData_t accel_raw;
    /*! Raw gyroscope values from last successful sensor read */
    bmx160RawData_t gyro_raw;
	/*! Raw magnetometer values from last successful sensor read */
    bmx160RawData_t mag_raw;

  private:
	void        write8  ( byte address, byte reg, byte value );
    void        write8  ( byte reg, byte value );
	byte		read8	( byte address, byte reg );
    byte        read8   ( byte reg );

    bmx160AccelRange_t 	_rangeAccel;
    bmx160GyroRange_t 	_rangeGyro;
    int32_t             _accelSensorID;
    int32_t             _gyroSensorID;
	int32_t             _magSensorID;
};

#endif
