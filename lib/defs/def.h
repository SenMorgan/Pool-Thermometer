/**
 * @file def.h
 * @author SenMorgan https://github.com/SenMorgan
 * @date 2021-11-16
 *
 * @copyright Copyright (c) 2021 Sen Morgan
 *
 */

#ifndef _DEF_H_
#define _DEF_H_

// MQTT definitions
#define DEFAULT_TOPIC             "/pool-thermometer/"
#define MQTT_WILL_TOPIC           DEFAULT_TOPIC "availability"
#define MQTT_QOS                  1
#define MQTT_RETAIN               0
#define MQTT_WILL_MESSAGE         DEFAULT_TOPIC "offline"
#define MQTT_SUBSCRIBE_TOPIC      DEFAULT_TOPIC "#"
#define MQTT_PUBLISH_TOPIC        DEFAULT_TOPIC "state/light"
#define MQTT_AVAILABILITY_TOPIC   DEFAULT_TOPIC "availability"
#define MQTT_AVAILABILITY_MESSAGE "online"
#define MQTT_CMD_TOPIC            DEFAULT_TOPIC "set/light"
#define MQTT_CMD_ON               "1"
#define MQTT_CMD_OFF              "0"

// The time the board will be in deep sleep mode
#define SLEEP_STEP_MS           60000
// Input voltage offset measured directly on the input pin
#define INPUT_VCC_INT_OFFSET_MV -120
// Input voltage offset measured on the battery (with diode and charging circuit)
#define INPUT_VCC_EXT_OFFSET_MV 917
// Resulted offset in millivolts
#define VCC_OFFSET_MV           INPUT_VCC_INT_OFFSET_MV + INPUT_VCC_EXT_OFFSET_MV
/** This variable saves the pressure at the sea level in hectopascal (is equivalent to milibar).
 * This variable is used to estimate the altitude for a given pressure
 * by comparing it with the sea level pressure. This example uses the default value,
 * but for accurate results, replace the value with the current sea level pressure at your location. */
#define SEALEVELPRESSURE_HPA    (1015.2)

// IO pins
// #define STATUS_LED   2 // Can't use if I2C is enabled
#define ONE_WIRE_BUS 1
#define LIGHT_PIN    3

DeviceAddress upper_water_thermometer = {0x28, 0x8A, 0xBC, 0x48, 0xF6, 0xFC, 0x3C, 0x3C};
DeviceAddress bottom_water_thermometer = {0x28, 0xB1, 0x44, 0x48, 0xF6, 0xA6, 0x3C, 0x21};

#endif // _DEF_H_c