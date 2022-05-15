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
#define MQTT_PUBLISH_TOPIC        DEFAULT_TOPIC "state"
#define MQTT_AVAILABILITY_TOPIC   DEFAULT_TOPIC "availability"
#define MQTT_AVAILABILITY_MESSAGE "online"

// The time the board will be in deep sleep mode
#define SLEEP_STEP_MS           60000
// Input voltage offset measured directly on the input pin
#define INPUT_VCC_INT_OFFSET_MV -120
// Input voltage offset measured on the battery (with diode and charging circuit)
#define INPUT_VCC_EXT_OFFSET_MV 917
// Resulted offset in millivolts
#define VCC_OFFSET_MV           INPUT_VCC_INT_OFFSET_MV + INPUT_VCC_EXT_OFFSET_MV

// IO pins
#define STATUS_LED   2
#define ONE_WIRE_BUS 3

#endif // _DEF_H_