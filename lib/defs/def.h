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
#define MQTT_AVAILABILITY_TOPIC   DEFAULT_TOPIC "availability"
#define MQTT_AVAILABILITY_MESSAGE "online"
#define MQTT_CMD_TOPIC_LIGHT      DEFAULT_TOPIC "set/light"
#define MQTT_STATE_TOPIC_LIGHT    DEFAULT_TOPIC "state/light"
#define MQTT_CMD_TOPIC_SLEEP      DEFAULT_TOPIC "set/sleep"
#define MQTT_STATE_TOPIC_SLEEP    DEFAULT_TOPIC "state/sleep"
#define MQTT_CMD_ON               "1"
#define MQTT_CMD_OFF              "0"

// The time the board will be in deep sleep mode
#define SLEEP_STEP_MS             60000
// Some delay to process MQTT messages before going to sleep
#define SLEEP_AFTER_MS            2000
// Time to wait before going to sleep if can't connect to the broker
#define SLEEP_AFTER_DISCONNECT_MS 60000
// Interval between sensor readings and publish in non-sleep mode
#define PUBLISH_INTERVAL_MS       10000
// Some delay to process MQTT messages before going to sleep
#define DELAY_AFTER_PUBLISH_MS    1000
// VCC offset coefficient
#define VCC_CORR_COEFFICIENT      0.989
// Resulted offset in millivolts
#define VCC_CORR_OFFSET_MV        -0.0634
// Light voltage cutoff
#define LIGHT_CUTOFF_MV           3200

// IO pins
// #define STATUS_LED   2 // Can't use if I2C is enabled
#define ONE_WIRE_BUS 1
#define LIGHT_PIN    3

DeviceAddress upper_water_thermometer = {0x28, 0x8A, 0xBC, 0x48, 0xF6, 0xFC, 0x3C, 0x3C};
DeviceAddress bottom_water_thermometer = {0x28, 0xB1, 0x44, 0x48, 0xF6, 0xA6, 0x3C, 0x21};

#endif // _DEF_H_c