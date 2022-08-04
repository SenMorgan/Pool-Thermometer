#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <Adafruit_BME280.h>

#include "credentials.h"
#include "def.h"

const char ssid[] = WIFI_SSID;
const char pass[] = WIFI_PASSWD;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

Adafruit_BME280 bme280;

ADC_MODE(ADC_VCC);

static int vcc_mv;
static float upper_water_sensor_temp, bottom_water_sensor_temp;
static float bme280_temp, bme280_hum, bme280_pres;
static uint8_t bme280_ready, sleep_enabled = 1;
static uint32_t time_stamp_sleep, time_stamp_publish, disconnected_time_stamp;

/**
 * @brief Read internal VCC voltage with oversampling
 *
 * @return Voltage in mV
 */
int read_filter_vcc()
{
    int32_t vcc_arr = 0;
    for (uint8_t i = 0; i < 20; i++)
    {
        vcc_arr += ESP.getVcc() * VCC_CORR_COEFFICIENT + VCC_CORR_OFFSET_MV;
        delay(2);
    }
    return vcc_arr / 20;
}

/**
 * @brief MQTT Callback
 */
void callback(String topic, byte *payload, uint16_t length)
{
    String msgString = "";
    for (uint16_t i = 0; i < length; i++)
        msgString += (char)payload[i];

    // Light control topic
    if (topic == MQTT_CMD_TOPIC_LIGHT)
    {
        if (msgString == MQTT_CMD_ON)
        {
            mqttClient.publish(MQTT_STATE_TOPIC_LIGHT, MQTT_CMD_ON, true);
            digitalWrite(LIGHT_PIN, HIGH);
            // Disable sleep mode if we want to control the light
            mqttClient.publish(MQTT_STATE_TOPIC_SLEEP, MQTT_CMD_OFF, true);
            sleep_enabled = 0;
        }
        else if (msgString == MQTT_CMD_OFF)
        {
            mqttClient.publish(MQTT_STATE_TOPIC_LIGHT, MQTT_CMD_OFF, true);
            digitalWrite(LIGHT_PIN, LOW);
        }
    }
    // Sleep mode control topic
    else if (topic == MQTT_CMD_TOPIC_SLEEP)
    {
        if (msgString == MQTT_CMD_ON)
        {
            sleep_enabled = 1;
            mqttClient.publish(MQTT_STATE_TOPIC_SLEEP, MQTT_CMD_ON, true);
            delay(DELAY_AFTER_PUBLISH_MS);
        }
        else if (msgString == MQTT_CMD_OFF)
        {
            sleep_enabled = 0;
            mqttClient.publish(MQTT_STATE_TOPIC_SLEEP, MQTT_CMD_OFF, true);
        }
    }
}

/**
 * @brief Initialize sensors after sleep mode
 *
 */
void init_sensors()
{
    bme280_ready = bme280.begin(BME280_ADDRESS_ALTERNATE);
    bme280.setSampling(
        bme280.MODE_FORCED,
        bme280.SAMPLING_X1,
        bme280.SAMPLING_X1,
        bme280.SAMPLING_X1,
        bme280.FILTER_OFF,
        bme280.STANDBY_MS_1000);

    // Onewire not working on Rx pin #3 when serial is enabled
    sensors.begin();
    sensors.setResolution(upper_water_thermometer, 12);
    sensors.setResolution(bottom_water_thermometer, 12);
}

/**
 * @brief Enable sensors sleep mode
 *
 */
void asleep_sensors()
{
    bme280.setSampling(        // use recommended settings for low-power weather monitoring
        bme280.MODE_SLEEP,     // sleep after each reading
        bme280.SAMPLING_X1,    // temperature 1x oversample
        bme280.SAMPLING_X1,    // pressure 1x oversample
        bme280.SAMPLING_X1,    // humidity 1x oversample
        bme280.FILTER_OFF,     // no IIR filtering
        bme280.STANDBY_MS_1000 // 1 sec standby duration
    );
}

/**
 * @brief Read sensors data
 *
 */
void read_send_sensors_data()
{
    sensors.requestTemperatures();
    vcc_mv = read_filter_vcc();                                            // mV
    upper_water_sensor_temp = sensors.getTempC(upper_water_thermometer);   // ˚C
    bottom_water_sensor_temp = sensors.getTempC(bottom_water_thermometer); // ˚C

    // Read data only if BME280 is connected
    if (bme280_ready)
    {
        bme280_temp = bme280.readTemperature();       // ˚C
        bme280_pres = bme280.readPressure() / 100.0F; // hPa
        bme280_hum = bme280.readHumidity();           // %
    }

    static char buff[20];
    if (vcc_mv > 0)
    {
        sprintf(buff, "%d", vcc_mv);
        mqttClient.publish(DEFAULT_TOPIC "vcc", buff);
    }
    if (upper_water_sensor_temp != DEVICE_DISCONNECTED_C)
    {
        sprintf(buff, "%0.2f", upper_water_sensor_temp);
        mqttClient.publish(DEFAULT_TOPIC "ds18b20/upper-water-temp", buff);
    }
    if (bottom_water_sensor_temp != DEVICE_DISCONNECTED_C)
    {
        sprintf(buff, "%0.2f", bottom_water_sensor_temp);
        mqttClient.publish(DEFAULT_TOPIC "ds18b20/bottom-water-temp", buff);
    }

    // Publish data only if BME280 is connected
    if (bme280_ready)
    {
        if (bme280_temp >= -40.0 && bme280_temp <= 80.0)
        {
            sprintf(buff, "%0.2f", bme280_temp);
            mqttClient.publish(DEFAULT_TOPIC "bme280/temp", buff);
        }
        if (bme280_hum >= 0.0 && bme280_hum <= 100.0)
        {
            sprintf(buff, "%0.2f", bme280_hum);
            mqttClient.publish(DEFAULT_TOPIC "bme280/hum", buff);
        }
        if (bme280_pres >= 300.0 && bme280_pres <= 1100.0)
        {
            sprintf(buff, "%0.2f", bme280_pres);
            mqttClient.publish(DEFAULT_TOPIC "bme280/press", buff);
        }
    }
}

void setup()
{
    pinMode(LIGHT_PIN, OUTPUT);
    digitalWrite(LIGHT_PIN, LOW);

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, pass);
    WiFi.hostname(HOSTNAME);

    // Arduino OTA initializing
    ArduinoOTA.setHostname(OTA_HOSTNAME);
    ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.begin();
    ArduinoOTA.onStart([]()
                       { digitalWrite(LIGHT_PIN, LOW); });

    // I2C stuff
    Wire.begin(0, 2);
    init_sensors();

    uint8_t cnt = 0;
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(100);
        cnt++;
        // Fall asleep if can't connect to the WiFi network
        if (cnt > 50)
            ESP.deepSleep(SLEEP_STEP_MS * 1000);
    }

    mqttClient.setServer(MQTT_SERVER, MQTT_SERVER_PORT);
    mqttClient.setCallback(callback);

    if (mqttClient.connect(HOSTNAME, MQTT_LOGIN, MQTT_PASSWORD,
                           MQTT_WILL_TOPIC, MQTT_QOS, MQTT_RETAIN, MQTT_WILL_MESSAGE))
    {
        mqttClient.subscribe(MQTT_SUBSCRIBE_TOPIC);
        // Send actual states
        mqttClient.publish(MQTT_STATE_TOPIC_SLEEP, MQTT_CMD_ON, true);
        mqttClient.publish(MQTT_STATE_TOPIC_LIGHT, MQTT_CMD_OFF, true);

        read_send_sensors_data();
    }
    // Fall asleep if can't connect to the broker
    else
    {
        asleep_sensors();
        delay(200);
        ESP.deepSleep(SLEEP_STEP_MS * 1000);
    }

    time_stamp_sleep = millis();
}

void loop()
{
    ArduinoOTA.handle();

    uint8_t connected_to_broker = mqttClient.loop();

    // If sleep mode enabled, wait for retained messages from broker
    if (sleep_enabled)
    {
        if (millis() - time_stamp_sleep > SLEEP_AFTER_MS)
        {
            asleep_sensors();
            delay(200);
            ESP.deepSleep(SLEEP_STEP_MS * 1000);
        }
    }
    else
    {
        // If sleep disabled and connected to the broker, read sensors data and publish it
        if (!sleep_enabled && connected_to_broker && millis() - time_stamp_publish > PUBLISH_INTERVAL_MS)
        {
            time_stamp_publish = millis();
            read_send_sensors_data();
        }

        // Save timestamp when loose connection to the broker
        if (!connected_to_broker && !disconnected_time_stamp)
            disconnected_time_stamp = millis();
        else if (connected_to_broker && disconnected_time_stamp)
            disconnected_time_stamp = 0;

        // Fall asleep after some time if can't connect to the broker
        if (disconnected_time_stamp && millis() - disconnected_time_stamp > SLEEP_AFTER_DISCONNECT_MS)
        {
            asleep_sensors();
            delay(200);
            ESP.deepSleep(SLEEP_STEP_MS * 1000);
        }
    }

    yield();
}