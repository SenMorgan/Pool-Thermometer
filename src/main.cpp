#include <Arduino.h>
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
static uint32_t time_stamp;

int read_filter_vcc()
{
    int32_t vcc_arr = 0;
    for (uint8_t i = 0; i < 20; i++)
    {
        vcc_arr += ESP.getVcc() + VCC_OFFSET_MV;
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

    // Commands topic
    if (topic == MQTT_CMD_TOPIC)
    {
        if (msgString == MQTT_CMD_ON)
        {
            mqttClient.publish(MQTT_PUBLISH_TOPIC, MQTT_CMD_ON, true);
            digitalWrite(LIGHT_PIN, HIGH);
            sleep_enabled = 0;
        }
        else if (msgString == MQTT_CMD_OFF)
        {
            mqttClient.publish(MQTT_PUBLISH_TOPIC, MQTT_CMD_OFF, true);
            digitalWrite(LIGHT_PIN, LOW);
            ESP.deepSleep(SLEEP_STEP_MS * 1000);
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

    // I2C stuff
    Wire.begin(0, 2);
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
    sensors.requestTemperatures();

    uint8_t cnt = 0;
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(100);
        cnt++;
        if (cnt > 50)
            ESP.deepSleep(SLEEP_STEP_MS * 1000);
    }

    mqttClient.setServer(MQTT_SERVER, MQTT_SERVER_PORT);
    mqttClient.setCallback(callback);

    if (mqttClient.connect(HOSTNAME, MQTT_LOGIN, MQTT_PASSWORD,
                           MQTT_WILL_TOPIC, MQTT_QOS, MQTT_RETAIN, MQTT_WILL_MESSAGE))
    {
        mqttClient.subscribe(MQTT_SUBSCRIBE_TOPIC);
        vcc_mv = read_filter_vcc();                                            // mV
        upper_water_sensor_temp = sensors.getTempC(upper_water_thermometer);   // ˚C
        bottom_water_sensor_temp = sensors.getTempC(bottom_water_thermometer); // ˚C

        // Read data only if BME280 is connected
        if (bme280_ready)
        {
            bme280_temp = bme280.readTemperature();       // ˚C
            bme280_pres = bme280.readPressure() / 100.0F; // hPa
            bme280_hum = bme280.readHumidity();           // %
            bme280.setSampling(                           // use recommended settings for low-power weather monitoring
                bme280.MODE_SLEEP,                        // sleep after each reading
                bme280.SAMPLING_X1,                       // temperature 1x oversample
                bme280.SAMPLING_X1,                       // pressure 1x oversample
                bme280.SAMPLING_X1,                       // humidity 1x oversample
                bme280.FILTER_OFF,                        // no IIR filtering
                bme280.STANDBY_MS_1000                    // 1 sec standby duration
            );
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
    else
        ESP.deepSleep(SLEEP_STEP_MS * 1000);

    time_stamp = millis();
}

void loop()
{
    mqttClient.loop();

    if (sleep_enabled && millis() - time_stamp > 2000)
    {
        ESP.deepSleep(SLEEP_STEP_MS * 1000);
    }
    yield();
}