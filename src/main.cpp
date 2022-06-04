#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include "Adafruit_BME680.h"

#include "credentials.h"
#include "def.h"

const char ssid[] = WIFI_SSID;
const char pass[] = WIFI_PASSWD;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

Adafruit_BME680 bme680;

ADC_MODE(ADC_VCC);

int vcc_mv;
float ds18b20_temp_sen_1;
float bme680_temp;
uint32_t bme680_press;
float bme680_hum;
uint32_t bme680_gas;
float bme680_alt;
uint32_t bme680_end_time;
uint8_t bme680_ready;

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

void setup()
{
    // Serial.begin(115200);

    // I2C stuff
    Wire.begin(0, 2);

    if (bme680.begin())
    {
        // Set up oversampling and filter initialization
        bme680.setTemperatureOversampling(BME680_OS_8X);
        bme680.setHumidityOversampling(BME680_OS_2X);
        bme680.setPressureOversampling(BME680_OS_4X);
        bme680.setIIRFilterSize(BME680_FILTER_SIZE_3);
        bme680.setGasHeater(320, 150); // 320*C for 150 ms

        // Tell BME680 to begin measurement.
        bme680_end_time = bme680.beginReading();
    }

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, pass);
    WiFi.hostname(HOSTNAME);
    // Onewire not working on Rx pin #3 when serial is enabled
    sensors.begin();
    sensors.setResolution(10);
    sensors.requestTemperatures();

    uint8_t cnt = 0;
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(100);
        cnt++;
        if (cnt > 10)
            ESP.deepSleep(SLEEP_STEP_MS * 1000);
    }

    mqttClient.setServer(MQTT_SERVER, MQTT_SERVER_PORT);

    if (mqttClient.connect(HOSTNAME, MQTT_LOGIN, MQTT_PASSWORD,
                           MQTT_WILL_TOPIC, MQTT_QOS, MQTT_RETAIN, MQTT_WILL_MESSAGE))
    {
        vcc_mv = read_filter_vcc();
        ds18b20_temp_sen_1 = sensors.getTempCByIndex(0);

        if (bme680_end_time > 0 && millis() >= bme680_end_time)
        {
            if (bme680.endReading())
            {
                bme680_temp = bme680.temperature;                       // ˚C
                bme680_press = bme680.pressure / 100.0;                 // hPa
                bme680_hum = bme680.humidity;                           // %
                bme680_gas = bme680.gas_resistance / 1000.0;            // KOhms
                bme680_alt = bme680.readAltitude(SEALEVELPRESSURE_HPA); // m
                bme680_ready = true;
            }
        }

        static char buff[20];
        sprintf(buff, "%d", vcc_mv);
        mqttClient.publish(DEFAULT_TOPIC "vcc", buff);
        if (ds18b20_temp_sen_1 > -55 && ds18b20_temp_sen_1 < 125)
        {
            sprintf(buff, "%0.2f", ds18b20_temp_sen_1);
            mqttClient.publish(DEFAULT_TOPIC "temp", buff);
        }
        if (bme680_ready)
        {
            sprintf(buff, "%f", bme680_temp);
            mqttClient.publish(DEFAULT_TOPIC "bme680/temp", buff);
            sprintf(buff, "%d", bme680_press);
            mqttClient.publish(DEFAULT_TOPIC "bme680/press", buff);
            sprintf(buff, "%f", bme680_hum);
            mqttClient.publish(DEFAULT_TOPIC "bme680/hum", buff);
            sprintf(buff, "%d", bme680_gas);
            mqttClient.publish(DEFAULT_TOPIC "bme680/gas", buff);
            sprintf(buff, "%f", bme680_alt);
            mqttClient.publish(DEFAULT_TOPIC "bme680/alt", buff);
        }
    }
    else
        ESP.deepSleep(SLEEP_STEP_MS * 1000);

    delay(1000);

    ESP.deepSleep(SLEEP_STEP_MS * 1000);
}

void loop()
{
}