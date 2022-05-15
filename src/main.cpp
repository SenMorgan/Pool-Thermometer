#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include "credentials.h"
#include "def.h"

const char ssid[] = WIFI_SSID;
const char pass[] = WIFI_PASSWD;

WiFiClient   espClient;
PubSubClient mqttClient(espClient);

OneWire           oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

ADC_MODE(ADC_VCC);

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
    pinMode(STATUS_LED, OUTPUT);

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, pass);
    WiFi.hostname(HOSTNAME);
    // Onewire not working on Rx pin #3 when serial is enabled
    // Serial.begin(115200);
    sensors.begin();
    sensors.setResolution(10);
    sensors.requestTemperatures();

    uint8_t cnt = 0;
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        // Serial.print(".");
        cnt++;
        if (cnt > 10)
            ESP.deepSleep(SLEEP_STEP_MS * 1000);
    }

    mqttClient.setServer(MQTT_SERVER, MQTT_SERVER_PORT);

    if (mqttClient.connect(HOSTNAME, MQTT_LOGIN, MQTT_PASSWORD,
                           MQTT_WILL_TOPIC, MQTT_QOS, MQTT_RETAIN, MQTT_WILL_MESSAGE))
    {
        // Serial.println("connected");
        static char buff[20];
        sprintf(buff, "%d", read_filter_vcc());
        mqttClient.publish(DEFAULT_TOPIC "vcc", buff);
        float int_temp = sensors.getTempCByIndex(0);
        if (int_temp > -55 && int_temp < 125)
        {
            sprintf(buff, "%0.2f", int_temp);
            mqttClient.publish(DEFAULT_TOPIC "temp", buff);
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