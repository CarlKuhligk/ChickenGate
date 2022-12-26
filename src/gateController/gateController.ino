#include <WiFi.h>
#include <MQTT.h>
#include <Arduino.h>
#include <Arduino_JSON.h>
#include "LinearActuator.h"

#include "config.h"
#include "ota.h"
LinearActuator *Axis1;
TaskHandle_t axis1Task;
void loopAxis1(void *parameter);

Preferences MyPreference;

WiFiClient net;
MQTTClient client;

void sendState();
long sendStateTimer = 0;
long sendStateTimeout = 1000;

void setup()
{
  MyPreference.begin("VariableStorage", false);
  Serial.begin(115200);
  pinMode(LED_D0_PIN, OUTPUT);

  connectWIFI();
  client.begin(mqttURL, mqttPort, net);
  client.onMessage(messageReceived);
  connectMQTT();
  startOTAWebserver();

  Axis1 = new LinearActuator(EN_PIN, PWM_PIN, CS_PIN, CH_A_PIN, CH_B_PIN, ENC_A_PIN, ENC_B_PIN, &MyPreference);
  Axis1->enable();
  Axis1->moveHome();

  xTaskCreatePinnedToCore(loopAxis1, "Axis1", 16384, NULL, 1, &axis1Task, 1);
}

void loop()
{
  // wifi & mqtt
  connectWIFI();
  connectMQTT();

  Axis1->loop();
  sendState();
  client.loop();
}

void loopAxis1(void *parameter)
{
  while (true)
  {
  }
}

void connectWIFI()
{
  // WIFI
  if (WiFi.status() != WL_CONNECTED)
  {
    WiFi.disconnect();
    Serial.println("WIFI reconnecting!");
    WiFi.begin(ssid, password);
    for (int i = 0; i < 30 && WiFi.status() != WL_CONNECTED; i++)
    {
      Serial.print(".");
      vTaskDelay(100);
      digitalWrite(LED_D0_PIN, !digitalRead(LED_D0_PIN));
    }
    if (WiFi.status() == WL_CONNECTED)
    {
      Serial.println("WIFI connected!");
    }
  }
}

void connectMQTT()
{
  // MQTT
  if (!client.connected() && WiFi.status() == WL_CONNECTED)
  {
    client.connect("ESP32GateController", mqttUser, mqttPassword);

    Serial.println("MQTT reconnecting!");
    for (int i = 0; i < 60 && !client.connected() && WiFi.status() == WL_CONNECTED; i++)
    {
      Serial.print(".");
      vTaskDelay(100);
      digitalWrite(LED_D0_PIN, !digitalRead(LED_D0_PIN));
    }
    if (client.connected())
    {
      client.subscribe("GateControl");
      client.publish("GateResponse", "connected");
      Serial.println("MQTT connected!\n");
      digitalWrite(LED_D0_PIN, HIGH);
    }
  }
}

void messageReceived(String &topic, String &payload)
{
  Serial.println("incoming: " + topic + " - " + payload);
  JSONVar myObject = JSON.parse(payload);

  if (topic == "GateControl")
  {
    if ((int)myObject["action"] == 1)
    {
      // open gate
      Axis1->moveHome();
    }
    else if ((int)myObject["action"] == 3)
    {
      // close gate
      Axis1->moveAbsolute(250.0f, 100.0f);
    }
  }
}

void sendState()
{
  if (millis() > sendStateTimeout + sendStateTimer)
  {
    sendStateTimer = millis();
    if (client.connected())
    {
      const String payload = Axis1->toJSON();
      client.publish("GateResponse", payload, false, 2);
    }
  }
}
