#define DEFAULT_MQTT_HOST "mqtt1.eoh.io"

// You should get Auth Token in the ERa App or ERa Dashboard
#define ERA_AUTH_TOKEN "731843c9-0753-4a0c-8da0-58ece75b285b"

#include <Arduino.h>
#include <ERa.hpp>
#include <ERa/ERaTimer.hpp>

#include <SoftwareSerial.h>
SoftwareSerial ArduinoSerial (D1, D2);

const char ssid[] = "TuanLucky";
const char pass[] = "0903721483";

String ChuoiReceiveArduinoJson = "";
String ChuoiSendArduinoJson    = "";

float temperature = 0.00;
float humidity    = 0.00;
float dust        = 0.00;

int   co2         = 0;
int   co          = 0;
int   smoke       = 0;
int   lpg         = 0;

uint64_t last = 0;

bool mode = false;
bool trig = false;

int fanmist  = 0;
int bluemode = 0;
int automode = 0;

void receiveJson();
void JsonSeparate(String input);
void sendJson();
void DataJson(String s1, String s2, String s3);

ERaTimer timer;

void timerEvent() {
  ERa.virtualWrite(V0, (ERaMillis() / 1000L / 3600L));
  ERa.virtualWrite(V1, ((ERaMillis() / 1000L / 60L) - int(ERaMillis() / 1000L / 3600L) * 60L));
  ERa.virtualWrite(V2, (ERaMillis() / 1000L) % 60L);
  if (ERaMillis() - last >= 80000)
  {
    ERa.virtualWrite(V3, temperature);
    ERa.virtualWrite(V4, humidity );
    ERa.virtualWrite(V5, co2      );
    ERa.virtualWrite(V6, co       );
    ERa.virtualWrite(V7, smoke    );
    ERa.virtualWrite(V8, lpg      );
    if (dust < 0) ERa.virtualWrite(V9, 0);
    else ERa.virtualWrite(V9, dust);
    ERA_LOG(ERA_PSTR("Timer"), ERA_PSTR("Uptime: %d"), ERaMillis() / 1000L);
  }
}

void setup() {
    /* Setup debug console */
    Serial.begin(9600);
    ArduinoSerial.begin(9600);

    ERa.begin(ssid, pass);
    timer.setInterval(1000L, timerEvent);
}

void loop() {
  receiveJson();

  if      (digitalRead(D6) == 0) mode = false;
  else if (digitalRead(D6) == 1) mode = true ;
  if      (digitalRead(D7) == 0) trig = false;
  else if (digitalRead(D7) == 1) trig = true ;

  if   ((mode == true)  && (trig == true) ) bluemode = 1;
  else bluemode = 0;
  if   ((mode == false) && (trig == true) ) automode = 1;
  else automode = 0;

  if      (digitalRead(D5) == 0) fanmist = 0;
  else if (digitalRead(D5) == 1) fanmist = 1;

  sendJson();

  ERa.run();
  timer.run();
}

void receiveJson()
{
  if (ArduinoSerial.available())
  {
    ChuoiReceiveArduinoJson = ArduinoSerial.readStringUntil('\n');
    JsonSeparate(ChuoiReceiveArduinoJson);
  }
}

void JsonSeparate(String input)
{
  int s11, s12, s21, s22, s31, s32, s41, s42, s51, s52, s61, s62, s71, s72;
  s11  = input.indexOf("\"Temp\":");
  s21  = input.indexOf("\"Humi\":");
  s31  = input.indexOf("\"CO2\":");
  s41  = input.indexOf("\"CO\":");
  s51  = input.indexOf("\"Smoke\":");
  s61  = input.indexOf("\"LPG\":");
  s71  = input.indexOf("\"Dust\":");

  s12  = input.indexOf("\",\"Humi\":");
  s22  = input.indexOf("\",\"CO2\":");
  s32  = input.indexOf("\",\"CO\":");
  s42  = input.indexOf("\",\"Smoke\":");
  s52  = input.indexOf("\",\"LPG\":");
  s62  = input.indexOf("\",\"Dust\":");
  s72  = input.indexOf("\"}");

  String s1 = input.substring(s11 + 8, s12);
  String s2 = input.substring(s21 + 8, s22);
  String s3 = input.substring(s31 + 7, s32);
  String s4 = input.substring(s41 + 6, s42);
  String s5 = input.substring(s51 + 9, s52);
  String s6 = input.substring(s61 + 7, s62);
  String s7 = input.substring(s71 + 8, s72);

  temperature = s1.toFloat();
  humidity    = s2.toFloat();
  co2         = s3.toInt();
  co          = s4.toInt();
  smoke       = s5.toInt();
  lpg         = s6.toInt();
  dust        = s7.toFloat();
}

void sendJson()
{
  DataJson(String(fanmist), String(bluemode), String (automode));
  ArduinoSerial.println(ChuoiSendArduinoJson);
}

void DataJson(String s1, String s2, String s3)
{
  ChuoiSendArduinoJson = "{\"Fan&Mist\":\"" + s1 + "\"," + "\"BlueMode\":\"" + s2 + "\","
                       + "\"AutoMode\":\""  + s3 + "\"}";
}