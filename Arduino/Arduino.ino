/* Vị trí thứ tự bánh xe:
    2       1

    3       4
*/

/* Include Library */
#include <Servo.h>
#include <AFMotor.h>
#include <Wire.h>
#include <SparkFun_Qwiic_Humidity_AHT20.h>

/* Define Digital and Analog Pins */
#define fan       48
#define mist      49
#define echo      50
#define trigger   51
#define dust_led  52
#define buzzer    53

#define MQ135PIN  A1
#define MQ7PIN    A2
#define MQ2PIN    A3
#define dust_a    A15

#define dustThreshold   0.50
#define LPGThreshold    10
#define smokeThreshold  10
#define COThreshold     10
#define CO2Threshold    420
#define tempThreshold   30
#define humiThreshold   30

#define         RL_VALUE_MQ135                 2
#define         RO_CLEAN_AIR_FACTOR_MQ135      3.59
#define         RL_VALUE_MQ2                   36
#define         RO_CLEAN_AIR_FACTOR_MQ2        9.577
#define         RL_VALUE_MQ7                   5
#define         RO_CLEAN_AIR_FACTOR_MQ7        26.09
#define         CALIBARAION_SAMPLE_TIMES       50
#define         CALIBRATION_SAMPLE_INTERVAL    500
#define         READ_SAMPLE_INTERVAL           5
#define         READ_SAMPLE_TIMES              2
#define         GAS_CARBON_DIOXIDE             9
#define         GAS_CARBON_MONOXIDE            3
#define         GAS_SMOKE                      5
#define         GAS_LPG                        1

/* Define Serial Port */
#define BLESerial Serial2
#define ESPSerial Serial1

AHT20 tempHumi;
AF_DCMotor motor1(1, MOTOR12_1KHZ);
AF_DCMotor motor2(2, MOTOR12_1KHZ);
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);
Servo servo;

/* Declare variables */
float temperature = 0.00;
float humidity    = 0.00;
float dust        = 0.00;
float Ro2         = 0.00;
float Ro7         = 0.00;
float Ro135       = 0.00;

int vSpeed   = 120;
int pos      = 90;
int distance = 0;
int Left     = 0;
int Right    = 0;
int co2      = 0;
int co       = 0;
int smoke    = 0;
int lpg      = 0;

bool flagMist = false;
int  MIST     = 0;
int  preMIST  = 0;
int  fanmist  = 0;
int  prefm    = 0;
int  bluemode = 0;
int  automode = 0;
int  preauto  = 0;

char state;

String ChuoiSendESPJson    = "";
String ChuoiReceiveESPJson = "";

float MQ135Calibration(int mq_pin);
float MQ135ResistanceCalculation(int raw_adc);
float MQ135Read(int mq_pin);
int   MQ135GetGasPercentage(float rs_ro_ratio, int gas_id);
float MQ2Calibration(int mq_pin);
float MQ2ResistanceCalculation(int raw_adc);
float MQ2Read(int mq_pin);
int   MQ2GetGasPercentage(float rs_ro_ratio, int gas_id);
float MQ7Calibration(int mq_pin);
float MQ7ResistanceCalculation(int raw_adc);
float MQ7Read(int mq_pin);
int   MQ7GetGasPercentage(float rs_ro_ratio, int gas_id);

/* Declare functions */
void turnONMist();
void turnOFFMist();
void triggerBuzzFanandMist();
void getSensorData();
void BLEData();
void forward();
void backward();
void left();
void right();
void topleft();
void topright();
void bottomleft();
void bottomright();
void rotateCCW();
void rotateCW();
void Stop();
void sendDataToESP();
void DataJson(String s1, String s2, String s3, String s4, String s5, String s6, String s7);
void receiveDataFromESP();
void JsonSeparate(String input);

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  BLESerial.begin(9600);
  ESPSerial.begin(9600);
  
  pinMode(mist    , OUTPUT);
  pinMode(fan     , OUTPUT);
  pinMode(trigger , OUTPUT);
  pinMode(echo    ,  INPUT);
  pinMode(dust_led, OUTPUT);
  pinMode(buzzer  , OUTPUT);

  servo.attach(10);
  servo.write(90);
  for (pos = 90; pos <= 180; pos++)
  {
    servo.write(pos);
    delay(10);
  }
  for (pos = 180; pos >= 0; pos--)
  {
    servo.write(pos);
    delay(10);
  }
  for (pos = 0; pos <= 90; pos++)
  {
    servo.write(pos);
    delay(10);
  }

  Ro2   = MQ2Calibration(MQ2PIN);
  Ro7   = MQ7Calibration(MQ7PIN);
  Ro135 = MQ135Calibration(MQ135PIN); 

  if (tempHumi.begin() == false)
  {
    Serial.println("AHT20 not detected. Please check wiring. Freezing.");
    while (1);
  }
  Serial.println("AHT20 acknowledged.");
}

void loop()
{
  getSensorData();
  sendDataToESP();
  BLEData();
  if (automode == 1) avoid();
  else if (automode == 0 && preauto == 1)
  {
    preauto = automode;
    Stop();
  }
  preauto = automode;
  receiveDataFromESP();

  if (MIST != preMIST)
  {
    preMIST = MIST;
    if      (MIST == 0) turnOFFMist();
    else if (MIST == 1) turnONMist();
  }

  if (fanmist != prefm)
  {
    prefm = fanmist;
    if (fanmist == 0)
    {
      turnOFFMist();
      digitalWrite(fan, LOW);
    }
    else if (fanmist == 1)
    {
      digitalWrite(fan, LOW);
      turnONMist();
      digitalWrite(fan, HIGH);
    }
  }

  if (((temperature > tempThreshold) || (humidity < humiThreshold) || (co2 > CO2Threshold)
      || (co > COThreshold) || (smoke > smokeThreshold) || (lpg > LPGThreshold) 
      || (dust > dustThreshold)) && (temperature != 0) && (humidity != 0))
  {
    triggerBuzzFanandMist();
    delay(1000);
  }
}

void turnONMist()
{
  if (flagMist == false)
  {
    digitalWrite(mist, HIGH);
    delay(100);
    digitalWrite(mist, LOW);
    flagMist = true;
  }
}

void turnOFFMist()
{
  if (flagMist == true)
  {
    digitalWrite(mist, HIGH);
    delay(100);
    digitalWrite(mist, LOW);
    delay(100);
    digitalWrite(mist, HIGH);
    delay(100);
    digitalWrite(mist, LOW);
    delay(100);
    flagMist = false;
  }
}

void triggerBuzzFanandMist()
{
  for (int i = 0; i < 5; i++)
  {
    digitalWrite(buzzer, HIGH);
    delay(200);
    digitalWrite(buzzer, LOW);
    delay(200);
  }
  digitalWrite(mist, HIGH);
  digitalWrite(fan, HIGH);
  delay(100);
  digitalWrite(mist, LOW);
  delay(5000);
  digitalWrite(mist, HIGH);
  delay(100);
  digitalWrite(mist, LOW);
  delay(100);
  digitalWrite(mist, HIGH);
  delay(100);
  digitalWrite(mist, LOW);
  delay(100);
  digitalWrite(fan, LOW);
}

void getSensorData()
{
  /* Temperature and Humidity */
  if (tempHumi.available() == true)
  {
    temperature = tempHumi.getTemperature();
    humidity    = tempHumi.getHumidity();
  }

  /* Air Component */
  co2   = MQ135GetGasPercentage(MQ135Read(MQ135PIN)/Ro135, GAS_CARBON_DIOXIDE) ;
  co    = MQ135GetGasPercentage(MQ135Read(MQ135PIN)/Ro135, GAS_CARBON_MONOXIDE);
  smoke = MQ2GetGasPercentage(MQ2Read(MQ2PIN)/Ro2, GAS_SMOKE);
  lpg   = MQ7GetGasPercentage(MQ7Read(MQ7PIN)/Ro7, GAS_LPG)  ;

  /* Dust Density */
  int samplingTime = 280;
  int deltaTime = 40;
  int sleepTime = 9680;
  float voMeasured = 0;
  float calcVoltage = 0;

  digitalWrite(dust_led, LOW);
  delayMicroseconds(samplingTime);
  voMeasured = analogRead(dust_a);
  delayMicroseconds(deltaTime);
  digitalWrite(dust_led, HIGH);
  delayMicroseconds(sleepTime);

  calcVoltage = voMeasured * (5.0 / 1023);
  dust = 0.17 * calcVoltage - 0.1;
  delay(100);
}

int calculateDistance()
{
  digitalWrite(trigger, LOW);
  delay(2);
  digitalWrite(trigger, HIGH);
  delay(5);
  digitalWrite(trigger, LOW);

  unsigned long duration = pulseIn(echo, HIGH);
  int cm = duration/2./29.412;
  return cm;
}

void BLEData()
{
  if ((BLESerial.available()) && bluemode == 1)
  {
    state = BLESerial.read();
  }

  if      (state == '0')  vSpeed = 0  ;
  else if (state == '4')  vSpeed = 100;
  else if (state == '6')  vSpeed = 155;
  else if (state == '7')  vSpeed = 180;
  else if (state == '8')  vSpeed = 200;
  else if (state == '9')  vSpeed = 230;
  else if (state == 'q')  vSpeed = 255;
  else if (state == 'F')  forward();
  else if (state == 'B')  backward();
  else if (state == 'L')  left();
  else if (state == 'R')  right();
  else if (state == 'I')  topRight();
  else if (state == 'G')  topLeft();
  else if (state == 'J')  bottomRight();
  else if (state == 'H')  bottomLeft();
  else if (state == 'S')  Stop();
  else if (state == 'V')  MIST = 1;
  else if (state == 'v')  MIST = 0;
  else if (state == 'X')  digitalWrite(fan, HIGH);
  else if (state == 'x')  digitalWrite(fan, LOW);
}

void forward()
{
  motor1.setSpeed(vSpeed);
  motor1.run(FORWARD);
  motor2.setSpeed(vSpeed);
  motor2.run(BACKWARD);
  motor3.setSpeed(vSpeed);
  motor3.run(BACKWARD);
  motor4.setSpeed(vSpeed);
  motor4.run(FORWARD);
}

void backward()
{
  motor1.setSpeed(vSpeed);
  motor1.run(BACKWARD);
  motor2.setSpeed(vSpeed);
  motor2.run(FORWARD);
  motor3.setSpeed(vSpeed);
  motor3.run(FORWARD);
  motor4.setSpeed(vSpeed);
  motor4.run(BACKWARD);
}

void left()
{
  motor1.setSpeed(vSpeed);
  motor1.run(FORWARD);
  motor2.setSpeed(vSpeed);
  motor2.run(FORWARD);
  motor3.setSpeed(vSpeed);
  motor3.run(BACKWARD);
  motor4.setSpeed(vSpeed);
  motor4.run(BACKWARD);
}

void right()
{
  motor1.setSpeed(vSpeed);
  motor1.run(BACKWARD);
  motor2.setSpeed(vSpeed);
  motor2.run(BACKWARD);
  motor3.setSpeed(vSpeed);
  motor3.run(FORWARD);
  motor4.setSpeed(vSpeed);
  motor4.run(FORWARD);
}

void topLeft()
{
  motor1.setSpeed(vSpeed);
  motor1.run(FORWARD);
  motor2.setSpeed(0);
  motor2.run(RELEASE);
  motor3.setSpeed(vSpeed);
  motor3.run(BACKWARD);
  motor4.setSpeed(0);
  motor4.run(RELEASE);
}

void topRight()
{
  motor1.setSpeed(0);
  motor1.run(RELEASE);
  motor2.setSpeed(vSpeed);
  motor2.run(BACKWARD);
  motor3.setSpeed(0);
  motor3.run(RELEASE);
  motor4.setSpeed(vSpeed);
  motor4.run(FORWARD);
}

void bottomLeft()
{
  motor1.setSpeed(0);
  motor1.run(RELEASE);
  motor2.setSpeed(vSpeed);
  motor2.run(FORWARD);
  motor3.setSpeed(0);
  motor3.run(RELEASE);
  motor4.setSpeed(vSpeed);
  motor4.run(BACKWARD);
}

void bottomRight()
{
  motor1.setSpeed(vSpeed);
  motor1.run(BACKWARD);
  motor2.setSpeed(0);
  motor2.run(RELEASE);
  motor3.setSpeed(vSpeed);
  motor3.run(FORWARD);
  motor4.setSpeed(0);
  motor4.run(RELEASE);
}

void rotateCCW()
{
  motor1.setSpeed(vSpeed);
  motor1.run(FORWARD);
  motor2.setSpeed(vSpeed);
  motor2.run(FORWARD);
  motor3.setSpeed(vSpeed);
  motor3.run(FORWARD);
  motor4.setSpeed(vSpeed);
  motor4.run(FORWARD);
}

void rotateCW()
{
  motor1.setSpeed(vSpeed);
  motor1.run(BACKWARD);
  motor2.setSpeed(vSpeed);
  motor2.run(BACKWARD);
  motor3.setSpeed(vSpeed);
  motor3.run(BACKWARD);
  motor4.setSpeed(vSpeed);
  motor4.run(BACKWARD);
}

void Stop() {
  motor1.setSpeed(0);
  motor1.run(RELEASE);
  motor2.setSpeed(0);
  motor2.run(RELEASE);
  motor3.setSpeed(0);
  motor3.run(RELEASE);
  motor4.setSpeed(0);
  motor4.run(RELEASE);
}

float MQ135Calibration(int mq_pin)
{
  int i;
  float RS_AIR_val=0,r0;
  for (i=0; i<CALIBARAION_SAMPLE_TIMES; i++)
  {
    RS_AIR_val += MQ135ResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  RS_AIR_val = RS_AIR_val/CALIBARAION_SAMPLE_TIMES;
  r0 = RS_AIR_val/RO_CLEAN_AIR_FACTOR_MQ135;
  return r0; 
}

float MQ135ResistanceCalculation(int raw_adc)
{
  return (((float)RL_VALUE_MQ135*(1023-raw_adc)/raw_adc));
}

float MQ135Read(int mq_pin)
{
  int i;
  float rs=0;
  for (i=0;i<READ_SAMPLE_TIMES;i++)
  {
    rs += MQ135ResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }
  rs = rs/READ_SAMPLE_TIMES;
  return rs;  
}

int MQ135GetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if (gas_id == GAS_CARBON_DIOXIDE)         return (400 + pow(10,((-2.890*(log10(rs_ro_ratio))) + 2.055)));
  else if (gas_id == GAS_CARBON_MONOXIDE)   return (pow(10,(1.457*pow((log10(rs_ro_ratio)), 2) - 4.725*(log10(rs_ro_ratio)) + 2.855)));
}

float MQ2Calibration(int mq_pin)
{
  int i;
  float RS_AIR_val=0,r0;
  for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++)
  {
    RS_AIR_val += MQ2ResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  RS_AIR_val = RS_AIR_val/CALIBARAION_SAMPLE_TIMES;
  r0 = RS_AIR_val/RO_CLEAN_AIR_FACTOR_MQ2;
  return r0; 
}

float MQ2ResistanceCalculation(int raw_adc)
{
  return (((float)RL_VALUE_MQ2*(1023-raw_adc)/raw_adc));
}

float MQ2Read(int mq_pin)
{
  int i;
  float rs=0;
  for (i=0;i<READ_SAMPLE_TIMES;i++)
  {
    rs += MQ2ResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }
  rs = rs/READ_SAMPLE_TIMES;
  return rs;  
}

int MQ2GetGasPercentage(float rs_ro_ratio, int gas_id)
{
  return (pow(10,(-0.976*pow((log10(rs_ro_ratio)), 2) - 2.018*(log10(rs_ro_ratio)) + 3.617)));
}

float MQ7ResistanceCalculation(int raw_adc)
{
  return (((float)RL_VALUE_MQ7*(1023-raw_adc)/raw_adc));
}

float MQ7Calibration(int mq_pin)
{
  int i;
  float RS_AIR_val=0,r0;
  for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++)
  {
    RS_AIR_val += MQ7ResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  RS_AIR_val = RS_AIR_val/CALIBARAION_SAMPLE_TIMES;
  r0 = RS_AIR_val/RO_CLEAN_AIR_FACTOR_MQ7;
  return r0; 
}

float MQ7Read(int mq_pin)
{
  int i;
  float rs=0;
  for (i=0;i<READ_SAMPLE_TIMES;i++)
  {
    rs += MQ7ResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }
  rs = rs/READ_SAMPLE_TIMES;
  return rs;  
}

int MQ7GetGasPercentage(float rs_ro_ratio, int gas_id)
{
  return (pow(10,(61.00*pow((log10(rs_ro_ratio)), 3) - 148.7*pow((log10(rs_ro_ratio)), 2) + 112.6*(log10(rs_ro_ratio)) - 23.30)));
}

void sendDataToESP()
{
  DataJson(String(temperature), String(humidity), String(co2), String(co), String(smoke), String(lpg), String(dust));
  ESPSerial.println(ChuoiSendESPJson);
  //ChuoiSendESPJson = "";
}

void DataJson(String s1, String s2, String s3, String s4, String s5, String s6, String s7)
{
  ChuoiSendESPJson = "{\"Temp\":\"" + s1 + "\"," + "\"Humi\":\"" + s2 + "\","
                    +"\"CO2\":\"" + s3 + "\"," + "\"CO\":\"" + s4 + "\","
                    +"\"Smoke\":\"" + s5 + "\"," + "\"LPG\":\"" + s6 + "\"Dust\":\"" + s7 + "\"}";
}

void receiveDataFromESP()
{
  if (ESPSerial.available())
  {
    ChuoiReceiveESPJson = ESPSerial.readStringUntil('\n');
    JsonSeparate(ChuoiReceiveESPJson);
  }
}

void JsonSeparate(String input)
{
  int s11, s12, s21, s22, s31, s32;
  s11  = input.indexOf("\"Fan&Mist\":");
  s21  = input.indexOf("\"BlueMode\":");
  s31  = input.indexOf("\"AutoMode\":");

  s12  = input.indexOf("\",\"BlueMode\":");
  s22  = input.indexOf("\",\"AutoMode\":");
  s32  = input.indexOf("\"}");

  String s1 = input.substring(s11 + 12, s12);
  String s2 = input.substring(s21 + 12, s22);
  String s3 = input.substring(s31 + 12, s32);

  fanmist  = s1.toInt();
  bluemode = s2.toInt();
  automode = s3.toInt();
}

int leftSee()
{
  servo.write(150);
  delay(800);
  int l = calculateDistance();
  servo.write(90);
  return l;
}

int rightSee()
{
  servo.write(30);
  delay(800);
  int r = calculateDistance();
  servo.write(90);
  return r;
}

void avoid()
{
    distance = calculateDistance();
    if (distance <= 20)
    {
      Stop();
      backward();
      delay(100);
      Stop();
      Left = leftSee();
      delay(800);
      Right = rightSee();
      if (Left < Right)
      {
        rotateCW();
        delay(500);
        Stop();
        delay(200);
      }
      else if (Left > Right)
      {
        rotateCCW();
        delay(500);
        Stop();
        delay(200);
      }
    }
    else forward();
    preauto = automode;
}