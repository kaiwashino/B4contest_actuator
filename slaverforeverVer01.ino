#include <Servo.h>
Servo myServo;
int l = 0;
const int sspin = 5;
const int pel_pin = 8;
const int SHT_SCK = 2;
const int SHT_DAT = 3;

const byte SHT_MeasureTemperature  = B00011;
const byte SHT_MeasureHumidity     = B00101;
const byte SHT_ReadStatusRegistor  = B00111;
const byte SHT_WriteStatusRegistor = B00110;
const byte SHT_SoftReset           = B11110;

struct SHT_result {
  unsigned short res;
  byte crc;
  byte err;
};

inline void SHT_clk()
{
  digitalWrite(SHT_SCK, HIGH);
  digitalWrite(SHT_SCK, LOW);
}

struct SHT_result SHT_read(byte cmd)
{
  struct SHT_result ret;

  ret.err = 0;
  digitalWrite(SHT_DAT, HIGH);
  pinMode(SHT_DAT, OUTPUT);

  // send START
  digitalWrite(SHT_SCK, HIGH);
  digitalWrite(SHT_DAT, LOW);
  digitalWrite(SHT_SCK, LOW);
  digitalWrite(SHT_SCK, HIGH);
  digitalWrite(SHT_DAT, HIGH);
  digitalWrite(SHT_SCK, LOW);

  // send address = 000
  digitalWrite(SHT_DAT, LOW);
  SHT_clk();
  SHT_clk();
  SHT_clk();

  for (int i = 0; i < 5; i++) {
    if ((cmd & B10000) != 0)
      digitalWrite(SHT_DAT, HIGH);
    else
      digitalWrite(SHT_DAT, LOW);
    SHT_clk();
    cmd <<= 1;
  }
  pinMode(SHT_DAT, INPUT);
  digitalWrite(SHT_DAT, HIGH);
  // wait for ACK
  unsigned long cur = millis();
  while (digitalRead(SHT_DAT) == HIGH) {
    if ((millis() - cur) > 1000) {
      ret.err = 1;
      return ret;
    }
  }
  // send clk
  SHT_clk();

  // wait for data to come
  cur = millis();
  while (digitalRead(SHT_DAT) == HIGH) {
    if ((millis() - cur) > 1000) {
      ret.err = 1;
      return ret;
    }
  }

  ret.res = 0;
  for (int j = 0; j < 2; j++) {
    for (int i = 0; i < 8; i++) {
      ret.res <<= 1;
      if (digitalRead(SHT_DAT) == HIGH)
        ret.res |= 1;
      SHT_clk();
    }
    // Send ACK
    pinMode(SHT_DAT, OUTPUT);
    digitalWrite(SHT_DAT, LOW);
    SHT_clk();
    pinMode(SHT_DAT, INPUT);
    digitalWrite(SHT_DAT, HIGH);
  }

  ret.crc = 0;
  for (int i = 0; i < 8; i++) {
    ret.crc <<= 1;
    if (digitalRead(SHT_DAT) == HIGH)
      ret.crc |= 1;
    SHT_clk();
  }
  // Skip ACK
  SHT_clk();

  // CRC をチェックした方がいい

  return ret;
}

void setup() {
  myServo.attach(9, 1050, 2000);
  myServo.writeMicroseconds(1600);
  pinMode(pel_pin, OUTPUT);
  digitalWrite(pel_pin, LOW);
  digitalWrite(SHT_SCK, LOW);
  digitalWrite(SHT_DAT, HIGH); // pull-up
  pinMode(SHT_SCK, OUTPUT);
  pinMode(SHT_DAT, INPUT);

  Serial.begin(9600);
}

void loop() {
  struct SHT_result ret;
  float h, t;
  l++;

  ret = SHT_read(SHT_MeasureHumidity);
  if (ret.err != 0)
    return;
  h = -4.0 + 0.0405 * ret.res - 2.8 * 0.000001 * ret.res * ret.res;
  //Serial.print(ret.res >> 8, HEX);
  //Serial.print(ret.res & 0xff, HEX);
  //Serial.print(" ");
  //Serial.print(ret.crc, HEX);
  //Serial.print(" ");

  ret = SHT_read(SHT_MeasureTemperature);
  if (ret.err != 0)
    return;
  t = -40.00 + 0.01 * ret.res;
  // Serial.print(ret.res >> 8, HEX);
  //Serial.print(ret.res & 0xff, HEX);
  //Serial.print(" ");
  //Serial.print(ret.crc, HEX);
  //Serial.print(" ");

  if (Serial.available() > 0) {
    digitalWrite(13, HIGH);
    switch (l % 2) {
      case 0:
        myServo.writeMicroseconds(2000);
        break;
      case 1:
        myServo.writeMicroseconds(1600);
        break;
      default:
        break;
    }
    digitalWrite(pel_pin, HIGH);
    analogWrite(sspin, 75);

  }


  if (Serial.available() == 0) {
    digitalWrite(pel_pin, LOW);
    analogWrite(sspin, 0);
    myServo.writeMicroseconds(1600);
  }

  //  Serial.print(h);
  //  Serial.print("% ");
  //  Serial.print(t);
  //  Serial.print("C");
  //  Serial.print(" ");
  //  Serial.print(l);
  //  Serial.print(" ");
  //  Serial.print(m);
  //  Serial.println("");
  Serial.read();
  serialCommunication(h, t);
  delay(1000);
}

void serialCommunication(float Humid, float Temp) {
  String humid = String(Humid);
  String temp = String(Temp);
  Serial.print(humid);
  Serial.print(",");
  Serial.print(temp);
  Serial.print('\n');
}
