#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_ADS1X15.h>
#include <OneButton.h>
#include <EEPROM.h>
#include <Bounce2.h>
#include <PID_v1.h> 

#define CLK 10
#define DT  9
#define SW  8
#define OLED_RESET 4
#define valvePin 3

float result;
float result_max = 0;
float mv = 0.0;
float calibrationv = 0;
float multiplier = 0.0078125;
int setpointHigh = 40;
int setpointLow = 19;
int millivolts, currentmv;

float elapsedTime, now_time, prev_time;        //Variables for time control
//float refresh_rate = 200;                   //PID loop time in ms
float now_pid_error, prev_pid_error;

double setpoint, Input, Output;

double Kp=2, Ki=1, Kd=1;
PID myPID(&Input, &Output, &setpoint, Kp, Ki, Kd, DIRECT);
Adafruit_ADS1115 ads;
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET);
OneButton btn(SW, true , true);

const unsigned long interval = 10000; // 10000 µs = 100 Hz

template <uint8_t K, class uint_t = uint16_t>
class EMA {
  public:
    /// Update the filter with the given input and return the filtered output.
    uint_t operator()(uint_t input) {
        state += input;
        uint_t output = (state + half) >> K;
        state -= output;
        return output;
    }

    static_assert(
        uint_t(0) < uint_t(-1),  // Check that `uint_t` is an unsigned type
        "The `uint_t` type should be an unsigned integer, otherwise, "
        "the division using bit shifts is invalid.");

    /// Fixed point representation of one half, used for rounding.
    constexpr static uint_t half = 1 << (K - 1);

  private:
    uint_t state = 0;
};

void EEPROMWriteInt(int p_address, int p_value) {
  byte lowByte = ((p_value >> 0) & 0xFF);
  byte highByte = ((p_value >> 8) & 0xFF);
  EEPROM.write(p_address, lowByte);
  EEPROM.write(p_address + 1, highByte);
}

unsigned int EEPROMReadInt(int p_address) {
  byte lowByte = EEPROM.read(p_address);
  byte highByte = EEPROM.read(p_address + 1);
  return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
}

int calibrate(int x)
{
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setCursor(30,30);  
  display.setTextSize(1);
  display.print(F("Calibrating"));
  display.display();
  static EMA<4> avg;
  int size = 1000;
  int result = 0;
  for (int i = 0 ; i < size ; i++)
  {
    int reading = ads.readADC_Differential_0_1();
    result = avg(reading);
  }
  result = abs(result);
  EEPROMWriteInt(x, result);
  delay(1000);
  return (result);
}

void dispSP()
  {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println("Set Point");
  display.setTextSize(2);
  display.println("----------");
  display.setTextSize(4);
  display.print(setpoint,1);
  display.println(F("%"));
  display.display();
  }


void setpointMenu()
{
  Bounce bounce = Bounce();
  bounce.attach(SW);
  bounce.interval(20);
  dispSP();
  bool exit = false;
  int currentCLK;
  int previousCLK = digitalRead(CLK);
  while (exit != true) {
    bounce.update();
    if (bounce.changed() ) 
    {
      int debounceInput = bounce.read();
      if (debounceInput == LOW) 
      {
        exit = true;
      }
    }
    currentCLK = digitalRead(CLK);
    if(currentCLK != previousCLK)
    {       
      if(digitalRead(DT) != currentCLK)
      { 
        setpoint+=0.5;
        if (setpoint > setpointHigh) {
          setpoint = setpointHigh;
        }
      }
      else
      {
        setpoint-=0.5;
        if (setpoint < setpointLow) {
          setpoint = setpointLow;
        }
      }
      dispSP();  
    }
    previousCLK = currentCLK;
  }
}

void doubleClick() {
  setpointMenu();
}

void longPress(){
  calibrationv = calibrate(0);
  display.setTextColor(WHITE);
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println("ADC Reading"); 
  display.setTextSize(2);
  display.print("----------");
  display.setTextSize(2);
  display.setCursor(35,30);
  display.print(calibrationv);
  display.display();

  delay(1000);
}

void setup() {
  Serial.begin(115200);
  pinMode(valvePin, OUTPUT);
  ads.setGain(GAIN_SIXTEEN);
  ads.begin();
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  btn.attachDoubleClick(doubleClick);
  btn.attachLongPressStart(longPress);
  calibrationv = EEPROMReadInt(0);
  if (calibrationv < 100) {
    calibrationv = calibrate(0);
  }
  Input = ads.readADC_Differential_0_1();
  setpoint = 19.0;  //make sure the valve starts closed. 
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);


}

void loop() {
  btn.tick();
  static EMA<4> filter;
  static unsigned long prevMicros = micros() - interval;
  if (micros() - prevMicros >= interval) 
  {
    millivolts = ads.readADC_Differential_0_1();
    currentmv = abs(filter(millivolts));
    result = (currentmv/calibrationv) * 20.9;
    if (result > 99.9) result = 99.9;
    float mv = currentmv * multiplier;
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setCursor(4,17);
    if (mv < 0.02 || result <= 0) {
      display.setTextSize(2);
      display.println(F("Sensor"));
      display.print(F("Error!"));
    } else {
    display.setTextSize(4);
    display.print(result,1);
    display.println(F("%"));

    if (result >= result_max) {
      result_max = result;
      }
    }
    Input = result;
    myPID.Compute();
    analogWrite(valvePin, Output);
    Serial.print("setpoint = "); Serial.print(setpoint);
    Serial.print("\t Input = "); Serial.print(Input);
    Serial.print("\t Output PWM = "); Serial.println(Output);
    display.display();
    prevMicros += interval;
  }
}