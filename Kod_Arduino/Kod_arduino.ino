#include <PWM.h>
#include <PID_v1.h>
#include <ReadFilter.h>

#define ZAPWM1 9
#define ZAPWM2 10
#define ANA0 3
#define DIG1 12
int32_t frq = 20;
double accuracy = 1.0;
double currentPos = 0;
double setpoint = 0;
double uchyb = 0;
int maxSetFRQ [] = {32,32,31,29,24}; // Valid frequencies are 2, 5, 10, 20 and 50 Hz.
int minSetFRQ [] = {1,1,2,3,6};
double Kp = 0.5;
double Ki = 0.1;
double Kd = 0.01;
double outPWM = 0;
bool pozwolenie = false;

PID pid(&currentPos, &outPWM, &setpoint, Kp, Ki, Kd, DIRECT);
ReadFilter rf(20,0,1023,false,1,100); // 2 ms measure


unsigned long previousMillis = 0;
const unsigned long interval = 20; // ms

void setup() {
  Serial.begin(9600);
  pinMode(ZAPWM1, OUTPUT);
  pinMode(ZAPWM2, OUTPUT);
  pinMode(ANA0, INPUT);
  pinMode(DIG1,INPUT);
  InitTimersSafe();
  SetPinFrequencySafe(ZAPWM1, frq);
  SetPinFrequencySafe(ZAPWM2, frq);
  pid.SetSampleTime(interval/2);
  pid.SetOutputLimits((double)minSetFRQ[3], (double)maxSetFRQ[3]);
  pid.SetMode(AUTOMATIC);
  updatePos();
  setpoint = currentPos;
  pwmWrite(ZAPWM1, 0);
  pwmWrite(ZAPWM2, 0);
  pozwolenie = false;
}

void loop() {
  handleSerialInput();
  handlePinInput();
  unsigned long currentMillis = millis();
  if ((currentMillis - previousMillis) >= interval) {
    updatePos();
    uchyb = setpoint - currentPos;
  if (((uchyb > accuracy) or (uchyb < (accuracy*(-1.0)))) and pozwolenie) {
    int outPWM2 = 0;
    if (uchyb < 0) {
      pid.SetControllerDirection(REVERSE);
      pid.Compute();
      outPWM2 = (int)((outPWM/100.0) * 255.0);
      pwmWrite(ZAPWM1, outPWM2);
      pwmWrite(ZAPWM2, 0);
    } else if (uchyb > 0) {
      pid.SetControllerDirection(DIRECT);
      pid.Compute();
      outPWM2 = (int)((outPWM/100.0) * 255.0);
      pwmWrite(ZAPWM2, outPWM2);
      pwmWrite(ZAPWM1, 0);
    }
    
  } else {
    pwmWrite(ZAPWM1, 0);
    pwmWrite(ZAPWM2, 0);
  }
    if (pozwolenie) {
    Serial.print(millis());
    Serial.print(";");
    Serial.print(currentPos);
    Serial.print(";");
    Serial.print(setpoint);
    Serial.print(";");
    Serial.println(uchyb);
    }

    previousMillis = currentMillis;
  }
}

void handlePinInput() {
  if (digitalRead(DIG1)) {
    pwmWrite(ZAPWM1, 0);
    pwmWrite(ZAPWM2, 0);
    delay(1000);
    updatePos();
    setpoint = currentPos;
    pozwolenie = false;
    Serial.println("stop");
  }
}

void updatePos() {
  unsigned long currentMillis = millis();
  int raw = rf.avg(ANA0);
  if (raw < 20) raw = 20;
  if (raw > 1003) raw = 1003;
  currentPos = (177.0 / 983.0) * ((double)raw - 20.0);
}

void handleSerialInput() {
  static String input = "";

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (input.length()) {
        processCommand(input);
        input = "";
      }
    } else {
      input += c;
    }
  }
} 

void processCommand(String cmd) {

  cmd.trim();

  if ((cmd.startsWith("P:")) or (cmd.startsWith("p:"))) {
    String valStr = cmd.substring(2);
    if (isNumber(valStr)){
      double val = valStr.toDouble();
      Kp = val;
      pid.SetTunings(Kp, Ki, Kd);
      Serial.print("Set Kp to ");
      Serial.println(pid.GetKp());

    }else {Serial.println("Invalid number. Remember to use '.' not ','");}


  } else if ((cmd.startsWith("I:")) or (cmd.startsWith("i:"))) {
    String valStr = cmd.substring(2);
    if (isNumber(valStr)){
      double val = valStr.toDouble();
      Ki = val;
      pid.SetTunings(Kp, Ki, Kd);
      Serial.print("Set Ki to ");
      Serial.println(pid.GetKi());

    }else {Serial.println("Invalid number. Remember to use '.' not ','");}


  } else if ((cmd.startsWith("D:")) or (cmd.startsWith("d:"))) {
    String valStr = cmd.substring(2);
    if (isNumber(valStr)){
      double val = valStr.toDouble();
      Kd = val;
      pid.SetTunings(Kp, Ki, Kd);
      Serial.print("Set Kd to ");
      Serial.println(pid.GetKd());

    }else {Serial.println("Invalid number. Remember to use '.' not ','");}


  } else if (cmd.startsWith("FRQ:")) {
    String valStr = cmd.substring(4);
    if (isNumber(valStr)){
      double val = valStr.toInt();
      if ((val == 10) or (val == 20) or (val == 50)) {
        int32_t val2 = (int)val;
        frq = val2;
        SetPinFrequencySafe(ZAPWM1, frq);
        SetPinFrequencySafe(ZAPWM2, frq);
        switch (frq){
          //case 2:
            //pid.SetOutputLimits((double)minSetFRQ[0], (double)maxSetFRQ[0]);
           // break;
          //case 5:
            //pid.SetOutputLimits((double)minSetFRQ[1], (double)maxSetFRQ[1]);
            //break;
          case 10:
            pid.SetOutputLimits((double)minSetFRQ[2], (double)maxSetFRQ[2]);
            break;
          case 20:
            pid.SetOutputLimits((double)minSetFRQ[3], (double)maxSetFRQ[3]);
            break;
          case 50:
            pid.SetOutputLimits((double)minSetFRQ[4], (double)maxSetFRQ[4]);
            break;
        }
        Serial.print("Set PWM frequency to ");
        Serial.print(frq); 
        Serial.println(" Hz");

      } else {
        Serial.println("Invalid frequency. Valid frequencies are 10, 20 and 50 Hz.");
      }

    }
    else {Serial.println("Invalid number. Remember to use '.' not ','");}
    

  } else if ((cmd.startsWith("Setpoint:")) or (cmd.startsWith("setpoint:"))) {
    String valStr = cmd.substring(9);
    if (isNumber(valStr)){
      double val = valStr.toDouble();
      if ((val >= 30.0) and (val <= 147.0)){
        setpoint = val;
        Serial.print("Set Setpoint to ");
        Serial.print(setpoint); 
        Serial.println(" mm");
      }else {Serial.println("Invalid setpoint. Valid setpoint is within the range 30-147 mm");}
      
    }else {Serial.println("Invalid number. Remember to use '.' not ','");}


  } else if (cmd == "stop") {
    pwmWrite(ZAPWM1, 0);
    pwmWrite(ZAPWM2, 0);
    delay(1000);
    updatePos();
    setpoint = currentPos;
    pozwolenie = false;
    Serial.println("stop");

  } else if (cmd == "start") {
    pozwolenie = true;
    Serial.println("start");

  } else if ((cmd.startsWith("Acc:")) or (cmd.startsWith("acc:"))) {
    String valStr = cmd.substring(4);
    if (isNumber(valStr)){
      double val = valStr.toDouble();
      accuracy = val;
      Serial.print("Set accuracy to ");
      Serial.print(accuracy); 
      Serial.println(" mm");
    }else {Serial.println("Invalid number. Remember to use '.' not ','");}

  } else if (cmd == "default") {
    pwmWrite(ZAPWM1, 0);
    pwmWrite(ZAPWM2, 0);
    delay(500);
    updatePos();
    setpoint = currentPos;
    pozwolenie = false;

    frq = 20;
    accuracy = 1.0;
    Kp = 0.5;
    Ki = 0.1;
    Kd = 0.01;

    pid.SetTunings(Kp, Ki, Kd);
    SetPinFrequencySafe(ZAPWM1, frq);
    SetPinFrequencySafe(ZAPWM2, frq);
    pid.SetOutputLimits((double)minSetFRQ[3], (double)maxSetFRQ[3]);

    Serial.print("Set Kp to ");
    Serial.println(pid.GetKp());
    Serial.print("Set Ki to ");
    Serial.println(pid.GetKi());
    Serial.print("Set Kd to ");
    Serial.println(pid.GetKd());
    Serial.print("Set PWM frequency to ");
    Serial.print(frq); 
    Serial.println(" Hz");
    Serial.print("Set accuracy to ");
    Serial.print(accuracy); 
    Serial.println(" mm");
    Serial.println("stop");

  } else if (cmd == "sleep") {

    frq = 20;
    accuracy = 1.0;
    Kp = 0.5;
    Ki = 0.1;
    Kd = 0.01;

    pid.SetTunings(Kp, Ki, Kd);
    SetPinFrequencySafe(ZAPWM1, frq);
    SetPinFrequencySafe(ZAPWM2, frq);
    pid.SetOutputLimits((double)minSetFRQ[3], (double)maxSetFRQ[3]);

    Serial.print("Set Kp to ");
    Serial.println(pid.GetKp());
    Serial.print("Set Ki to ");
    Serial.println(pid.GetKi());
    Serial.print("Set Kd to ");
    Serial.println(pid.GetKd());
    Serial.print("Set PWM frequency to ");
    Serial.print(frq); 
    Serial.println(" Hz");
    Serial.print("Set accuracy to ");
    Serial.print(accuracy); 
    Serial.println(" mm");

    setpoint = 80.0;
    pozwolenie = true;

  } else {
    Serial.println("Invalid command. Valid commands are: 'P:', 'I:', 'D:', 'FRQ:', 'stop', 'sleep', 'default' and 'setpoint:' ");
  }

}

bool isNumber (String str){
  int numOfDots = 0;
  bool exception = false;
  for (int i = 0; i < str.length() ; i++){
    char c = str[i];
    if (c == '.'){numOfDots++;}
    else if (!isDigit(c)){exception = true;}
  }
  if ((exception) or numOfDots > 1){return false;}
  else {return true;}
}


