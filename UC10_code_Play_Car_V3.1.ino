//Empower Project PLAY car software 3.1, May 10, 2024
    //added pinMode input pullups for analog input pins to prevent adjacent
    //channels from interfering with each other when left floating (i.e. unplugged joystick)
    
#include "CytronMotorDriver.h"

#define LEFT    0
#define RIGHT   1

#include <SoftwareSerial.h>
SoftwareSerial SwSerial(10,11);
//#include <BlynkSimpleSerialBLE.h>

char auth[]="nEG7Gr8j17dn-b4xOREaZCdBTzINk2HG";
bool connected = false;
// Configure the motor driver.

CytronMD motorL(PWM_DIR, 5, 4);
CytronMD motorR(PWM_DIR, 6, 7);

// Wiring Connections.
const int joystickPinX = A0;                      // Analog input pin X (joystick) - Left/Right
const int joystickPinY = A1;                      // Analog input pin Y (joystick) - Fwd/Rev
const int buttonPinX1 = 10;                       //Digital button input pin X1 (buttons) - Left
const int buttonPinX2 = 11;                       //Digital button input pin X2 (buttons) - Right 
const int buttonPinY1 = 13;                       //Digital button input pin Y1 (buttons) - Forward (Changed from 12 to 13)
const int buttonPinY2 = 12;                       //Digital button input pin Y2 (buttons) - Reverse (Changed from 13 to 12)
const int ModeSwitch = 3;                         //Digital Switch input pin
const int analogInPinSpeed = A4;                  //analog speed control (potentiometer speed dial)
const int keySwitch = 2;                          //key switch input
const int brakeLeft = 1;                          //left motor brake relay output
const int brakeRight = 0;                         //right motor brake relay output
const int z= 0.4600;                              //exponential adjustment factor
const int a= 150;                                 //dead zone constant

const int maxaccel= 1;                           //maximum acceleration value. changed it from 10 to 1 in order to have a smoother acceleration/ deacceleration
unsigned long milliT = 200;                       //Period of millis() function based action

bool updateMotors = false;                        // used to clock the motor speed setting
int sensorValueX = 0;                             // value from pot x
int sensorValueY = 0;                             // value from pot y
int sensorValueX2 = 0;                            //value from X2 buttons
int sensorValueY2= 0;                             //value from Y2 buttons
int sensorSpeed= 0;                               //value for speed
int outputValueX = 0;                             // value output to the PWM (analog out) X
int outputValueY = 0;                             // value output to the pwm analog out Y
int newleftmotorsignal = 0;                       // left variable value
int newrightmotorsignal = 0;                      // right variable value
int rampleft = 0;                                 // value used to hold target speed for left motor
int rampright = 0;                                // value used to hold target speed for right motor
unsigned long milliOld = 0;                       // last read value of millis()
int oldSpeedLeft= 0;                              // deadzone compensated speed left
int oldSpeedRight= 0;                             // deadzone compensated speed right
int ValueX= 0;                                    // x value compared to reference
int ValueY = 0;                                   // y value compared to reference

bool osfall = false;                              //one-shot trigger bit
bool osfset = false;                              //one-shot reference bit
unsigned long brakeTime = 0;                      //time that motor signal stopped

int Speed =20;                                    //Speed adjustment factor
int TruSpeed = 0;

String inString = " ";


void setup(){
  Serial.begin(9600);
  pinMode(buttonPinX1,INPUT_PULLUP);
  pinMode(buttonPinX2,INPUT_PULLUP);
  pinMode(buttonPinY1,INPUT_PULLUP);
  pinMode(buttonPinY2,INPUT_PULLUP);
  pinMode(ModeSwitch,INPUT_PULLUP);
  pinMode(keySwitch, INPUT_PULLUP);
  pinMode(brakeLeft, OUTPUT);
  pinMode(brakeRight, OUTPUT);

  
  //Blynk.begin(SwSerial, auth);
  TCCR0A = _BV(COM0A1)| _BV(COM0B1)| _BV(WGM01) | _BV(WGM00);
  TCCR0B = _BV(CS00);
  motorL.setSpeed(0);                             // start motors at 0 speed
  motorR.setSpeed(0);
  milliOld = millis();                            // init milliOld so the first execution isn't missed
  
  digitalWrite(brakeLeft, LOW);                   //turn on left motor brake at start
  digitalWrite(brakeRight, LOW);                  //turn on right motor brake at start

  //Pause program if user inputs are active when power is turned on
  //to prevent accidental motion when it isn't expected
  //==============================================
  int joyX;                                       //x axis joystick temp
  int joyY;                                       //y axis joystick temp
  bool mode;                                      //mode switch position
  bool butX1;                                     //button X1
  bool butX2;                                     //button X2
  bool butY1;                                     //button Y1
  bool butY2;                                     //button Y2
  
  bool state;                                     //Low = controls are neutral, High = controls are active

 
  do {
    joyX = analogRead(joystickPinX);              //read joystick X postion between 0 and 1023
    joyY = analogRead(joystickPinY);              //read joystick Y position between 0 and 1023
    mode = digitalRead(ModeSwitch);               //read mode switch position - High = button, Low = joystick
    butX1 = digitalRead(buttonPinX1);             //read button X1 - Low = press
    butX2 = digitalRead(buttonPinX2);             //read button X2 - Low = press
    butY1 = digitalRead(buttonPinY1);             //read button Y1 - Low = press
    butY2 = digitalRead(buttonPinY2);             //read button Y2 - Low = press

    //=========================== Uncomment to print initial button status
    //Serial.print("mode=");
    //Serial.print(mode);
    //Serial.print("   ");
    //Serial.print("buttons=");
    //Serial.print(butX1);
    //Serial.print(butX2);
    //Serial.print(butY1);
    //Serial.println(butY2);
    //===========================
    
    if ( (mode == HIGH) && !(butX1 && butX2 && butY1 && butY2)){      //mode = button control AND any button is pressed
      state = true;                                                  //one or more buttons have been pressed
    }
    else if ( (mode == LOW) && ( (joyX < (512-a)) || (joyX > (512+a)) || (joyY < (512-a)) || (joyY > (512+a)) ) ){     //mode = joystick control AND one or more joystick signal is not in the center
      state = true;
    }
    else {                                          //all controls are in neutral
     state = false;
    }

  } while (state == true);                        //loop while controls are active

 
  //==============================================
    
}
void loop(){

  pinMode (A0, INPUT_PULLUP);                     //adds pull up resistor to x joystick to prevent channel ghosting
  pinMode (A1, INPUT_PULLUP);                     //adds pull up resistor to y joystick to prevent channel ghosting
  pinMode (A4, INPUT_PULLUP);                     //adds pull up resistor to speed signal to prevent channel ghosting
  
  sensorSpeed = analogRead(analogInPinSpeed);     //read speed value from speed dial
  Speed = map(sensorSpeed,0, 1023, 220, 360);     //scale speed dial value to 1 to 10 scale. ( Changed lower values from 260 to 200)
  TruSpeed = Speed - a;                           //find the true value of the speed
  
  //key switch lockout
  /// key switch lockout
  while (digitalRead(keySwitch)){                 //pauses program in loop until key is replaced
    motorL.setSpeed(0);                           //stop left motor
    motorR.setSpeed(0);                           //stop right motor
    digitalWrite(brakeLeft, LOW);                 //activate left brake
    digitalWrite(brakeRight, LOW);                //activate right brake
  }

  // button selection
  int ButtonMode = digitalRead(ModeSwitch);       //Mode select: High = buttons, Low = joystick
  
  outputValueX = getOutputValue(joystickPinX, buttonPinX1, buttonPinX2, ButtonMode, true, Speed); // get X inputs and apply voltage compensation
  outputValueY = getOutputValue(joystickPinY, buttonPinY1, buttonPinY2, ButtonMode, true, Speed); // get Y inputs and apply voltage compensation

  //=============================== Uncomment to see speed pot and joystick signal values
  //Serial.print("speed= ");
  //Serial.print(sensorSpeed);
  //Serial.print("   ");
  //Serial.print("joy x = ");
  //Serial.print(outputValueX);
  //Serial.print("   ");
  //Serial.print("joy y= ");
  //Serial.println(outputValueY);
  //================================

  // Limits within our parameters. selecting direction
  newleftmotorsignal= outputValueX - outputValueY;                          //switched sign from + to -, to fix Fwd/Rev swapped issue
  newleftmotorsignal = constrain(newleftmotorsignal, -Speed, Speed);
  newrightmotorsignal= outputValueX + outputValueY;                         //switched sign from - to +, to fix Fwd/Rev swapped issue
  newrightmotorsignal = constrain (newrightmotorsignal, -Speed, Speed);
  
  // Adjustment for deadzone and only using values in the middle of the motor voltage (80%)
  if (newleftmotorsignal > ((0.8*TruSpeed) + a) && newrightmotorsignal > ((0.8*TruSpeed) + a)){
  }
  else if (newleftmotorsignal < ((-0.8*TruSpeed) + a) && newrightmotorsignal < ((-0.8*TruSpeed) + a)){
  }
  else {
    newrightmotorsignal = constrain(newrightmotorsignal, (-0.8*TruSpeed) - a, (0.8*TruSpeed) + a);
    newleftmotorsignal = constrain(newleftmotorsignal, (-0.8*TruSpeed) - a, (0.8*TruSpeed) + a);
  }

    // wait 2s after turning the board on
  updateMotors = checkTime(milliOld,milliT);      //sets bit to update motor signals once set time (milliT) has elapsed

  //Serial.print(updateMotors);
  //Serial.print(" ");

  if (updateMotors){
    
    int leftmotorsignal= acceleration(oldSpeedLeft, newleftmotorsignal, maxaccel, a);
    int rightmotorsignal= acceleration(oldSpeedRight, newrightmotorsignal, maxaccel, a);

    //Serial.print(leftmotorsignal);
    //Serial.print(" ");
        
    oldSpeedLeft =  motorSpeed(motorL, a, leftmotorsignal, LEFT, brakeLeft); 
    oldSpeedRight = motorSpeed(motorR, a, rightmotorsignal, RIGHT, brakeRight);
    
   milliOld = millis();                            // this needs to be run at the end of this if statement so it wont execute until the next elapsed period
  }

  //Serial.print(oldSpeedLeft);
  //Serial.println(" ");
} 



//===============================================================================
int getOutputValue(int joystickPin, int buttonPin1, int buttonPin2, int mode, bool vComp, int SpeedScale){
    int joystickRaw;                             // addr for joystick analog read
    int buttonRaw1;                              // addr for button analog read
    int buttonRaw2;                              // addr for button analog read
    int output;                                  // holding register for calculation

    if (mode == HIGH){                           //button mode selected
      buttonRaw1 = digitalRead(buttonPin1);
      buttonRaw2 = digitalRead(buttonPin2);

      if (buttonRaw1 == HIGH && buttonRaw2 == HIGH || buttonRaw1 == LOW && buttonRaw2 == LOW){
        output = 0;
      }
      else if (buttonRaw1 == HIGH && buttonRaw2 == LOW){
        output = SpeedScale;
      }
      else{
        output = -SpeedScale;
      }
      return output;
    }
    else{                                         //joystick mode selected
      joystickRaw = analogRead(joystickPin);
      if (vComp == true) {                        // if voltage compensation is required add values
          output = joystickRaw;
      }
      else {
          output = joystickRaw;
      }

      if (output < 61 || output > 961){           //if signal is less than 0.3V or greater than 4.7V, joystick signal is out of normal range
        output = 512;                             //write neutral joystick position output to prevent unwanted motion during error
      }

      output = map(output, 102, 920, -SpeedScale, SpeedScale);        //joystick input map from 0.5V - 4.5V to -speed - +speed 
      return output;
    }    
}

//===============================================================================
int motorSpeed(CytronMD motor, int deadZone, int inputspeed, bool side, int brake){
    int newspeed;
    newspeed = inputspeed;

    osfall = ( inputspeed <= deadZone && osfset);      //one-shot triggers on positive edge falling signal (motor on -> motor off = true)
    osfset = !(inputspeed <= deadZone);

    if (osfall){
      brakeTime = millis();                   //record the time when motor stops
    }

    if (abs(inputspeed) <= deadZone) {
        motor.setSpeed(0);

        if (checkTime(brakeTime, 1000)){          //if motor has been stopped for at least 1000ms, activate brake
          digitalWrite(brake, LOW);
        }
    }
    else if (inputspeed > deadZone) {            // if speed is positive and side is right XOR speed is negative and side is left
        motor.setSpeed(inputspeed-deadZone);        
        digitalWrite(brake, HIGH);                  //deactivate brake
        }   
    else {                                       // else speed is negative and side is right XOR speed is positive and side is right
        motor.setSpeed(inputspeed+deadZone);
        digitalWrite(brake, HIGH);                  //deactivate brake
    }
    
    return newspeed;
}

//===============================================================================
unsigned long checkTime(unsigned long old, unsigned long deltaGoal){
    unsigned long current;
    bool timesUp;
    current = millis();
    
    // this checks for millis() roll over; this will only happen if arduino is left on for 50 days 
    if (current - old >= deltaGoal) {
        timesUp = true;
    }
    else {
        timesUp = false;
    }
    return timesUp;
}

//===============================================================================
int acceleration(int oldspeed, int signalspeed, int maxChange, int deadzone) {
  int newspeed;
  int currChange;
  bool posDir; // true for positive change and false for change in the negative direction
  
  currChange = signalspeed - oldspeed; //calculate current change in speed
  if(abs(signalspeed)>deadzone && abs(oldspeed)<deadzone) {
    if (signalspeed > 0){
      newspeed= deadzone;}
    else {
      newspeed=-deadzone;}
  }
  else {

  if (currChange >= 0) {
    posDir = true;
  }
  else {
    posDir = false;
  }

  //set newspeed depending on whether currChange exceeds maxChange or not 
   if (abs(currChange) > maxChange) {
     if (posDir){
     newspeed = oldspeed + maxChange; //add by maxChange if posDir = 1
     } else {
       newspeed = oldspeed - maxChange; //subtract by maxChange if posDir = 0
       }
     } else {
       newspeed = signalspeed; //set newspeed to target signal speed if under allowable change
  }}

  return newspeed;
}
