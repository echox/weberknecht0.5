/*********************************************************************
  This is an example for our nRF51822 based Bluefruit LE modules

  Pick one up today in the adafruit shop!

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  MIT license, check LICENSE for more information
  All text above, and the splash screen below must be included in
  any redistribution
*********************************************************************/

#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
#include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"


// ------- servo





#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)

#define SERVO_LEG_REST  375

#define SERVOMID 360
#define SERVO_LEG_MID  375 // center position

#define HEAD_SERVO_INDEX 0

#define LEG_FRONT_LEFT_YAW 1
#define LEG_FRONT_LEFT_PITCH 2

#define LEG_FRONT_RIGHT_YAW 3
#define LEG_FRONT_RIGHT_PITCH 4

#define LEG_BACK_RIGHT_YAW 5
#define LEG_BACK_RIGHT_PITCH 6

#define LEG_BACK_LEFT_YAW 7
#define LEG_BACK_LEFT_PITCH 8

#define BUTTON_FRONT_PIN 4
#define BUTTON_REAR_PIN 5

#define BUTTON_FRONT_INDEX 0
#define BUTTON_REAR_INDEX 1

#define SERVO_COUNT 9

#define NOTE_C4  3830

#define NOTE_D4  3400
#define NOTE_E4  3038
#define NOTE_F4  2864
#define NOTE_G4  2550
#define NOTE_A4  2272
#define NOTE_B4  2028
#define NOTE_C5  1912

#define QUARTER 4
#define HALF 8
#define WHOLE 16


// our servo # counter
uint8_t servonum = 0;

//----



// servo array
int servos[] = {
  SERVOMID,
  SERVO_LEG_MID,
  SERVO_LEG_MID,

  SERVO_LEG_MID,
  SERVO_LEG_MID,
  SERVO_LEG_MID,

  SERVO_LEG_MID,
  SERVO_LEG_MID,
  SERVO_LEG_MID
};

// Button debouncing
int buttons[] = {
  LOW,
  LOW
};

// --- abstandsmesser
#include <NewPing.h>
#define  TRIGGER_PIN  6
#define  ECHO_PIN     7
#define MAX_DISTANCE 500 // Maximum distance we want to ping for (in centimeters).
//Maximum sensor distance is rated at 400-500cm.
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // Setup für NewPing (Pins und maximale Distanz).

int DistanceIn;
int DistanceCm;
// ---


/*=========================================================================
    APPLICATION SETTINGS

      FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
     
                                Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                                running this at least once is a good idea.
     
                                When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                                Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
         
                                Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
#define FACTORYRESET_ENABLE         0
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

// Create the bluefruit object, either software serial...uncomment these lines

SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                              BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);


/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/

//---- servo

void setServoPulse(uint8_t n, double pulse) {
  double pulselength;

  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= 60;   // 60 Hz
  Serial.print(pulselength); Serial.println(" us per period");
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit");
  pulse *= 1000;
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

//----


void resetLegs(void) {
  servos[LEG_FRONT_LEFT_PITCH] = SERVO_LEG_MID;
  servos[LEG_FRONT_LEFT_YAW] = SERVO_LEG_MID;
  servos[LEG_FRONT_RIGHT_PITCH] = SERVO_LEG_MID;
  servos[LEG_FRONT_RIGHT_YAW] = SERVO_LEG_MID;
  servos[LEG_BACK_RIGHT_PITCH] = SERVO_LEG_MID;
  servos[LEG_BACK_RIGHT_YAW] = SERVO_LEG_MID;
  servos[LEG_BACK_LEFT_PITCH] = SERVO_LEG_MID;
  servos[LEG_BACK_LEFT_YAW] = SERVO_LEG_MID;
  setServos();
}


void sleepPosition(void) {
  servos[HEAD_SERVO_INDEX] = SERVOMID;
  servos[LEG_FRONT_LEFT_PITCH] = SERVOMIN;
  servos[LEG_FRONT_LEFT_YAW] = SERVOMIN;
  servos[LEG_FRONT_RIGHT_PITCH] = SERVOMAX;
  servos[LEG_FRONT_RIGHT_YAW] = SERVOMAX;

  servos[LEG_BACK_RIGHT_PITCH] = SERVOMIN;
  servos[LEG_BACK_RIGHT_YAW] = SERVOMIN;
  servos[LEG_BACK_LEFT_PITCH] = SERVOMAX;
  servos[LEG_BACK_LEFT_YAW] = SERVOMAX;
  setServos();  
}

void setServos(void) {
  int i;
  for (i = 0; i < 9; i++) {
    pwm.setPWM(i, 0, servos[i]);
    delay(50);
  }
}

void setServo(int idx, int value) {
    servos[idx] = value;
    pwm.setPWM(idx, 0, value);
    delay(50); 
}

void moveLeg(int leg, int yaw, int pitch ) {
  int range = SERVOMAX - SERVOMIN;
  
  float stepValue = range/180;
  int convertedYaw;
  int convertedPitch;
  int yawingServo; 
  int pitchingServo;

  if ( (leg > 0) && (leg < 5)) {

    switch(leg) 
    {
      case 1:
        yawingServo = LEG_FRONT_LEFT_YAW;
        pitchingServo = LEG_FRONT_LEFT_PITCH;
        convertedYaw = (yaw * stepValue) + SERVOMIN ;
        convertedPitch = (pitch * stepValue) + SERVOMIN;
        break;
      case 2:
        yawingServo = LEG_FRONT_RIGHT_YAW;
        pitchingServo = LEG_FRONT_RIGHT_PITCH;
        convertedYaw = (yaw * stepValue) + SERVOMIN ;
        convertedPitch = (pitch * stepValue) + SERVOMIN;
        break;
     case 3:
        yawingServo = LEG_BACK_RIGHT_YAW;
        pitchingServo = LEG_BACK_RIGHT_PITCH;
        convertedYaw = (yaw * stepValue) + SERVOMIN ;
        convertedPitch = (pitch * stepValue) + SERVOMIN;
        break;
      case 4:
        yawingServo = LEG_BACK_LEFT_YAW;
        pitchingServo = LEG_BACK_LEFT_PITCH;
        convertedYaw = (yaw * stepValue) + SERVOMIN ;
        convertedPitch = (pitch * stepValue) + SERVOMIN;
        break;
    }
    pwm.setPWM(yawingServo, 0, convertedYaw);
    pwm.setPWM(pitchingServo, 0, convertedPitch);
  }
  


}

String getState(void) {
  String state = "";

  int i;
  for (i = 0; i < 9; i++) {
    state += "s";
    state += i;
    state += ":";
    state += servos[i];
    state += ";";
  }

  state += "d:";
  state += sonar.ping_cm();
  state += ";";

  state += "b:";
  int brightness = analogRead(0);
  state += brightness;
  state += ";";

  state += "l:";
  state += digitalRead(13);
  state += ";";

  return state;
}



String sweep(int count) {

  if (count < 2) {
    count = 0;
  }
  else if (count > 400) {
    count = 400;
  }
  String result = "?";
  result += count;
  result += ";";

  int fullRange = SERVOMAX - SERVOMIN;
  int stepSize = fullRange / (count - 1);
  for (int i = 0; i < count; ++i) {
    int rangeForStep = SERVOMIN + i * stepSize ;
    if (i == count - 1) {
      rangeForStep = SERVOMAX;
    }

    // move head
    pwm.setPWM(HEAD_SERVO_INDEX, 0, rangeForStep);
    delay(100);
    // read distance
    result += sonar.ping_cm();
    result += ";";
  }
  result += "\n";
  pwm.setPWM(HEAD_SERVO_INDEX, 0, SERVOMID);
  return result;
}


void playTone(int pin, int tone_, long duration) {
  long elapsed_time = 0;
  int rest_count = 100;
  long durationMicro = duration * 10000;
  if (tone_ > 0) { // if this isn't a Rest beat, while the tone has 
    //  played less long than 'duration', pulse speaker HIGH and LOW
    while (elapsed_time < durationMicro) {

      digitalWrite(pin,HIGH);
      delayMicroseconds(tone_ / 2);

      // DOWN
      digitalWrite(pin, LOW);
      delayMicroseconds(tone_ / 2);

      // Keep track of how long we pulsed
      elapsed_time += (tone_);
    } 
  }
  else { // Rest beat; loop times delay
    for (int j = 0; j < rest_count; j++) { // See NOTE on rest_count
      delayMicroseconds(durationMicro);  
    }                                
  }                                 
}

int stepper = 0;
void setup(void)
{

  pinMode(13, OUTPUT);

  pinMode(1, OUTPUT);

  pinMode(BUTTON_FRONT_PIN, INPUT);
  pinMode(BUTTON_REAR_PIN, INPUT);

  //---- servo
  pwm.begin();

  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  yield();

  //-----

  //while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit Command <-> Data Mode Example"));
  Serial.println(F("------------------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ) {
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
  Serial.println(F("Then Enter characters to send to Bluefruit"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
    delay(500);
  }

  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set module to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));
  resetLegs();
}
/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{

  // ---- ultraschall
  /*
    DistanceCm = sonar.ping_cm();
    ble.print("Distance: ");
    ble.print(DistanceCm);
    ble.println(" cm");

    int light = analogRead(0);
    ble.print("Light: ");
    ble.println(light);

  */
  //---


  //--- servo

  /*
    for(int s=0;s<8;s++) {

    // Drive each servo one at a time
    Serial.print("Servo: ");
    Serial.println(servonum);
    for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
      pwm.setPWM(servonum, 0, pulselen);
    }

    delay(500);
    for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
      pwm.setPWM(servonum, 0, pulselen);
    }

    delay(100);

    servonum ++;
    if (servonum > 7) servonum = 0;
    }
  */

  stepper++;
  
  // Check Buttons
    
  buttons[BUTTON_FRONT_INDEX] = digitalRead(BUTTON_FRONT_PIN);
  buttons[BUTTON_REAR_INDEX] = digitalRead(BUTTON_REAR_PIN);

  if (stepper >= 1500) {
    ble.println(getState());
    stepper = 0;
    
    if (buttons[BUTTON_FRONT_INDEX] == HIGH) {
      resetLegs();
      ble.println("leg reset.");
    }
    
    if (buttons[BUTTON_REAR_INDEX] == HIGH) {
      sleepPosition();
      ble.println("start pos");
    }
  }
  // -- servo

  // Check for user input
  char n, inputs[BUFSIZE + 1];

  if (Serial.available()) {
    n = Serial.readBytes(inputs, BUFSIZE);
    inputs[n] = 0;
    // Send characters to Bluefruit
    //   Serial.print("Sending: ");
    //   Serial.println(inputs);

    // Send input data to host via Bluefruit
    ble.print(inputs);
  }


  String cmd;
  // Echo received data
  while ( ble.available() )
  {

    int c = ble.read();
    Serial.print((char)c);
    // Hex output too, helps w/debugging!
    Serial.print(" [0x");
    if (c <= 0xF) Serial.print(F("0"));
    Serial.print(c, HEX);
    Serial.print("] ");

    cmd += ((char) c);
  }

  if (cmd.charAt(0) == 'r') {
    resetLegs();
    ble.println("leg reset");
  }
   else if (cmd.charAt(0) == 'x') {
    sleepPosition();
    ble.println("start pos");
  }
  else if (cmd.charAt(0) == 's') {
    int idx = cmd.substring(1, cmd.indexOf('=')).toInt();
    int val = cmd.substring(cmd.indexOf('=') + 1, cmd.indexOf('\n')).toInt();
    servos[idx] = val;
    pwm.setPWM(idx, 0, val);
    
    //setServos();
  } else if (cmd.charAt(0) == 'b') {

    ble.println("Beeping");
    tone(1, 1500, 250);
    delay(100);
    tone(1, 240, 250);
    delay(10);
    tone(1, 440, 250);
    delay(10);
    tone(1, 540, 250);
   
  } else if (cmd.charAt(0) == '?') {
    int steps = cmd.substring(1, cmd.indexOf('\n')).toInt();
    String sweepResult = sweep(steps);
    ble.println(sweepResult);
  }
  else if (cmd.charAt(0) == 'l') {
    digitalWrite(13, !digitalRead(13));
  }
  else if (cmd.charAt(0) == 'm') { 
    int firstSeparatorIndex = cmd.indexOf(';');
    int secondSeparatorIndex = cmd.indexOf(';', firstSeparatorIndex+1);
    int leg = cmd.substring(1, firstSeparatorIndex).toInt();
    int yaw = cmd.substring(firstSeparatorIndex+1, secondSeparatorIndex).toInt();
    int pitch = cmd.substring(secondSeparatorIndex+1, cmd.indexOf('\n')).toInt();
    String reply = "leg move:";
    reply += leg;
    reply += ";";
    reply += yaw;
    reply += ";";
    reply += pitch;
    ble.println(reply);
    moveLeg(leg,yaw,pitch); //TODO: 
  }
    else if (cmd.charAt(0) == 'p') {
    int firstSeparatorIndex = cmd.indexOf(';');
    int secondSeparatorIndex = cmd.indexOf(';', firstSeparatorIndex+1);
    int thirdSeparatorIndex = cmd.indexOf(';', secondSeparatorIndex+1);
    int fourthSeparatorIndex = cmd.indexOf(';', thirdSeparatorIndex+1);
    int l1 = cmd.substring(1, firstSeparatorIndex).toInt();
    int l2 = cmd.substring(firstSeparatorIndex+1, secondSeparatorIndex).toInt();
    int l3 = cmd.substring(secondSeparatorIndex+1, thirdSeparatorIndex).toInt();
    int l4 = cmd.substring(thirdSeparatorIndex+1, cmd.indexOf('\n')).toInt();


    setServo(LEG_FRONT_LEFT_PITCH, l1);
    setServo(LEG_FRONT_RIGHT_PITCH,l2);    
    setServo(LEG_BACK_RIGHT_PITCH, l3);
    setServo(LEG_BACK_LEFT_PITCH, l4);
    
    String reply = "leg pitch move:";
  }

  else if (cmd.charAt(0) == 'z') {

    ble.println("singing!");
    playTone(1, NOTE_E4, QUARTER);
    delay(100);
    playTone(1, NOTE_D4, QUARTER);
    delay(100);
    playTone(1, NOTE_C4, QUARTER);
    delay(100);
    playTone(1, NOTE_D4, QUARTER);
    delay(100);
    playTone(1, NOTE_E4, QUARTER);
    delay(100);
    playTone(1, NOTE_E4, QUARTER);
    delay(100);
    playTone(1, NOTE_E4, HALF);
    delay(100);
    playTone(1, NOTE_D4, QUARTER);
    delay(100);
    playTone(1, NOTE_D4, QUARTER);
    delay(100);
    playTone(1, NOTE_D4, HALF);
    delay(100);
    playTone(1, NOTE_E4, QUARTER);
    delay(100);
    playTone(1, NOTE_G4, QUARTER);
    delay(100);
    playTone(1, NOTE_G4, HALF);
    delay(100);
    playTone(1, NOTE_E4, QUARTER);
    delay(100);
    playTone(1, NOTE_D4, QUARTER);
    delay(100);
    playTone(1, NOTE_C4, QUARTER);
    delay(100);
    playTone(1, NOTE_D4, QUARTER);
    delay(100);
    playTone(1, NOTE_E4, QUARTER);
    delay(100);
    playTone(1, NOTE_E4, QUARTER);
    delay(100);
    playTone(1, NOTE_E4, QUARTER);
    delay(100);
    playTone(1, NOTE_E4, QUARTER);
    delay(100);
    playTone(1, NOTE_D4, QUARTER);
    delay(100);
    playTone(1, NOTE_D4, QUARTER);
    delay(100);
    playTone(1, NOTE_E4, QUARTER);
    delay(100);
    playTone(1, NOTE_D4, QUARTER);
    delay(100);
    playTone(1, NOTE_C4, WHOLE);
    delay(100);
  }
}
