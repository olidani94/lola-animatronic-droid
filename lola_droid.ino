// ------------- LIBRARIES ------------- //
#include "Arduino.h"
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"
#include <Servo.h>

// ------------- VARIABLES ------------- //
// MP3
SoftwareSerial mySoftwareSerial(10, 11); // RX, TX
DFRobotDFPlayerMini myDFPlayer;
int speakerVolume = 20;
void printDetail(uint8_t type, int value);

// BUTTON
int buttonPin = A0; // select the input pin for the button

// SERVOS
Servo leftWing;
Servo rightWing;
int leftWingServoPin = 5;
int rightWingServoPin = 6;
int maxLeftServoValue = 60; // Value for left
int minLeftServoValue = 0;
int maxRightServoValue = 180-60; // Value for right (inverted)
int minRightServoValue = 180;
int maxServoValue = 60;  // Absolute value for movement
int minServoValue = 0;

// MICROPHONE MODULE
int microphonePin = A4; // select the input pin for the microphone

// LEDS
int redLedPin = 2; // select the input pin for the potentiometer
int blueLedPin = 3; // select the input pin for the potentiometer

// FLOW MANAGEMENT
long currentTime;
long lastActionTime;
bool is_evil_mode_active;
int first_basic_song = 3; // Somehow song 0018 is number 1, 0001 is number 2, 0017 is number 18.
int last_basic_song = 18;

// ------------- SETUP ------------- //
void setup()
{
  // Start serial
  mySoftwareSerial.begin(9600);
  Serial.begin(115200);

  // Start I/O
  // - initialize the LED pins as an output:
  pinMode(blueLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);
  turnLightsBlue();
  
  // - initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT_PULLUP);

  // - initialize the microphone pin as an input:
  pinMode(microphonePin, INPUT);
  
  // - Attach the servos
  attachServos();
  closeWings();
  
  // Test hardware
  //test_hardware_except_mp3();
  
  // Start MP3
  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
  
  if (!myDFPlayer.begin(mySoftwareSerial, true, false)) {  //Use softwareSerial to communicate with mp3. https://forum.arduino.cc/t/arduino-and-dfplayer-works-only-after-pressing-the-reset-button/599065/3
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    // while(true); // This can block the program to avoid emiting sound
  }
  Serial.println(F("DFPlayer Mini online."));
  myDFPlayer.setTimeOut(2000);
  myDFPlayer.volume(speakerVolume);  //Set volume value. From 0 to 30
  //----Set different EQ----
  // myDFPlayer.EQ(DFPLAYER_EQ_NORMAL);
  //  myDFPlayer.EQ(DFPLAYER_EQ_POP);
  //  myDFPlayer.EQ(DFPLAYER_EQ_ROCK);
  //  myDFPlayer.EQ(DFPLAYER_EQ_JAZZ);
  //  myDFPlayer.EQ(DFPLAYER_EQ_CLASSIC);
  myDFPlayer.EQ(DFPLAYER_EQ_BASS);
  
  // myDFPlayer.play(1);  //Play the first mp3
  // Say hello
  myDFPlayer.volume(speakerVolume);  //Set volume value. From 0 to 30
  myDFPlayer.playMp3Folder(5); //play specific mp3 in SD:/MP3/0005.mp3; File Name(0~65535)
  delay(1000);
  openWings();
  delay(500);
  closeWings();
  lastActionTime = millis();
}


// ------------- LOOP ------------- //
void loop()
{
  // Update time
  currentTime = millis();

  // Detect button to turn into evil mode
  if (digitalRead(buttonPin) == LOW){
      // EVIL MODE FOR TEN SECONDS
      is_evil_mode_active = true;
      turnLightsRed();
      myDFPlayer.playMp3Folder(2);
      delay(1000); // This delay is here because mp3 module takes a while to run      
      flapWingsRandom();
      delay(1000);

      // Play custom song sometimes
      if (random(1, 5) == 1){
          openWings();
          myDFPlayer.playMp3Folder(1);
          delay(6000 + random(1000, 4000));
      } else {
          myDFPlayer.playMp3Folder(2);
          flapWingsRandom();
          openWings();
          delay(3000);
      }
      
      // Exit evil mode
      myDFPlayer.pause();
      closeWings();
      turnLightsBlue();
      is_evil_mode_active = false;      
  }


  // Prepare to play a random sound
  int songToPlay = random(first_basic_song, last_basic_song+1);    
  
  // If last action was longer than 10 seconds ago, do something
  if (currentTime - lastActionTime > 10000) {
    // Serial.println("I am bored");
    // DF ROBOT RESTARTS SOMEHOW; SET VOLUME
    myDFPlayer.volume(speakerVolume);  //Set volume value. From 0 to 30
    
    // Songs 3 and 4 are fly sequences
    if (songToPlay >= 5){
        myDFPlayer.playMp3Folder(songToPlay);
        delay(1000); // This delay is here because mp3 module takes a while to run
        flapWingsRandom();
    } else {
      myDFPlayer.playMp3Folder(4);
      openWings();
      delay(3000);
      myDFPlayer.playMp3Folder(3);
      delay(3000);
      flapWingsRandom();
    }
    //myDFPlayer.pause();
    currentTime = millis();
    lastActionTime = millis();
    
  } else if ((currentTime - lastActionTime > 3000) and (digitalRead(microphonePin))) { // If last action was between 3 and 10 seconds and noise was heard, do something now
    //Serial.println("Noise heard");
    // DF ROBOT RESTARTS SOMEHOW; SET VOLUME
    myDFPlayer.volume(speakerVolume);  //Set volume value. From 0 to 30
    
    // Songs 3 and 4 are fly sequences
    if (songToPlay >= 5){
        myDFPlayer.playMp3Folder(songToPlay);
        delay(1000); // This delay is here because mp3 module takes a while to run
        flapWingsRandom();
    } else {
      myDFPlayer.playMp3Folder(4);
      openWings();        
      delay(3000);
      myDFPlayer.playMp3Folder(3);
      delay(3000);
      flapWingsRandom();
    }
    //myDFPlayer.pause();    
    currentTime = millis();
    lastActionTime = millis();    
  }
  
  // Provide information about mp3 player
  if (myDFPlayer.available()) {
    printDetail(myDFPlayer.readType(), myDFPlayer.read()); //Print the detail message from DFPlayer to handle different errors and states.
  }

  delay(100);
}


// ------------- FUNCTIONS ------------- //
// --------- LEDS
void turnLightsRed(){
  digitalWrite(redLedPin, HIGH);
  digitalWrite(blueLedPin, LOW);
}

void turnLightsBlue(){
  digitalWrite(blueLedPin, HIGH);
  digitalWrite(redLedPin, LOW);
}

void turnLightsOff(){
  digitalWrite(blueLedPin, LOW);
  digitalWrite(redLedPin, LOW);
}

// --------- SERVOS
void attachServos(){
  leftWing.attach(leftWingServoPin);
  rightWing.attach(rightWingServoPin);
}

void detachServos(){
  leftWing.detach();
  rightWing.detach();
}

void openWings(){
  rightWing.write(maxRightServoValue);
  leftWing.write(maxLeftServoValue);
}

void closeWings(){
  rightWing.write(minRightServoValue);
  leftWing.write(minLeftServoValue);
}

void flapWingsRandom(){

  int moveRightWing = random(1, 6);      // if 1, dont move
  int moveLeftWing = random(1, 6);       // if 1, dont move
  int moveRightWingFirst = random(1, 3); // if 1, move right first
  int delayAtOpenPosition = random(100, 500); // random stop at open position
  int delayAtFirstClosePosition = random(50, 150); // random stop at first close position
  int delayAtSecondOpenPosition = random(50, 150); // random stop at second open position
      
  int descendInTwoTimesRightWing = random(1, 4); // if 1, ascend again while descending
  int descendInTwoTimesLeftWing = random(1, 4); // if 1, ascend again while descending
  int peakServoValue = 0; // To manage the angle of the wing

  // To avoid movement
  if (moveRightWing == 1){
    moveRightWing = 0;
  } else {
    moveRightWing = 1;
  }

  if (moveLeftWing == 1){
    moveLeftWing = 0;
  } else {
    moveLeftWing = 1;
  }
  
  // First opening sequence
  if (moveRightWingFirst == 1){
    // Move first right wing
    peakServoValue = random(maxServoValue-20, maxServoValue) * moveRightWing;
    rightWing.write(minRightServoValue-peakServoValue);
    
    // Move then left wing
    peakServoValue = random(maxServoValue-20, maxServoValue) * moveLeftWing;
    leftWing.write(peakServoValue);
    
  } else {
    // Move first left wing
    peakServoValue = random(maxServoValue-20, maxServoValue) * moveLeftWing;
    leftWing.write(peakServoValue);
    
    // Move then right wing
    peakServoValue = random(maxServoValue-20, maxServoValue) * moveRightWing;
    rightWing.write(minRightServoValue-peakServoValue);
   }

  // random delay before closing
  delay(delayAtOpenPosition);

  // close wings
  closeWings();
  
  // random delay before second opening sequence
  delay(delayAtFirstClosePosition);
  
  // Second opening sequence
  if (descendInTwoTimesRightWing == 1){
    // Move first right wing
    peakServoValue = random(maxServoValue-20, maxServoValue) * moveRightWing;
    rightWing.write(minRightServoValue-peakServoValue);
  }
  
  if (descendInTwoTimesLeftWing == 1){
    // Move first right wing
    peakServoValue = random(maxServoValue-20, maxServoValue) * moveLeftWing;
    leftWing.write(peakServoValue);
  }

  // random delay before final close
  delay(delayAtSecondOpenPosition);
  
  // close wings
  closeWings();
  
}

// --------- MP3
void printDetail(uint8_t type, int value){
  switch (type) {
    case TimeOut:
      Serial.println(F("Time Out!"));
      break;
    case WrongStack:
      Serial.println(F("Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      Serial.println(F("Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      Serial.println(F("Card Removed!"));
      break;
    case DFPlayerCardOnline:
      Serial.println(F("Card Online!"));
      break;
    case DFPlayerPlayFinished:
      Serial.print(F("Number:"));
      Serial.print(value);
      Serial.println(F(" Play Finished!"));
      break;
    case DFPlayerError:
      Serial.print(F("DFPlayerError:"));
      switch (value) {
        case Busy:
          Serial.println(F("Card not found"));
          break;
        case Sleeping:
          Serial.println(F("Sleeping"));
          break;
        case SerialWrongStack:
          Serial.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          Serial.println(F("Check Sum Not Match"));
          break;
        case FileIndexOut:
          Serial.println(F("File Index Out of Bound"));
          break;
        case FileMismatch:
          Serial.println(F("Cannot Find File"));
          break;
        case Advertise:
          Serial.println(F("In Advertise"));
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }

}

// --------- TESTS
void test_hardware_except_mp3(){
  Serial.println("Testing the hardware...");
  delay(2000);
  
  // ------------------ Test LEDS
  Serial.println("Testing the LEDS. Turning lights blue for two seconds.");
  turnLightsBlue();
  delay(2000);

  Serial.println("Testing the LEDS. Turning lights red for two seconds.");
  turnLightsRed();
  delay(2000);
  
  Serial.println("Testing the LEDS. Turning lights off.");
  turnLightsOff();
  
  // Test button 
  Serial.println("Testing the button. For 10 seconds lights will change color when pressed.");

  int numLoops = 0;
  turnLightsBlue();
  bool buttonState = LOW;
  
  while (numLoops < 20){
    // read the state of the pushbutton value:
    buttonState = digitalRead(buttonPin);
  
    // check if the pushbutton is pressed. If it is, the buttonState is LOW:
    if (buttonState == LOW) {
        turnLightsRed();
    } else {
        turnLightsBlue();
    }

    numLoops = numLoops + 1;
    delay(500);
  }
  
  turnLightsOff();
  delay(2000);
  
  
  // ------------------ Test MIC
  Serial.println("Testing the microphone. Lights will change color if sound detected.");
  turnLightsBlue();

  numLoops = 0;
  turnLightsBlue();
  delay(1000);
  
  while (numLoops < 20){
    // read the state of the microphone value:
    int sensorValue = digitalRead(microphonePin);
  
    // check if the microphone has detected sound and is HIGH:
    if (sensorValue == HIGH) {
        turnLightsRed();
    } else {
        turnLightsBlue();
    }

    numLoops = numLoops + 1;
    Serial.println(sensorValue);
    delay(100);
  }
   
  delay(2000);
  
  
  // ------------------ Test SERVOS
  Serial.println("Testing the servos. Moving left servo to 90 and then 0.");
  leftWing.write(65);
  delay(2000);
  leftWing.write(0);
  delay(2000);

  leftWing.write(65);
  delay(2000);
  leftWing.write(0);
  delay(2000);

  Serial.println("Testing the servos. Moving right servo to 90 and then 0.");
  rightWing.write(65);
  delay(2000);
  rightWing.write(0);
  delay(2000);

  rightWing.write(90);
  delay(2000);
  rightWing.write(0);
  delay(2000);

  // ------------------ Finish test
  Serial.println("Tests completed. Waiting 5 seconds to exit");
  delay(5000);
  
}
