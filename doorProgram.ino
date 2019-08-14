/*************************************************************
  Download latest Blynk library here:
    https://github.com/blynkkk/blynk-library/releases/latest

  Blynk is a platform with iOS and Android apps to control
  Arduino, Raspberry Pi and the likes over the Internet.
  You can easily build graphic interfaces for all your
  projects by simply dragging and dropping widgets.

    Downloads, docs, tutorials: http://www.blynk.cc
    Sketch generator:           http://examples.blynk.cc
    Blynk community:            http://community.blynk.cc
    Follow us:                  http://www.fb.com/blynkapp
                                http://twitter.com/blynk_app

  Blynk library is licensed under MIT license
  This example code is in public domain.

 *************************************************************

  This sketch shows how to read values from Virtual Pins

  App project setup:
    Button widget (0...1) on Virtual Pin V1
 *************************************************************/

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial


#include <ESP8266WiFi.h>
#include <NTPClient.h> //https://lastminuteengineers.com/esp8266-ntp-server-date-time-tutorial/
#include <BlynkSimpleEsp8266.h>
#include <WiFiUdp.h>


#define closeSensorPin 5
#define switchPin 14
#define ledPin 4

const long utcOffsetInSeconds = -25200;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);

//Logic for the state of the door
bool doorIsOpen = false;
bool prevState = false;

int readFreq = 200; //read sensors every 200ms
unsigned long prevReadTime = 0;

//Make sure the door isn't open too long
unsigned long doorOpenLength = 900000; //door can be open 15 minutes before notification sent to phone
unsigned long timeOpened = 0;

//Triggering the switch pin
unsigned long timeSetHigh = 0;
unsigned long timeToHold = 250; //hold the pin high for 1/4 second
bool switchOn = false;

//Blinking the LED to show the system is running
unsigned long onTime = 100; //100 ms
unsigned long offTime = 2000; //2 sec
unsigned long lastLedSwitch = 0;
bool ledOn = false;
 
WidgetLCD lcd(V0);
WidgetTerminal terminal(V2);


// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "token";

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "SSID";
char pass[] = "PASSWORD";

// This function will be called every time Button Widget
// in Blynk app writes values to the Virtual Pin V1
BLYNK_WRITE(V1)
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable
  // process received value
  if( pinValue == 1 && switchOn == false){ 
    switchOn = true;
    digitalWrite(switchPin, HIGH);
    timeSetHigh = millis();   
  }
}

//timer to control switch pin off so its turned on for a ceratin amount of time without using 'delay'
void turnPinOff(){
  if( switchOn == true ){
    unsigned long currentTime = millis();
    if ( currentTime - timeSetHigh >= timeToHold ){
      digitalWrite(switchPin, LOW);
      switchOn = false;
    }
  }
}

//This prints to the LCD based on the state change of the door sensor, it is independent of the switch being pressed or not
void print_lcd(bool state)
{
  if( state != prevState){ //Check if the state has changed
    lcd.clear();

    Serial.println("state change");
    if( state == false ){ //door is closed
      printMessage("Door has closed");
      lcd.print(2, 0, "Door Is:");
      lcd.print(4, 1, "CLOSED");
    }
    else if( state == true ){ //door is open
      printMessage("Door has opened");
      Blynk.notify("Garage Door was opened");
      timeOpened = millis();
      lcd.print(2, 0, "Door Is:");
      lcd.print(4, 1, "OPEN");
    }
    else{
      lcd.print(5, 0, "ERROR");
    }
    prevState = state;
  }
}

//Read the door sensor and set doorIsOpen to the boolean reading
void readSensors()
{
  unsigned long currentTime = millis();
  if( currentTime - prevReadTime >= readFreq ){
    prevReadTime = currentTime;
    doorIsOpen = digitalRead( closeSensorPin ); //Normally Open swtich, reads high when door is open, reads low when door is closed
    Serial.print("Door is: ");
    Serial.println(doorIsOpen);
    print_lcd(doorIsOpen);
  }
}

//Timer for tracking how long door has been open and pushing notifications if the open time exceeds the set time
void checkDoorOpenTime(){
  if( doorIsOpen == true ){
    unsigned long currentTime = millis();
    if( currentTime - timeOpened >= doorOpenLength ){
      Blynk.notify("Garage door was left open!");
      timeOpened = currentTime; //reset timeOpened to the current time so the message gets sent again every __ minutes based on doorOpenLength
    }
  }
}



void printMessage( const String& inputString){ //https://arduino.stackexchange.com/questions/3059/functions-with-string-parameters
  timeClient.update();

  terminal.print( inputString );
  terminal.print(" --- ");
  terminal.print(daysOfTheWeek[timeClient.getDay()]);
  terminal.print(", ");
  terminal.print(timeClient.getHours());
  terminal.print(":");
  terminal.print(timeClient.getMinutes());
  terminal.print(":");
  terminal.println(timeClient.getSeconds());
  terminal.flush();
}

void blinkLED(){
  unsigned long currentTime = millis();
  if( ledOn == false ){
    if( currentTime - lastLedSwitch >= offTime ){
      ledOn = true;
      lastLedSwitch = currentTime;
      digitalWrite(ledPin, HIGH);
    }
  }
  else if( ledOn == true ){
     if( currentTime - lastLedSwitch >= onTime ){
      ledOn = false;
      lastLedSwitch = currentTime;
      digitalWrite(ledPin, LOW);
    }
  }  
}

//Run all the functions -> Read sensors, check the door open time, and if the door switch is set high - run a timer to turn it off after a certain time. 
void runAll(){
  readSensors();
  checkDoorOpenTime();
  turnPinOff();
  blinkLED();
}


void setup()
{
  // Debug console
  Serial.begin(9600);

  Blynk.begin(auth, ssid, pass);

  lcd.clear();
  
  pinMode(closeSensorPin, INPUT_PULLUP);
  pinMode(switchPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  digitalWrite(switchPin, LOW); 

  timeClient.begin(); 

  printMessage("Connected");
}



void loop()
{
  Blynk.run();
  runAll();
}
