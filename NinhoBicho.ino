//
//
//
#include "NinhoBicho.h"



void setup()
{
  DEBUG_ENABLE();
  
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT); 

  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  pinMode(CLOSED_TRAY_SWITCH_PIN, INPUT); 
  digitalWrite(CLOSED_TRAY_SWITCH_PIN, HIGH);
  
  pinMode(OPENED_TRAY_SWITCH_PIN, INPUT); 
  digitalWrite(OPENED_TRAY_SWITCH_PIN, HIGH);
  
  // Set initial rotation direction
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
}

bool isTrayClosed()
{

  return !digitalRead(CLOSED_TRAY_SWITCH_PIN);
}

bool isTrayOpened()
{

  return !digitalRead(OPENED_TRAY_SWITCH_PIN);
}

uint16_t readSensor()
{
  return analogRead(SENSOR_ANALOG_PIN);   //(analogRead(SENSOR_ANALOG_PIN)/2)*2.54;   
}

void toggleLED()
{
  static bool ledState=false;

  ledState=!ledState;
  digitalWrite(LED_PIN,ledState);
}

void stopTray()
{
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
}

void openTray()
{
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  analogWrite(ENA_PIN,motorPower);
}

void closeTray()
{
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);
  analogWrite(ENA_PIN,motorPower);
}


void loop()
{

  bool bTrayClosed=false;
  bool bTrayOpened=false;
  uint16_t distance=0;
  
  static trayPulseState currentTrayState=TRAY_IDLE;
  static sensorState currentSensorState=SENSOR_IDLE;
  static uint8_t sensorMode=STOPPED;
  //static uint16_t trayDelay=NULL_TRAY_DELAY;
  
  bTrayClosed=isTrayClosed();
  bTrayOpened=isTrayOpened();
  distance=readSensor();


  DEBUG_PRINTLN();
 
      //is the sensor reading within range?
      if(distance>FAR_DISTANCE)
      {
        //remains idle
        sensorMode=STOPPED;
        DEBUG_PRINT("sensorMode: STOPPED");
      }
      else if(distance<FAR_DISTANCE && distance>MEDIUM_DISTANCE)
      {
          //Slow movement
          sensorMode=SLOW;
          motorPower=SLOW_MOTOR_POWER;
          //trayDelay=SLOW_TRAY_DELAY; 
          DEBUG_PRINT("sensorMode: SLOW");
      
      }
      else if(distance<MEDIUM_DISTANCE && distance>NEAR_DISTANCE)
      {
          //Medium movement
          sensorMode=MEDIUM;
          motorPower=MEDIUM_MOTOR_POWER;
          //trayDelay=MEDIUM_TRAY_DELAY; 
          DEBUG_PRINT("sensorMode: MEDIUM");
     
      }
      else if(distance<NEAR_DISTANCE)
      {
          //Fast movement 
          sensorMode=FAST;
          motorPower=FAST_MOTOR_POWER;
          //trayDelay=FAST_TRAY_DELAY;  
          DEBUG_PRINT("sensorMode: FAST");         
      }

  DEBUG_PRINT(" | Distance: ");
  DEBUG_PRINTLN(distance);

  switch(currentTrayState)
  {
    case TRAY_IDLE:
      //sensor
      if(sensorMode!=STOPPED)
      {
        currentTrayState=TRAY_DETECTED;
        DEBUG_PRINTLN("currentTrayState: TRAY_IDLE -> TRAY_DETECTED ");
      }
      stopTray();
      DEBUG_PRINTLN("currentTrayState: TRAY_IDLE ");
    break;
    
    case TRAY_DETECTED:
      //
      if(sensorMode==SLOW)
      {
        currentTrayState=TRAY_OPEN;
        //trayDelay=SLOW_TRAY_DELAY;
        DEBUG_PRINTLN("currentTrayState: TRAY_DETECTED -> SLOW ");
        
      }
      else if(sensorMode==MEDIUM)
      {
        currentTrayState=TRAY_OPEN;
        //trayDelay=MEDIUM_TRAY_DELAY;
        DEBUG_PRINTLN("currentTrayState: TRAY_DETECTED -> MEDIUM ");
      } 
      else if(sensorMode==FAST)
      {
        currentTrayState=TRAY_OPEN;
        //trayDelay=FAST_TRAY_DELAY;
        DEBUG_PRINTLN("currentTrayState: TRAY_DETECTED -> FAST ");
      } 
      else
      {
        currentTrayState=TRAY_IDLE;
        //trayDelay=NULL_TRAY_DELAY;
        DEBUG_PRINTLN("currentTrayState: TRAY_DETECTED -> TRAY_IDLE ");        
      }
    break;
            
    case TRAY_OPEN:
       openTray();
       //delay(trayDelay);
       if(!bTrayOpened)
       {
        DEBUG_PRINTLN("currentTrayState: TRAY_OPEN -> bTrayOpened false ");        
        break;
       }
       //stopTray();
       currentTrayState=TRAY_CLOSE;
       DEBUG_PRINTLN("currentTrayState: TRAY_OPEN -> TRAY_CLOSE ");        
       
    break;
    
    case TRAY_CLOSE:
       closeTray();
       //delay(trayDelay);
       if(!bTrayClosed)
       {
        DEBUG_PRINTLN("currentTrayState: TRAY_CLOSE -> bTrayClosed false ");        
        break;
       }
       //stopTray();
       currentTrayState=TRAY_DETECTED;
       DEBUG_PRINTLN("currentTrayState: TRAY_CLOSE -> TRAY_DETECTED ");        

    break;
  }
   delay(200);
}

/*
void loop_() {
  uint8_t potValue = analogRead(A0); // Read potentiometer value
  uint8_t pwmOutput = map(potValue, 0, 1023, 0 , 255); // Map the potentiometer value from 0 to 255
  analogWrite(enA, pwmOutput); // Send PWM signal to L298N Enable pin
  // Read button - Debounce
  if (digitalRead(button) == true) {
    pressed = !pressed;
  }
  while (digitalRead(button) == true);
  delay(20);
  // If button is pressed - change rotation direction
  if (pressed == true  & rotDirection == 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    rotDirection = 1;
    delay(20);
  }
  // If button is pressed - change rotation direction
  if (pressed == false & rotDirection == 1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    rotDirection = 0;
    delay(20);
  }
}

  switch(currentSensorState)
  {
    case SENSOR_IDLE:
      //is the sensor reading within range?
      if(distance>FAR_DISTANCE)
      {
        //remains idle
        sensorMode=STOPPED;
        break; 
      }
      else if(distance<FAR_DISTANCE && distance>NEAR_DISTANCE)
      {
          //Slow movement
          sensorMode=SLOW;
          trayDelay=SLOW_TRAY_DELAY;          
      }
      else if(distance<NEAR_DISTANCE)
      {
          //Fast movement 
          sensorMode=FAST;
          trayDelay=FAST_TRAY_DELAY;   
      }
      
    break;
    
    case SENSOR_OBJECT_NEAR:
      
    break;
        
    case SENSOR_OBJECT_FAR:
    break;
    
  }

*/
