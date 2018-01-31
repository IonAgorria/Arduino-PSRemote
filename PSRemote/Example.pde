#include <PSRemote.h>

PSRemote PS3Game;   // create an object for the PS3 Game Controller
  
void setup() 
{ 
  Serial.begin( 38400 ); //WARNING: Lowering and alot of serial use can cause delays that makes ps responses slow, you can put faster if you use serial alot to reduce delay
  Serial.println("Aloha");
  PS3Game.init();
}

void loop() 
{ 
  PS3Game.task(); //WARNING: "Long" delays may cause (it sure does) terrible lag (1-3 secs or more) in ps remote responses (presses, joystics and sensors)
  
  if (PS3Game.statConnected() && PS3Game.statReportReceived()) { // report received ?
    if(PS3Game.buttonChanged()){   // right and left buttons change mode joystick/Accelerometer
      if(PS3Game.buttonPressed(buLeft)) {
        PS3Game.LED(psLED1);
      }
      
      if(PS3Game.buttonPressed(buRight)) {
        PS3Game.LED(psLED2);
      }
      
      if(PS3Game.buttonPressed(buUp)) {
        PS3Game.LEDRumble(psLED3+psRumbleLow);
      }
      
      if(PS3Game.buttonPressed(buDown)) {
        PS3Game.LEDRumble(psLED1+psLED4);
      }
      
      if(PS3Game.buttonPressed(buCross)) {
        Serial.println("Rumble High");
        PS3Game.Rumble(psRumbleHigh);
      }
    
      if(PS3Game.buttonPressed(buCircle)) {
        Serial.println("Rumble Low");
        PS3Game.Rumble(psRumbleLow);
      }
      
      if(PS3Game.buttonPressed(buTriangle)) {
        Serial.println("Rumble None");
        PS3Game.Rumble(0);
      }
      
    }
     else {
    
      if(PS3Game.buttonPressed(buL1)) {
        Serial.print("JoyStick 1 = X ");
        Serial.print(int(PS3Game.getJoystick(leftJoystickX)));
        Serial.print(" Y ");
        Serial.println(int(PS3Game.getJoystick(leftJoystickY)));
      }
    
      if(PS3Game.buttonPressed(buR1)) {
        Serial.print("JoyStick 2 = X ");
        Serial.print(int(PS3Game.getJoystick(rightJoystickX)));
        Serial.print(" Y ");
        Serial.println(int(PS3Game.getJoystick(rightJoystickY)));
      }
      
      if(PS3Game.buttonPressed(buL2)) {
        Serial.print("Accelerometer = X ");
        Serial.print(int(PS3Game.getMotion(AccelerometerX)));
        Serial.print(" Y ");
        Serial.print(int(PS3Game.getMotion(AccelerometerY)));
        Serial.print(" Z ");
        Serial.println(int(PS3Game.getMotion(AccelerometerZ)));
      }
      
      if(PS3Game.buttonPressed(buR2)) {
        Serial.print("Gyrometer = Z ");
        Serial.println(int(PS3Game.getMotion(GyrometerZ)));
      }
    }
  }
}