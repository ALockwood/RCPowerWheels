//Written for Uno
//TODO: Remove excess logging (debug) code.
//TODO: Address in-line TODO's :p
//TODO: Allow steering trim in the future my making SteeringNeutral R/W (EEPROM req'd?)

#include <XBOXRECV.h>
#include <SoftwareSerial.h>

//Xbox Wireless Receiver Vars
USB Usb; //Init USB controller
XBOXRECV controller(&Usb); //Attach controller to 'Usb' by passing a pointer ref
const byte MasterController = 0; //Define which controller can control the Arduino. Default to controller #1 (0).
const short analogLeftStickDeadzonePos = 7000;
const short analogLeftStickDeadzoneNeg = -7000;

//Power Vars
const byte DrivePWMPin = 6;  //Pin that will send the drive motor(s) PWM signal
const byte DriveDirectionPin = 5; //Pin that will be set as digital to control forward or reverse drive motor direction. HIGH = REV, LOW = FWD
boolean directionForward = true;

//Steering Vars
const short SteeringMaxLeft = 2960;  //Tested left turn maximum 'target' value for linear actuator position via Jrk
const short SteeringMaxRight = 1900;  //Tested right turn maximum 'target' value for linear actuator position via Jrk
const short SteeringNeutral = 2370;  //Measured neutral steering position. 
short SteeringMotorPosition;  //Holds the value for current/desrired steering 'target'

//Pololu Jrk Serial Coms Vars
const byte UARTDetect = 170; //Tells Pololu Jrk to auto-detect UART speed and serves as the 'command byte'
const byte MotorOFF = 255; //Turns off the motor
const byte MotorCmd_SetTargetHighRes = 192; //Command to move the motor. MUST READ: https://www.pololu.com/docs/0J38/4.e
const byte MotorCmd_Low5 = 31; //0x1F - Bitwise & with target value to get the lower 5 bits to put with MotorCmd_SetTargetHighRes
const byte MotorCmd_High7 = 127; //0x7f - Bitwise & with target value to get upper
//Device number not required when using compact protocol for Pololu Jrk
//const byte JRKDeviceNumber = 11; //Jrk controller device number (Default is 11: https://www.pololu.com/docs/0J38/4.c)

//SoftwareSerial Vars
SoftwareSerial JrkSerial(2, 3); //Defines a software serial connection for the Jrk with Rx on pin 2, Tx on pin 3.

//Misc Vars
boolean controlActive = false; //When false, no remote functions will work. Toggled via start button on controller.

//SETUP
void setup() 
{ 
  //Configure the drive pins
  pinMode(DrivePWMPin, OUTPUT); //Sets the drive motor PWM pin as output.
  pinMode(DriveDirectionPin, OUTPUT); //Set the drive direction pin as output. Put a 1k resistor on this.
  SetDriveMotorPowerLevel(0); //Initialize drive motors off.
  SetDriveDirection(true); //Default to FWD direction
      
  Serial.begin(115200); //Start the USB-connected serial port for debugging (pins: 0, 1)
  JrkSerial.begin(9600); //Start the SoftwareSerial port for Jrk coms

  //USB shield must connect, otherwise do nothing.
  if (Usb.Init() == -1) 
  {
    Serial.print(F("\r\nUSB shield init failed!"));
    while (1); //effectively halt by looping forever with no escape
  }
  Serial.print(F("\r\nXbox Wireless Receiver Library Started"));
}

//FUNCTIONS

//METHODS (technically just more functions... but ones that return no values)

//Sets up controller LEDs and 
void SetVehicleAndControllerDefaults()
{  
  //If controller isn't connected (controlActive == false) then steering should be centered and power to wheels should be zero. Controller lights flash.
  if (controlActive == false)
  {
    controller.setLedMode(ROTATING, MasterController);
    DisableSteering();
    SetDriveMotorPowerLevel(0);
  }
  else
  {//Set controller lights. Top left of X ring is on for FWD, bottom left is on for REV.
    if (directionForward == true)
    {
      controller.setLedOn(LED1, MasterController);
    }
    else
    {
      controller.setLedOn(LED3, MasterController);	  
    }
	
	SetDriveDirection(directionForward);
  }
}

//Sets the power level for the drive motors. 0-255 == Off - Full Power.
void SetDriveMotorPowerLevel(byte PowerLevel)
{
  if (PowerLevel > 0) //Only log to serial if a non-zero value is set. Could help with debugging analog inputs that don't zero out correctly?
  {
    Serial.print("Drive Motor Power: ");
    Serial.println(PowerLevel);
  }
  
  analogWrite(DrivePWMPin, PowerLevel);
}

//Sets the drive direction pin value high or low for MD30C. Low == FWD, HIGH == REV
void SetDriveDirection(boolean Forward)
{
  if (Forward == true)
  {
    digitalWrite(DriveDirectionPin, LOW);
  }
  else
  {
    digitalWrite(DriveDirectionPin, HIGH);
  }
}

void SetSteeringPosition(short AnalogStickPosition)
{
  //Deadzone check. If value is within deadzone set steering value to neutral, otherwise map the value
  if (AnalogStickPosition <= analogLeftStickDeadzoneNeg || AnalogStickPosition >= analogLeftStickDeadzonePos)
  {
    //DEBUG
    Serial.print("Stick Val: ");
    Serial.println(AnalogStickPosition);
    Serial.print("Steering val: ");
    Serial.println(map(AnalogStickPosition, -32768, 32767, SteeringMaxLeft, SteeringMaxRight));
    //END DEBUG
    //Xbox analog stick ranges from -32768 to 32767: max left to max right.
    SteeringMotorPosition = map(AnalogStickPosition, -32768, 32767, SteeringMaxLeft, SteeringMaxRight);
  }
  else
  {
    SteeringMotorPosition = SteeringNeutral; //Return to neutral position
  }
  
  //Send the steering target value to the Jrk
  JrkSerial.write(UARTDetect);
  JrkSerial.write(MotorCmd_SetTargetHighRes + (SteeringMotorPosition & MotorCmd_Low5)); //Set target command + 5 data bits
  JrkSerial.write((SteeringMotorPosition >> 5) & 0x7F); //7 remaining data bits

  delay(1); //Delay for 1ms for stability
}

//Turn off the steering motor
//TODO: Consider sending a 'center' command first?
void DisableSteering()
{
  JrkSerial.write(UARTDetect);
  JrkSerial.write(MotorOFF);
}

//Set defaults if the remote Rx disconnects. Functionally, a "Dead man's switch"
//Confirmed working with battery pull.
//TODO: Test natural out-of-range result.
void Deadman()
{
  controlActive = false;
  SetVehicleAndControllerDefaults(); //Set defaults if the controller disconnects or isn't connected
}

//MAIN
void loop() 
{
  Usb.Task(); //Blocks until something happens on USB or 5s, whichever comes first (I think??)

  if (controller.XboxReceiverConnected) 
  {
    if (controller.Xbox360Connected[MasterController])
    {
      //Watch for the start button being pressed, then toggle control
      if (controller.getButtonClick(START, MasterController))
      {
        Serial.println("START - Btn down");
        controlActive = !controlActive;
        SetVehicleAndControllerDefaults();
      }
      
      if (controlActive)
      {
        if (controller.getButtonPress(R2, MasterController)) //Get right trigger value if non-zero, then set PWM signal
        {
          SetDriveMotorPowerLevel(controller.getButtonPress(R2, MasterController));
        }
        else
        {
         SetDriveMotorPowerLevel(0); 
        }

        //Steer the vehicle!
        SetSteeringPosition(controller.getAnalogHat(LeftHatX,MasterController));
        
        if (controller.getButtonClick(UP, MasterController)) 
        {
          Serial.println(F("Up"));
          directionForward = true;
          SetVehicleAndControllerDefaults();
        }
        
        if (controller.getButtonClick(DOWN, MasterController)) 
        {
          Serial.println(F("Down"));
          directionForward = false;
          SetVehicleAndControllerDefaults();
        }
        
		//TODO: Use these for something. Perhaps trim mode?
        if (controller.getButtonClick(BACK,MasterController)) 
        {
          //controller.setLedBlink(ALL,MasterController);
          Serial.println(F("Back"));
        }
  
        if (controller.getButtonClick(SYNC,MasterController)) 
        {
          Serial.println(F("Sync"));
          controller.disconnect(MasterController);
        }
        
      } //controlActive = true
      else
      {
        SetVehicleAndControllerDefaults(); 
      }
    } //controller[MasterController] connected
    else
    {
      Deadman(); //Set defaults if the controller disconnects or isn't connected
    }
  } //if xbox RX connected
  else
  {
    Deadman(); //Set defaults if the remote Rx disconnects.
  }
} //end loop
