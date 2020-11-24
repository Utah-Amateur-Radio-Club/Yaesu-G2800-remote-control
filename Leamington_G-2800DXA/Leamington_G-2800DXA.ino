/*
  Modified design for Utah Amateur Radio Club remote station at Leamington Utah.
  TTL interface to G-2800DXA controller.
  Output: rotate_left (TTL - Active LOW)
  Output: rotate_right (TTL - Active LOW)
  Output: MotorSpeed (PWM 25%, 50%, 75, 100%)
  Input: Position (0-5V corresponds to 0-450 degrees rotation)
  Using UNO A/D and D/A functions for Speed and Position.  I2C and ADS1115 interfaces not required.

  Yaesu G-800SA to Ham Radio Deluxe Interface
  Emulates Yeasu GS-232A Azimuth Controller

  Written by Glen Popiel, KW5GP
  Modified by Chuck Johnson, WA7JOS - May 2020

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 3 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

*/

// Debug Mode must be off to communicate with Ham Radio Deluxe
#define debug_mode 0  // Set to 1 for debug data on Serial Port
#include <EEPROM.h>  // Include the EEPROM Library

#define relay_1 4      // Define Relay 1 as Pin 4
#define relay_2 5      // Define Relay 2 as Pin 5
#define relay_3 6      // Define Relay 3 as Pin 6
#define relay_4 7      // Define Relay 4 as Pin 7
#define rotate_left 2  // Define Rotate Left as Pin 2 (Active LOW)
#define rotate_right 3 // Define Rotate Right as Pin 3 (Active LOW)
#define MotorSpeed 11  // Define Rotator Speed Control as Pin 11 (PWM - 25,50,75,100%)
#define Position A0    // Define Rotator Position Pot (0-5V : 0-450 degrees)
#define EEP_Enable 12  // Enable EE_PROM writes (Active LOW)

#define BAUD_RATE 9600  // Set the Serial Port Baud rate to 9600

#define EEPROM_ID_BYTE 1     // EEPROM ID to validate EEPROM data location
#define EEPROM_ID  56        // EEPROM ID Value
#define EEPROM_AZ_CAL_0 2    // Azimuth Zero Calibration EEPROM location   
#define EEPROM_AZ_CAL_MAX 12 // Azimuth Max Calibration Data EEPROM location  
#define AZ_CAL_0_DEFAULT     2  // Preset the Azimuth Zero Calibration Point to 0
#define AZ_CAL_MAX_DEFAULT 1020  // Preset the Azimuth Max (450 degree) Calibration Point

#define AZ_Tolerance 1  // Set the Azimuth Accuracy Tolerance (degrees)
#define AZ_Coast 3      // Set the Azimuth Coast distance (degrees)
#define AZ_Window 7     // Set the Azimuth Deceleration window (degrees)
#define Speed_1 63      // Motor speed 1
#define Speed_2 127     // Motor speed 2
#define Speed_3 191     // Motor speed 3
#define Speed_4 255     // Motor speed 4

//variables
byte inByte = 0;  // incoming serial byte
byte serial_buffer[50];  // incoming serial byte buffer
int serial_buffer_index = 0;  // The index pointer variable for the Serial buffer
int set_AZ;  // Azimuth set value
int current_AZ;  // Current Azimuth raw value
String Serial_Send_Data; // Data to send to Serial Port
int AZ_0;  // Azimuth Zero Value from EEPROM
int AZ_MAX; // Azimuth Max Value from EEPROM
int AZ_Degrees; // mapped AZ ADC value to Degrees
String Requested_AZ; // RS232 Requested Azimuth - M and short W command
int AZ_To; // Requested AZ Move
int AZ_Distance; // Distance to move AZ

void setup()
{
  pinMode(relay_1, OUTPUT);
  pinMode(relay_2, OUTPUT);
  pinMode(relay_3, OUTPUT);
  pinMode(relay_4, OUTPUT);
  pinMode(rotate_left, OUTPUT); // Define the Control Pins as Outputs
  pinMode(rotate_right, OUTPUT);
  pinMode(MotorSpeed, OUTPUT);
  pinMode(Position, INPUT);
  pinMode(EEP_Enable, INPUT);

  digitalWrite(relay_1, LOW);       // Turn off all the relays just to be sure
  digitalWrite(relay_2, LOW);       // Turn off all the relays just to be sure
  digitalWrite(relay_3, LOW);       // Turn off all the relays just to be sure
  digitalWrite(relay_4, LOW);       // Turn off all the relays just to be sure
  digitalWrite(EEP_Enable, HIGH);   // EE_Prom Enable pullup
  Serial.begin(BAUD_RATE);          // initialize serial communication

  set_AZ = -1;                      // Preset the Azimuth Move Variable
  az_rotate_stop();                 // Halt rotation
  read_eeprom_cal_data();           // Read the Azimuth Calibration Values from EEPROM

}  // End Setup Loop

void loop()
{
  check_serial(); // Check the Serial Port for Data
  check_move();   // Check to see if executing move command
} // End Main Loop

// Functions

void read_eeprom_cal_data()  // Function to Read the Azimuth Calibration Data
{
  if (EEPROM.read(EEPROM_ID_BYTE) == EEPROM_ID) // Verify the EEPROM has valid data
  {
    AZ_0   = (EEPROM.read(EEPROM_AZ_CAL_0)   * 256) + EEPROM.read(EEPROM_AZ_CAL_0 + 1);    // Read the Azimuth Zero Calibration Value from EEPROM
    AZ_MAX = (EEPROM.read(EEPROM_AZ_CAL_MAX) * 256) + EEPROM.read(EEPROM_AZ_CAL_MAX + 1);  // Read the Azimuth Maximum Calibration Value from EEPROM
    if (debug_mode) // If in Debug Mode Print the Calibration Values
    {
      Serial.println("Read EEPROM Calibration Data Valid ID");
      Serial.println(AZ_0  , DEC);
      Serial.println(AZ_MAX, DEC);
    }
  }
  else // initialize eeprom to default values
  {
    if (debug_mode)
      Serial.println("Read EEPROM Calibration Data Invalid ID - setting to defaults");
    AZ_0 = AZ_CAL_0_DEFAULT;  // Set the Calibration To Default Values
    AZ_MAX = AZ_CAL_MAX_DEFAULT;
    write_eeprom_cal_data();  // Write the Default Values to EEPROM
  }
}

void write_eeprom_cal_data() // Function to Write the Calibration Values to EEPROM
{
  Serial.println("Writing EEPROM Calibration Data");
  Serial.println(AZ_0  , DEC);
  Serial.println(AZ_MAX, DEC);
  EEPROM.write(EEPROM_ID_BYTE, EEPROM_ID); //   Write the EEPROM ID
  EEPROM.write(EEPROM_AZ_CAL_0, highByte(AZ_0)); // Write the Azimuth Zero Calibration High Order Byte
  EEPROM.write(EEPROM_AZ_CAL_0 + 1, lowByte(AZ_0));   // Write the Azimuth Zero Calibration Low Order Byte
  EEPROM.write(EEPROM_AZ_CAL_MAX, highByte(AZ_MAX)); // Write the Azimuth Max Calibration High Order Byte
  EEPROM.write(EEPROM_AZ_CAL_MAX + 1, lowByte(AZ_MAX)); // Write the Azimuth Max Calibration Low Order Byte
}

void check_serial() // Function to check for data on the Serial port
{
  if (Serial.available() > 0) // Get the Serial Data if available
  {
    inByte = Serial.read();  // Get the Serial Data
    // You may need to uncomment the following line if your PC software
    // will not communicate properly with the controller
    // Serial.print(char(inByte));  // Echo back to the PC
    if (inByte == 10)  // ignore Line Feeds
    {
      return;
    }
    if (inByte != 13) // Add to buffer if not CR
    {
      serial_buffer[serial_buffer_index] = inByte;
      if (debug_mode) // Print the Character received if in Debug mode
      {
        Serial.print("Received = ");
        Serial.println(serial_buffer[serial_buffer_index]);
      }
      serial_buffer_index++;  // Increment the Serial Buffer pointer
    }
    else   // It's a Carriage Return, execute command
    {
      if ((serial_buffer[0] > 96) && (serial_buffer[0] < 123))  //If first character of command is lowercase, convert to uppercase
        serial_buffer[0] = serial_buffer[0] - 32;

      // Decode first character of command
      switch (serial_buffer[0])
      {
// A Command - Stop the Azimuth Rotation
        case 65:
          if (debug_mode)
            Serial.println("A Command Received");
          az_rotate_stop();
          break;

// C - Return current azimuth
        case 67:
          if (debug_mode)   // Return the Buffer Index Pointer in Debug Mode
          {
            Serial.println("C Command Received");
            Serial.println(serial_buffer_index);
          }
          send_current_az();  // Return Azimuth if C Command
          break;

// F - Set the Max Calibration
        case 70:
          if (digitalRead(EEP_Enable))
            Serial.println("Calibration Disabled");
          else
          {
            if (serial_buffer_index == 2) // It's a secondary calibration F0=0, F5=450, F?=ask
            {
              if (serial_buffer[1] == 48) // It's an F0 - Calibrate the 0 degree value
              {
                if (debug_mode)
                  Serial.println("F0 command Received");
                set_0_cal();
                break;
              }
              if (serial_buffer[1] == 53) // It's an F5 - Calibrate the 450 degree value
              {
                if (debug_mode)
                  Serial.println("F5 command Received");
                set_max_az_cal();  // F - Set the Max Azimuth Calibration
                break;
              }
              if (serial_buffer[1] == 63) // It's an F? - Report calibration values
              {
                if (debug_mode)
                  Serial.println("F? command Received");
                Serial.println(AZ_0  , DEC);
                Serial.println(AZ_MAX, DEC);
                break;
              }
            }
            else
              break;
          }
          
// K - Control Relays
        case 75:
          if (debug_mode)
          {
            Serial.println("K Command Received");
            Serial.println(serial_buffer_index);
          }
          if (serial_buffer_index == 2)
          {
            if (serial_buffer[1] == 48)
              // Disable Relays
            {
              if (debug_mode)
                Serial.println("K0 command Received");
              digitalWrite(relay_1, LOW);       // Turn off relay 1
              digitalWrite(relay_2, LOW);       // Turn off relay 2
              digitalWrite(relay_3, LOW);       // Turn off relay 3
              digitalWrite(relay_4, LOW);       // Turn off relay 4
              break;
            }
            if (serial_buffer[1] == 49)
              // Enable Relay 1
            {
              if (debug_mode)
                Serial.println("K1 command Received");
              digitalWrite(relay_2, LOW);       // Turn off relay 2
              digitalWrite(relay_3, LOW);       // Turn off relay 3
              digitalWrite(relay_4, LOW);       // Turn off relay 4
              digitalWrite(relay_1, HIGH);      // Turn on  relay 1
              break;
            }
            if (serial_buffer[1] == 50)
              // Enable Relay 2
            {
              if (debug_mode)
                Serial.println("K2 command Received");
              digitalWrite(relay_1, LOW);       // Turn off relay 1
              digitalWrite(relay_3, LOW);       // Turn off relay 3
              digitalWrite(relay_4, LOW);       // Turn off relay 4
              digitalWrite(relay_2, HIGH);      // Turn on  relay 2
              break;
            }
            if (serial_buffer[1] == 51)
              // Enable Relay 3
            {
              if (debug_mode)
                Serial.println("K3 command Received");
              digitalWrite(relay_1, LOW);       // Turn off relay 1
              digitalWrite(relay_2, LOW);       // Turn off relay 2
              digitalWrite(relay_4, LOW);       // Turn off relay 4
              digitalWrite(relay_3, HIGH);      // Turn on  relay 3
              break;
            }
            if (serial_buffer[1] == 52)
              // Enable Relay 4
            {
              if (debug_mode)
                Serial.println("K4 command Received");
              digitalWrite(relay_1, LOW);       // Turn off relay 1
              digitalWrite(relay_2, LOW);       // Turn off relay 2
              digitalWrite(relay_3, LOW);       // Turn off relay 3
              digitalWrite(relay_4, HIGH);      // Turn on  relay 4
              break;
            }
          }
          else
            break;
          break;

// L - Rotate Azimuth CCW
        case 76:
          if (debug_mode)
            Serial.println("L Command Received");
          analogWrite(MotorSpeed, Speed_4); // Restore speed
          rotate_az_ccw();  // Call the Rotate Azimuth CCW Function
          break;

// M - Rotate to Set Point
        case 77:
          if (debug_mode)
            Serial.println("M Command Received");
          analogWrite(MotorSpeed, Speed_4); // Restore speed
          rotate_to();  // Call the Rotate to Set Point Command
          break;

// R - Rotate Azimuth CW
        case 82:
          if (debug_mode)
            Serial.println("R Command Received");
          analogWrite(MotorSpeed, Speed_4); // Restore speed
          rotate_az_cw();  // Call the Rotate Azimuth CW Function
          break;

// S - Stop All Rotation
        case 83:
          if (debug_mode)
            Serial.println("S Command Received");
          az_rotate_stop();  // Call the Stop Azimith Rotation Function
          break;

// X - Motor Speed Control
        case 88:
          if (serial_buffer_index == 2)
          {
            if (serial_buffer[1] == 49)
            {
              if (debug_mode)
                Serial.println("X1 command Received");
              analogWrite(MotorSpeed, Speed_1);
              break;
            }
            if (serial_buffer[1] == 50)
            {
              if (debug_mode)
                Serial.println("X2 command Received");
              analogWrite(MotorSpeed, Speed_2);
              break;
            }
            if (serial_buffer[1] == 51)
            {
              if (debug_mode)
                Serial.println("X3 command Received");
              analogWrite(MotorSpeed, Speed_3);
              break;
            }
            if (serial_buffer[1] == 52)
            {
              if (debug_mode)
                Serial.println("X4 command Received");
              analogWrite(MotorSpeed, Speed_4);
              break;
            }
            else if (debug_mode)
              Serial.println("Invalid X command Received");
            break;
          }
          if (debug_mode)
            Serial.println("Invalid command Received");
      }
      serial_buffer_index = 0;  // Clear the Serial Buffer and Reset the Buffer Index Pointer
      serial_buffer[0] = 0;
    }
  }
}

void send_current_az() // Send the Current Azimuth Function
{
  read_adc();  // Read the ADC
// Map Azimuth to degrees
  if (debug_mode)
    Serial.println(current_AZ);
  AZ_Degrees = Degrees(current_AZ);
  if (debug_mode)
    Serial.println(AZ_Degrees);
// Send it back via serial
  Serial_Send_Data = "";
  if (AZ_Degrees < 100)  // pad with 0's if needed
    Serial_Send_Data = "0";
  if (AZ_Degrees < 10)
    Serial_Send_Data = "00";
  Serial_Send_Data = "+0" + Serial_Send_Data + String(AZ_Degrees);  // Send the Azimuth in Degrees
  Serial.println(Serial_Send_Data);  // Return value via RS-232 port
}

void set_0_cal() // Set the 0 degree Azimuth Calibration Function
{
  Serial.println("Calibrate 0 Degree AZ Function");
  read_adc();  // Read the ADC
// save current Azimuth value to EEPROM - 0 degree Calibration
  Serial.println(current_AZ);
  AZ_0 = current_AZ;  // Set the Azimuth 90 degree Calibration to Current Azimuth Reading
  write_eeprom_cal_data();  // Write the Calibration Data to EEPROM
  Serial.println("0 degree Azimuth Calibration Complete");
}

void set_max_az_cal() // Set the Max Azimuth (450 degree) Calibration Function
{
  Serial.println("Calibrate Max AZ (450 Degree) Function");
  read_adc();  // Read the ADC
// save current az and el values to EEPROM - Zero Calibration
  Serial.println(current_AZ);
  AZ_MAX = current_AZ;  // Set the Azimuth Maximum (450 degree) Calibration to Current Azimuth Reading
  write_eeprom_cal_data();  // Write the Calibration Data to EEPROM
  Serial.println("Max Azimuth (450 degree) Calibration Complete");
}

void rotate_az_ccw() // Function to Rotate Azimuth CCW
{
  digitalWrite(rotate_left,  LOW);  // Set the Rotate Left Pin Low (active)
  digitalWrite(rotate_right, HIGH);  // Make sure the Rotate Right Pin is High (inactive)
}

void rotate_az_cw() // Function to Rotate Azimuth CW
{
  digitalWrite(rotate_right, LOW);    // Set the Rotate Right Pin Low
  digitalWrite(rotate_left, HIGH);    // Make sure the Rotate Left Pin High
}

void az_rotate_stop() // Function to Stop Azimuth Rotation
{
  digitalWrite(rotate_right, HIGH);  // Turn off the Rotate Right Pin
  digitalWrite(rotate_left, HIGH);   // Turn off the Rotate Left Pin
  analogWrite(MotorSpeed, Speed_4);
  set_AZ = -1;
}

void rotate_to() // Function to Rotate to Set Point
{
  if (debug_mode)
    Serial.println("M Command -  Rotate Azimuth To Function");
// Decode Command - Format Mxxx - xxx = Degrees to Move to
  if (debug_mode)
    Serial.println(serial_buffer_index);
  if (serial_buffer_index == 4)  // Verify the Command is the proper length
  {
    if (debug_mode)
      Serial.println("Value in [1] to [3]?");
    Requested_AZ = (String(char(serial_buffer[1])) + String(char(serial_buffer[2])) + String(char(serial_buffer[3])));  // Decode the Azimuth Value
    AZ_To = (Requested_AZ.toInt()); // AZ Degrees to Move to as integer
    if (AZ_To < 0) // Make sure we don't go below 0 degrees
      AZ_To = 0;
    if (AZ_To > 450) // Make sure we don't go over 450 degrees
      AZ_To = 450;
    if (debug_mode)
    {
      Serial.println(Requested_AZ);
      Serial.println(AZ_To);
    }
// set the move flag and start
    read_adc();  // Read the ADC
// Map it to degrees
    if (debug_mode)
      Serial.println(current_AZ);
    AZ_Degrees = Degrees(current_AZ);
    if (debug_mode)
      Serial.println(AZ_Degrees);
    AZ_Distance = AZ_To - AZ_Degrees;  // Figure out far we have to move
    set_AZ = AZ_To;
    if (abs(AZ_Distance) <= AZ_Tolerance)  // No move needed if we're within the Tolerance Range
    {
      az_rotate_stop();  // Stop the Azimuth Rotation
      set_AZ = -1;  // Turn off the Move Command
    }
    else  // Move Azimuth - figure out which way
    {
      if (AZ_Distance > 0)   //We need to move CW
        rotate_az_cw();  // If the distance is positive, move CW
      else
        rotate_az_ccw();  // Otherwise, move counterclockwise
    }
  }
}

void read_adc() // Function to read the ADC
{
  current_AZ = analogRead(Position);  // Read ADC Channel 0
  if (debug_mode)
  {
    Serial.println("Read ADC Function  ");
    Serial.println(current_AZ);
  }
}

void check_move() // Check to see if we've been commanded to move
{
  if (set_AZ != -1)   // We're moving - check and stop as needed
  {
    read_adc();  // Read the ADC
// Map AZ to degrees
    if (debug_mode)
      Serial.println(current_AZ);
    AZ_Degrees = Degrees(current_AZ);
    if (debug_mode)
      Serial.println(AZ_Degrees);
    if (set_AZ != -1) // If Azimuth is moving
    {
      AZ_Distance = set_AZ - AZ_Degrees;  // Check how far we have to move
      if (abs(AZ_Distance) <= AZ_Window)  // Approaching target - slow down
        analogWrite(MotorSpeed, Speed_1);
      if (abs(AZ_Distance) <= AZ_Coast)  // Turn off early to allow coast
      {
        az_rotate_stop();  // Stop the Azimuth Rotation
        set_AZ = -1;  // Turn off the Azimuth Move Command
      }
      else  // Move Azimuth - figure out which way
      {
        if (AZ_Distance > 0)   //We need to move CW
          rotate_az_cw();  // Rotate CW if positive
        else
          rotate_az_ccw();  // Rotate CCW if negative
      }
    }
  }
}

int Degrees(int az_value)
{
  int corrected_degrees;
  if (az_value <= AZ_0)
    corrected_degrees = 0;
  if ((az_value >= AZ_0) && (az_value <= AZ_MAX))
    corrected_degrees = map(az_value, AZ_0, AZ_MAX, 0, 450);
  if (az_value > AZ_MAX)
    corrected_degrees = 450;
  return (corrected_degrees);
}
