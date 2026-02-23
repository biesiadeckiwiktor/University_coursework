#include <HCSR04.h>
#include <Stepper.h>
#include <LiquidCrystal.h>

// values read from analog input
// 1 - 813
// 3 - 650
// 5 - 425
// 2 - 730

//variables
int stepsPerRevolution = 2048; // counts per full revolution of output shaft according to specification
int steps90Degrees = stepsPerRevolution / 4; // counts per 1/4 turn (90 degrees)
int userInput[4];
int password[4] = {810,650,425,730}; //1 3 5 2 - values in array are analog input
long distance; // variable to store distance value from ultrasonic sensor
int sensorValue = 0;
int tolerance = 50; // tolerance for analog reading comparison due to flactuations in readings

// pin setup
Stepper myStepper(stepsPerRevolution, 10, 12, 11, 13);
HCSR04 hc(9, 6); //initialisation class HCSR04 (trig pin , echo pin)
LiquidCrystal lcd(8, 7, 5, 4, 3, 2);

void setup() {
  Serial.begin(9600);
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("...Automatic...");
  lcd.setCursor(0, 1);
  lcd.print("....Barrier....");
  delay(2000);
  myStepper.setSpeed(15);
}

void loop() {
  getDistance();
  Serial.println(distance);
  delay(250);
  
  if (distance < 10)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Vehicle detected");
    delay(2000);
    
    // Password entry loop
    while (true) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Enter password:");
      
      // Get 4 button presses
      for (int i = 0; i <= 3; i++)
      {
        // Wait for button press
        while (analogRead(A0) < 100) {
          delay(10);
        }
        sensorValue = analogRead(A0);
        userInput[i] = sensorValue;
        
        lcd.setCursor(i, 1);
        lcd.print("*");
        
        // Wait for button release
        while (analogRead(A0) > 100) {
          delay(10);
        }
        delay(200);
      }
      
      // Check password
      bool isEqual = true;
      for (int i = 0; i <= 3; i++) {
        if (abs(userInput[i] - password[i]) > tolerance) {
          isEqual = false;
          break;
        }
      }
      
      if (isEqual) 
      {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Password correct");
        delay(1500);
        
        // Open barrier
        lcd.clear();
        lcd.print("Opening barrier");
        myStepper.step(steps90Degrees);
        delay(1000);
        
        lcd.clear();
        lcd.print("Barrier open");
        
        
        while (true) {
          getDistance();
          if (distance > 20) {
            delay(3000); // Wait a few seconds after vehicle passes
            break;
          }
          delay(500);
        }
        
        // Close barrier
        lcd.clear();
        lcd.print("Closing barrier");
        myStepper.step(-steps90Degrees);
        delay(1000);
        
        lcd.clear();
        lcd.print("Barrier closed");
        delay(2000);
        lcd.clear();
        lcd.print("...Automatic...");
        lcd.setCursor(0, 1);
        lcd.print("....Barrier....");
        
        // Exit password loop and go back to main loop
        break;
      }
      else 
      {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Password wrong");
        delay(2000);
        // Loop continues, prompting for password again
      }
    }
  }
}

// method for reading distance using ultrasonic sensor
long getDistance() 
{
  distance = hc.dist();
  return distance;
}