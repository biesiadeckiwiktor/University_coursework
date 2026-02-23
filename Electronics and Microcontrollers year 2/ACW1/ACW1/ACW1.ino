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
int steps90Degrees = stepsPerRevolution / 4; // caounts per 1/4 turn (90 degrees)
int userInput[4];
int password[4] = {810,650,425,730}; //1 5 3 2 - vallues in array are analog input
long distance; // variable to store distance value from ultrasonic sensor
int sensorValue = 0;
float voltage = 0;

// pin setup
Stepper myStepper(stepsPerRevolution, 9, 11, 10, 12); //
HCSR04 hc(9, 6); //initialisation class HCSR04 (trig pin , echo pin)
LiquidCrystal lcd(8,7,5,4,3,2);

void setup() {
Serial.begin(9600);
lcd.begin(16, 2);
lcd.clear();
lcd.print("...Automatic...");
lcd.setCursor(0, 1);
lcd.print("....Barrier....");
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
  lcd.print("Please enter password");
  for (int i = 0, i <=3, i++)
  {
    sensorValue = analogRead(A0);
    userInput[i] = sensorValue;
  }
  bool isEqual = equal(begin(userInput), end(userInput), begin(password));

  if (isEqual) 
  {
  lcd.print("Password correct");
  myStepper.setSpeed(15);
  myStepper.step(steps90Degrees);
  }
  else 
  {
  lcd.print("Password incorrect");
  }
  

}
}

// method for reading distance using ultrasonic sensor
long getDistance() 
{
  distance = hc.dist();
  return distance;
}