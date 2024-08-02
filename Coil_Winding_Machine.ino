#include <AccelStepper.h>
#include <LiquidCrystal_I2C.h>

// the comments are in english because arduino had problems with having georgian text in it 

// stepper pins
const int stepPin = 6;  // Connect to driver's Step input
const int dirPin = 4;   // Connect to driver's Direction input
const int enablePin = 5; // Connect to driver's Enable input (if available)

AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

const byte i2cAddress = 0x27;

LiquidCrystal_I2C lcd(i2cAddress, 16, 2);

// button pins
const int startButton = 8;
const int stopButton = 9;
const int increaseButton = 10;
const int decreaseButton = 11;

// potentiometer pin
const int potPin = A0;
const int hallPin = 2;
// pin that powers the transistor base for the dc motor
const int dcPin = 3;

//steps per revolution of the stepper motor
const double revolution = 200.0;

long lastHallTime = 0;

long lastPrtTime = 0;
//Cylinder Length
const double length = 100.0;

// how much actuator travels per one revolution of the rail
const double distancePerRev = 8.0;

// diameter of the coil
const double wireDiameter = 0.8;

int desiredRpm = 0;
int currentRpm = 0;


//amount of steps stepper has to make to traverse the whole cylinder
double amountOfSteps = (length / distancePerRev) * revolution;
const double distance = amountOfSteps;
// distance the actuator moves per one revolution of the cylinder
double revsPerSec = desiredRpm / 60.0;

double distPerSec = (double)(revsPerSec * wireDiameter);

int stepsPerSec = ((revolution * distPerSec) / distancePerRev);
double timeRequired = (amountOfSteps / stepsPerSec) * 1000;
int maxRpm = 114;
// position of the actuator relative to the starting point (measured in steps of the motor)
long position = 0;
// previous position ( used for detecting when one step was made)
long lastPosition = 0;
// state of the program: if it is false, it is either paused or in setup mode, if it is true, it is currently running
bool winding = false;
int currentTurns = 0;
int hallCount = 0;
// number of coils
int numberOfTurns = 0;

// direction to which the stepper motor moves (1 means to the left, -1 means to the right)
int direction = 1;
int lastValue = 0;
int speed; 
// last time increase and decrease buttons weren't pressed 
long lastHighIncrTime = 0;
long lastHighDecrTime = 0;
double realPeriod = 0;
long lastTurnTime = 0;
bool periodMeasure = false;
long lastMeasureTime = 0;


double kp = 0.2; 
double ki = 0.05; 
double kd = 0; 
double maxIntegral = 100000.0; 
double lastError = 0;        
double integral = 0;         
long lastTime = 0;


void setup() {
  initialize();
  initButtons();
  initLcd();
}

// initalizes lcd
void initLcd() {
  lcd.init();
  lcd.backlight();
  lcdWelcome();
}
// initalizes arduino code;
void initialize() {
  Serial.begin(9600);
  pinMode(enablePin, OUTPUT);
  pinMode(hallPin, INPUT_PULLUP);
  digitalWrite(enablePin, LOW);
  attachInterrupt(digitalPinToInterrupt(2), hall, FALLING);
  stepper.setCurrentPosition(0);
  stepper.setAcceleration(100);
  stepper.setMaxSpeed(2 * stepsPerSec);
}
// initializes buttons
void initButtons() {
  pinMode(startButton, INPUT_PULLUP);
  pinMode(stopButton, INPUT_PULLUP);
  pinMode(decreaseButton, INPUT_PULLUP);
  pinMode(increaseButton, INPUT_PULLUP);
}
void lcdWelcome() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Automatic  Coil");
  lcd.setCursor(0, 1);
  lcd.print("Winding Machine");
}
void hall() {
  if (millis() - lastHallTime >= 200) {
    hallCount++;
    lastHallTime = millis();
    lastMeasureTime = millis();
  }
}


void loop() {
  if (digitalRead(startButton) == LOW) {
    startWinding();
  }

  if (digitalRead(stopButton) == LOW) {
    stopWinding();
  }

  if (digitalRead(increaseButton) == HIGH) {
    lastHighIncrTime = millis();
  }

  if (digitalRead(decreaseButton) == HIGH) {
    lastHighDecrTime = millis();
  }

  if (digitalRead(increaseButton) == LOW) {
    increaseTurns();
    delay(200);
  }

  if (digitalRead(decreaseButton) == LOW) {
    decreaseTurns();
    delay(200);
  }
  if (abs(analogRead(A0) - lastValue) > 100) {
    changeSpeed();
    lastValue = analogRead(A0);
  }
  if (winding)
    wind();

  Serial.print(desiredRpm);
  Serial.print('\t'); 
  Serial.print(currentRpm);
  Serial.print('\t'); 
  Serial.println(hallCount);

}

// code for winding the coils
void wind() {
  desiredRpm = max(desiredRpm,0);
  update();
  stepper.setMaxSpeed(2 * stepsPerSec);
  // changes the stepper motor speed according to the rpm of the dc motor 
  // stops the winding if the desired amount of coils is reached
  if (hallCount == numberOfTurns) {
    stopWinding();
    return;
  }
  // makes the stepper turn in the opposite direction
  if (amountOfSteps == 0) {
    amountOfSteps = distance;
    stepper.stop();
    if (direction == 1)
      stepper.moveTo(0);
    else stepper.moveTo(distance);
    direction *= -1;
    return;
  }
  
  pidControl();
  if (abs(stepper.currentPosition() - lastPosition) == 1) {
    lastPosition = stepper.currentPosition();
    amountOfSteps--;
  }
  long periodMillis = (1 / (currentRpm / 60) * 1000);
  if (millis() - lastTurnTime >= periodMillis) {
    lastTurnTime = millis();
    currentTurns++;
  }
  if (millis() - lastPrtTime > 200) {
    lastPrtTime = millis();
    lcdPrint(0, "Turns: " + String(hallCount) + "/" + String(numberOfTurns));
    lcd.setCursor(0, 1);
    lcd.print("Speed: " + String(currentRpm) + "/" + String(desiredRpm));
  }
  stepper.run();
}
void pidControl() {

      
    double error = desiredRpm - currentRpm;
        // Serial.println("error: " + String(error));

    long now = millis();

    double interval = (now - lastTime); 
    // Serial.println("interval: " + String(interval));
    if (interval <= 0) {
        return;
    }
    integral += error * interval;

    if (integral > maxIntegral) {
        integral = maxIntegral;
    } else if (integral < -maxIntegral) {
        integral = -maxIntegral;
    }
    
            // Serial.println("integral: " + String(integral));

    double derivative = (error - lastError) / interval;

     double output = kp * error + ki * integral + kd * derivative;


    // if (desiredRpm == currentRpm) {
    //     integral = 0;
    // }
    output = constrain(output,0,255);
    // if (output > maxRpm) {
    //     output = maxRpm;
    // } else if (output < 0) {
    //     output = 0;
    // }
    // Serial.println("output: " + String(output));
    int pwmOutput = (output * 255) / maxRpm;
    
    if (pwmOutput > 255) {
        pwmOutput = 255;
    } 
    else if (pwmOutput < 0) {
        pwmOutput = 0;
    }
    currentRpm = constrain(output,0,maxRpm);
    analogWrite(dcPin, pwmOutput);
    lastError = error;
    lastTime = now;
}


// prints on lcd
void lcdPrint(int row, String word) {
  if (word.length() > 16)
    return;
  lcd.clear();
  lcd.setCursor(0, row);
  lcd.print(word);
}
// relationship between voltage applied to the dc motor and rpm. 
// because our dc motor is not ideal, we measured rpm at various voltages with a hall sensor 
// we tried using linear regression, but the error was too big
// so we used different functions on different intervals
double mapRpm(double volts) {
  if (volts <= 4) {
    return 20 * volts - 10;
  } else if (volts <= 6) {
    return 10 * volts + 32;
  } else if (volts <= 9) {
    return 5 * volts + 62.5;
  } else {
    return 2 * volts + 89;
  }
}
// updates the important global variables
void update() {
  revsPerSec = currentRpm / 60;
  distPerSec = (double)(revsPerSec * wireDiameter);
  stepsPerSec = (int)((revolution * distPerSec) / distancePerRev);
}
// updates global variables and shows rpm on lcd 
void changeSpeed() {
  speed = analogRead(potPin);
  double volts = (analogRead(potPin) / 1023.0) * 12;
  desiredRpm = mapRpm(volts);
 // currentRpm = 0;
//  update();
  String s = "Speed: " + String((int)desiredRpm) + "/RPM";
  lcdPrint(0, s);
}
// starts the winding process if the number of turns is higher than 0
void startWinding() {
  if (!winding) {
    if (numberOfTurns > 0) {
      winding = true;
      lastTurnTime = millis();
      lcdPrint(0, "Starting Winding");
     analogWrite(3, constrain(desiredRpm/2, 0,255));
      stepper.setMaxSpeed(2 * stepsPerSec);
      stepper.moveTo(amountOfSteps);
    }
  }
}
// stops the winding process
void stopWinding() {
  if (winding) {
    if (hallCount >= numberOfTurns) {
      hallCount = 0;
      numberOfTurns = 0;
    }
    winding = false;
    stepper.stop();
    analogWrite(dcPin, 0);
    
    lastHighDecrTime = millis();
    lastHighIncrTime = millis();
    lcdPrint(0, "Winding stopped");
  }
}

// increases the amount of turns when pressing the increase button.  if the user releases the button quickly, increases by one, 
//if the user holds the button for too long, it increases more quickly, 
void increaseTurns() {
  if (winding) {
    lcdPrint(0, "Pause first");
    return;
  }

  if (millis() - lastHighIncrTime > 1000)
    numberOfTurns += 5;
  else {
    numberOfTurns++;
  }
  String s = "Turns: " + String(numberOfTurns);
  lcdPrint(0, s);
}

// decreases the amount of turns when pressing the decrease button.
void decreaseTurns() {
  if (winding) {
    lcdPrint(0, "Pause first");
    return;
  }
  if (numberOfTurns > 1) {
    if (millis() - lastHighDecrTime > 1000)
      numberOfTurns = numberOfTurns - 5;
    else {
      numberOfTurns--;
    }
    numberOfTurns = max(0, numberOfTurns);
    String s = "Turns: " + String(numberOfTurns);
    lcdPrint(0, s);
  }
}
