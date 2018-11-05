// Developer: Brayden DeBoon
// email: brayden.deboon@uoit.net


// To run this experiment, you need the pendulum, arduino (obviously)
// a DC motor and DC motor driver. The ones used for the demo are similar to:

// Arduino: https://www.amazon.ca/ARDUINO-A000066-Uno-DIP-1-5/dp/B008GRTSV6/ref=sr_1_13?ie=UTF8&qid=1541450925&sr=8-13&keywords=arduino
// DC Motor Driver: https://www.amazon.ca/DROK-Industrial-Controller-Optocoupler-Isolation/dp/B06XGD5SCB/ref=sr_1_4_m?s=hi&ie=UTF8&qid=1541451211&sr=1-4&keywords=L298
// DC Motor: https://www.amazon.ca/RS-550-12-24V-Various-Cordless-Electric/dp/B075LC415S/ref=sr_1_1?ie=UTF8&qid=1541451297&sr=8-1&keywords=RS550
// Encoders: https://www.digikey.ca/product-detail/en/cui-inc/AMT102-V/102-1307-ND/827015
// We used random objects around the lab to make the pendulum, as MECE's I assume you can all build one
// Motor base voltage was 23V. 

//Define our motor OUTPUT pins.

#define motor_cw 9 //PWM pin on arduino for clockwise rotation
#define motor_ccw 10 //PWM pin for counter-clockwise rotation
#define encoder0PinA  2 //define channel A of our encoder (pendulum)
#define encoder0PinB  5 //define channel B of encoder (pendulum)
#define motor_enable 11 //motor contorller enable pin (Digital OUT)

int motor_voltage; //create global variable for motor voltage
double encoder0Pos = 0; //global variable for pendulum angle encoder data

// With respect to angle
double error = 0; //Error between measured and desired
double previousError; //For calculating derivative of error
double totalError = 0; //For calculating intergral of error
double desired_angle = 0; //What angle do we want

//Initialize starting angle (zero degrees) and cart position
double angle = 0;

//Controller Gains (PID Controller)
double Kp = 80; // Proportional Gain
double Ki = 0.1; // Integral Gain '1/s'
double Kd = 8; // Derivative Gain 's'
//Note that these gains will be different for different pendulums!

int motor_adjust;

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(9600); //Uncomment if you want to visualize data through serial port
  pinMode(11, OUTPUT); //Enable pin for motor driver
  pinMode(motor_cw, OUTPUT);
  pinMode(motor_ccw, OUTPUT);
  pinMode(encoder0PinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoder0, RISING);

  digitalWrite(motor_enable, HIGH); // Set enable to HIGH, activates driver
  analogWrite(motor_cw, LOW); //Make sure motors start 'off'
  analogWrite(motor_ccw, LOW);
}

void loop() {
  // The following line calculates pendulum position from the encoder
  // data. The encoder has 2048 pulses per revolution, the pendulum's
  // real angle in degrees = 360degrees*(encoder0Pos / 2048) 
  angle = (encoder0Pos / 2048) * 360; 
  //Serial.println(angle); // Uncomment if you want to get a visual on angle
  previousError = error; // This is for derivative calculation
  error = desired_angle - angle; //Want angle to be zero!!!!!!!!!!!!!

  if (abs(error) > 1) {
    // This statement calculates the accumulative error over time
    // i.e. the integral of the error saved as 'totalError'
    totalError += error; 
  }
  // THE CONTROLLER 
  motor_voltage = (Kp * error) + (Ki * totalError) + (Kd * (error - previousError));

  // NOTES:
  //
  // Proportional part: (Kp * error)
  // Integral part: (Ki * totalError)
  // Derivative part: (Kd * (error - previousError) <- change over time
  
  //Serial.print(" Voltage = ");
  //Serial.println(angle); // Uncomment for voltage data

  // If proposed voltage positive, turn motor counter clockwise
  if (motor_voltage > 0) { //
    analogWrite(motor_cw, LOW); //Makes sure no clockwise signal
    //The next 'if' statement double checks that the suggested voltage 
    //from the controller is feasible. The value has to be between 0 and 255 for
    // Arduino to use analogWrite (Our PWM signal).
    if (motor_voltage > 255) {
      motor_voltage = 255;
    }

    // The next line writes our control input to our motor driver
    analogWrite(motor_ccw, motor_voltage);
  }
  
  // Similar thing but in the clockwise direction
  if (motor_voltage < 0) {
    analogWrite(motor_ccw, LOW);
    if (motor_voltage < -255) {
      motor_voltage = -255;
    }
    motor_voltage = motor_voltage * -1; // Ensures 0 < motor_voltage < 255
    analogWrite(motor_cw, motor_voltage); //Writes to Analog Pin
  }

}

// This function reads encoder data through what is called an
// Interrupt pin. Look this up if youre interested on reading encoder data.
void doEncoder0() {
  if (digitalRead(encoder0PinB) == LOW) {
    encoder0Pos++; //If rotating one way
  }
  else {
    encoder0Pos--; // If rotating the other way
  }
}

