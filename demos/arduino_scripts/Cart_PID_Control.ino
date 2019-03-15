/*
   METE 3100U: PID Control of a motor that moves a cart.

   Microcontroller: Arduino Mega 2560

   Inputs:
      Potentiometer to analog pin sets desired position of the cart along the track.
      AMT102 Rotary Encoder coupled to motor shaft. Configured to 512 pulses/rev.

   Outputs:
      DROK L298 H-Bridge powered with 12V DC.

*/




// Define pin numbers.
#define motor_cw 9      // Define H-bridge clockwise enable.
#define motor_ccw 10    // Define H-bridge couter clockwise enable.
#define pot_in A0       // Analog input for potentiometer.
#define encoder0PinA  2 // Define channel A of our encoder
#define encoder0PinB  5 // Define channel B of encoder


// Declare Variables.
int motor_voltage; // Global variable for motor voltage.
int motor_adjust;  // Contains voltage delta to compensate for error.
int potValue;      // Store analog value of potentiometer voltage division.


double encoder0Pos = 0;      // Global variable for encoder pulses.
double cart_pos_Actual = 0;  // Actual position of the cart, based off of encoder reading.
double cart_pos_Desired = 0; // Desired cart position, based off of the potentiometer input.

double error = 0;       // Instantaneous error.
double previousError;   // Last saved error value.
double totalError = 0;  // Cumulative error.

double Kp_cart = 30;   // Proportional Gain for Cart Position
double Ki_cart = 0.15; // Integral Gain for Cart Position
double Kd_cart = 40;   // Derivative Gain for Cart Position









// ****************************************** SETUP ******************************************
void setup() {
  Serial.begin(9600);

  // Enable I/O pins.
  pinMode(motor_cw, OUTPUT);
  pinMode(motor_ccw, OUTPUT);
  pinMode(pot_in, INPUT);

  analogWrite(motor_cw, LOW);
  analogWrite(motor_ccw, LOW);

  // Attach an interrupt to the encoder pulse pin.
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoder0, RISING);

  // Make initial voltage is zero.
  motor_voltage = 0;
}







// ****************************************** LOOP ******************************************
void loop()
{

  // Update Cart Position.
  cart_pos_Actual = encoder0Pos / 12.33; // Position converted to mm.



  // Read in desired position from potentiometer. Map the value to 0 - 300mm on cart track.
  potValue = analogRead(pot_in);
  cart_pos_Desired = map(potValue, 0, 1024, 0, 300);



  // Collect error values. Store the previous. Delta position error. Culmative error if significant.
  previousError = error;

  error = cart_pos_Desired - cart_pos_Actual;

  if (abs(error) > 1)
    totalError += error;




  // Update control parameters.
  motor_voltage = (Kp_cart * error) + (Ki_cart * totalError) + (Kd_cart * (error - previousError));



  // Adjust the motor voltage to fit the bounds. Lower bound.
  if (motor_voltage > 0)
  {
    analogWrite(motor_cw, LOW);
    if (motor_voltage > 255)
    {
      motor_voltage = 255;
    }

    // Output the desired voltage to the motor using PWM to H-bridge. Enable ccw rotation.
    analogWrite(motor_ccw, motor_voltage);
  }



  // Adjust the motor voltage to fit the bounds. Upper bound.
  if (motor_voltage < 0)
  {
    analogWrite(motor_ccw, LOW);
    if (motor_voltage < -255)
    {
      motor_voltage = -255;
    }

    // Output the desired voltage to the motor using PWM to H-bridge. Enable cw rotation.
    motor_voltage = motor_voltage * -1;
    analogWrite(motor_cw, motor_voltage);
  }

  // Minor delay.
  delay(20);
}





// ****************************************** doEncoder0 ******************************************
// If a change in the pulses is read update the encoder count. 
void doEncoder0() 
{
  if (digitalRead(encoder0PinB) == LOW) 
    encoder0Pos++;
  else 
    encoder0Pos--;
}
