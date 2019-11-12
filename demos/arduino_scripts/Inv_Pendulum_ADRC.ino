
// ADRC Controller Adruino Script
// Define your OUTPUT pins.
// This script will use a motor as our ouput, through a L298N Driver
// We are trying to control a cart position! Your applications can be different!

#define motor_cw 9
#define motor_ccw 10

int motor_voltage; //create global variable for motor voltage

// Two encoders for inverted pendulum (pendulum angle and cart)
//Define you INPUT pins (we will be using two quadrature encoders)
#define encoder0PinA  2 //define channel A of our encoder 0 
#define encoder0PinB  5 //define channel B of encoder 0

double encoder0Pos = 0; //Cart angle encoder value

// Place your reference initial conditions here!
double cart_reference = 0;

// Add in timing paramters!
unsigned long currrent_time; // For current time
unsigned long old_time; // For calculating time derivatives

// Add in ADRC gains!
double h = 0.002; //Estimated time step (time it takes to run one loop())
double v_1 = 0; // Updated reference from transient profile
double v_2 = 0; // Time varying reference from transient profile
double z_1 = 0; // Observed state 1 (The state we are tryig to control)
double z_2 = 0; // First time derivative of observed state 1
double z_3 = 0; // Total disturbance term for dual integral plant
// Linear estimation of input proportionality
double b_0 = 0.2; //i.e. current to torque ratio for a motor, after gear reduction!

///////////////////////////////////////////////////////////
// TUNABLE ADRC GAINS! ADJUST THESE IF NEEDED
// Trasient profile generator aggressiveness factor
// Lower r_0 means more aggressive transient profile
double r_0 = 50; 
// Extended state observer gain 1 
double beta_01 = 1; 
// Extended state observer gain 2
double beta_02 = 1/(3*h); 
// Extended state observer gain 3
double beta_03 = 1/(64*h*h); 
// Nonlinear error function gains
gamma_1 = 0.5;
gamma_2 = 0.25;
// Nonlinear Feedback combiner gains
double r_1 = 50; // Feedback Comb. Aggression
double h_1 = 0.008; // Precision factor (MAIN TUNING PARAMETER) Low h = more aggressive
double c_1 = 0.75; // Fine tuning parameter
///////////////////////////////////////////////////////////////


void setup() {
  // Put your setup code here, to run once:
  old_time = millis(); //Initialize old_time counter
  
  // Initialize all inputs and outputs
  
  //Initialize Output
  pinMode(motor_cw, OUTPUT);
  pinMode(motor_ccw, OUTPUT);
  
  //Initialize Input
  pinMode(encoder0PinA, INPUT_PULLUP);
  pinMode(encoder0PinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoder0, RISING);
  
  //Ensure motors are off to begin!
  analogWrite(motor_cw, LOW);
  analogWrite(motor_ccw, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:

  currrent_time = millis(); //Get current time in milliseconds
  
  // Place your reference here!
  // Our reference will be a sinusoidal cart position between 10cm and -10cm
  cart_reference = 0.1*sin(0.4*(current_time/1000)); 

  //Relates the encoder count to meters
  cart_pos = (-1 * encoder0Pos / 51.42)/1000; //Cart Pos in meters
  
  
  // *********Now for the ADRC!!**************
  // Place your reference everywhere there is a "cart_reference"
  // Place your measured input everywhere there is a "cart_pos"
  h = current_time - old_time;
  // Transient Profile generator!
  v_1 = v_1 + h*v_2; // Update transient profile reference
  v_2 = v_2 + h*fhan(v_1 - cart_reference, v_2, r_0, h) //Update derivative of transient profile

  // Extended State Observer!
  // Fist observed state update from extended state observer
  z_1 = z_1 + h*z_2 - beta_01*(z_1 - cart_pos);     
  // Second observed state update from extended state observer
  z_2 = z_2 + h*(z_3 + b_0*motor_voltage) - beta_02*fal((z_1 - cart_pos),gamma_1,h);
  // Total disturbance term
  z_3 = z_3 - beta_03*fal((z_1 - cart_pos), gamma_2,h);
  
  //Nonlinear feedback combiner!
  motor_voltage = -(fhan((v_1-z_1),c_1*(v_2-z_2),r_1,h_1) + z_3)/b_0;
  
  
  ///////////// Thats it for ADRC! /////////////////////////////////
  // The rest of the script sends the found input to the motors! Easy as pie!
  motor_adjust = round((motor_voltage/12)*255); //Adjust motor voltage to Arduino PWM max/min 
  old_time = current_time;

  if (motor_adjust > 0) { // Forward Direction
    analogWrite(motor_cw, LOW);
    if (motor_adjust > 255) { // Input saturation
      motor_adjust = 255;
    }
    analogWrite(motor_ccw, motor_adjust);
  }

  if (motor_adjust < 0) { // Reverse Direction
    analogWrite(motor_ccw, LOW);
    if (motor_adjust < -255) { // Input saturation
      motor_adjust = -255;
    }
    motor_adjust = motor_adjust * -1;
    analogWrite(motor_cw, motor_adjust);
  }
  

}

void doEncoder0() {
  if (digitalRead(encoder0PinB) == LOW) {
    encoder0Pos++;
  }
  else {
    encoder0Pos--;
  }
}

double fal_(double e, double alpha, double delta) {
  
  double fal;
  if (abs(e) <= delta) {
    fal = e/(pow(delta,(1-alpha)));
  }
  else {
    fal = (pow(abs(e), alpha))*sign(e);
  }
  
  return fal;
}

double sign(double val) {
  if (val < 0) {return -1;}
  if (val == 0) {return 0;}
  return 1;  
}

double fhan(double v_1, double v_2, double r_0, double h_0) {

  double d, a_0, y, a_1, a_2, s_y, s_a, a, fhan;

  d = h_0*pow(r_0, 2);
  a_0 = h_0*v_2;
  y = v_1 + a_0;
  a_1 = sqrt(d*(d+8*abs(y)));
  a_2 = a_0 + (sign(y)*(a_1 - d))/2;
  s_y = (sign(y + d) - sign(y - d))/2;
  a = (a_0 + y - a_2)*s_y + a_2;
  s_a = (sign(a+d)-sign(a-d))/2;
  fhan = -r_0*(((a/d)-sign(a))*s_a)-(r_0*sign(a));
  return fhan;

}
