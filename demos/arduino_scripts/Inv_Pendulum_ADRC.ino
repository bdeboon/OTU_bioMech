
// ADRC Controller Adruino Script
//Define your OUTPUT pins.
// This script will use a motor as our ouput, through a L298N Driver 

#define motor_cw 9
#define motor_ccw 10

int motor_voltage; //create global variable for motor voltage

// Two encoders for inverted pendulum (pendulum angle and cart)
//Define you INPUT pins (we will be using two quadrature encoders)
#define encoder0PinA  2 //define channel A of our encoder 0 
#define encoder0PinB  5 //define channel B of encoder 0
#define encoder1PinA  3 //define channel A of our encoder 1
#define encoder1PinB  7 //define channel B of encoder 1

double encoder0Pos = 0; //Pendulum angle encoder value
double encoder1Pos = 0; //Cart position encoder value


// Place your references here!
double pendulum_reference = 0;
double cart_reference = 0;

//With respect to Track and velocity
double cart_pos = 0;

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(9600);
  pinMode(A0, INPUT);
  pinMode(11, OUTPUT);
  pinMode(motor_cw, OUTPUT);
  pinMode(motor_ccw, OUTPUT);
  pinMode(encoder0PinA, INPUT_PULLUP);
  pinMode(encoder0PinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoder0, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), doEncoder1, RISING);


  digitalWrite(11, HIGH);
  analogWrite(motor_cw, LOW);
  analogWrite(motor_ccw, LOW);
  angle_cal = analogRead(A0);
}

void loop() {
  // put your main code here, to run repeatedly:
  // Place your reference here!
  desired_angle = reference + 3 * (angle_adj / 1024);

  cart_pos = -1 * encoder1Pos / 51.42; //Cart Pos in mm

  //Serial.println(desired_angle);

  angle = (encoder0Pos / 2048) * 360;
  //Serial.println(angle);
  previousError = error;
  error = desired_angle - angle; //Want angle to be zero

  if (abs(error) > 1) {
    totalError += error;
  }
  if (abs(cart_pos) > 100) {
    total_cart_error += cart_pos;
  }

  motor_voltage = (Kp * error) + (Ki * totalError) + (Kd * (error - previousError));

  motor_adjust = Kp_cart * cart_pos + Ki_cart * total_cart_error;

  motor_voltage = round(motor_voltage + motor_adjust);



  //Serial.print("Cart Pos: ");
  //Serial.print(cart_pos);
  //Serial.print(" Voltage = ");
  //Serial.println(angle);


  if (motor_voltage > 0) {
    analogWrite(motor_cw, LOW);
    if (motor_voltage > 255) {
      motor_voltage = 255;
    }
    analogWrite(motor_ccw, motor_voltage);
  }

  if (motor_voltage < 0) {
    analogWrite(motor_ccw, LOW);
    if (motor_voltage < -255) {
      motor_voltage = -255;
    }
    motor_voltage = motor_voltage * -1;
    analogWrite(motor_cw, motor_voltage);
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
void doEncoder1() {
  if (digitalRead(encoder1PinB) == LOW) {
    encoder1Pos++;
  }
  else {
    encoder1Pos--;
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
