
#include <Wire.h>
#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
 
//initializing all the variables
#define LOOPTIME                      100     //Looptime in millisecond
const byte noCommLoopMax = 10;                //number of main loops the robot will execute without communication before stopping
unsigned int noCommLoops = 0;                 //main loop without communication counter

double speed_cmd_left2 = 0;      

const int PIN_ENCOD_A_MOTOR_LEFT = 2;               //A channel for encoder of left motor                    
const int PIN_ENCOD_B_MOTOR_LEFT = 8;               //B channel for encoder of left motor

const int PIN_ENCOD_A_MOTOR_RIGHT = 3;              //A channel for encoder of right motor         
const int PIN_ENCOD_B_MOTOR_RIGHT = 9;              //B channel for encoder of right motor 

// Pin variables for motors.
const int right_pwm_fwd = 10 ;
const int right_pwm_bwd = 11;

const int left_pwm_fwd =12 ;
const int left_pwm_bwd =7;


unsigned long lastMilli = 0;

//--- Robot-specific constants ---
const double radius = 0.0525;                   //Wheel radius, in m
const double wheelbase = 0.42;               //Wheelbase, in m
const double encoder_cpr = 168;               //Encoder ticks or counts per rotation
const double speed_to_pwm_ratio = 0.00369;    //Ratio to convert speed (in m/s) to PWM value. It was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the slope of the linear function).
const double min_speed_cmd = 0.20295;          //(min_speed_cmd/speed_to_pwm_ratio) is the minimum command value needed for the motor to start moving. This value was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the constant of the linear function).
const double min_speed_cmd = 0.2214;     // for 60 pwm


double speed_req = 0;                         //Desired linear speed for the robot, in m/s
double angular_speed_req = 0;                 //Desired angular speed for the robot, in rad/s

double speed_req_left = 0;                    //Desired speed for left wheel in m/s
double speed_act_left = 0;                    //Actual speed for left wheel in m/s
double speed_cmd_left = 0;                    //Command speed for left wheel in m/s 

double speed_req_right = 0;                   //Desired speed for right wheel in m/s
double speed_act_right = 0;                   //Actual speed for right wheel in m/s
double speed_cmd_right = 0;                   //Command speed for right wheel in m/s 
                        
const double max_speed = 0.4;                 //Max speed in m/s

int PWM_leftMotor = 0;                     //PWM command for left motor
int PWM_rightMotor = 0;                    //PWM command for right motor 
                                                      
// PID Parameters
const double PID_left_param[] = { 0.01, 0, 0.1 }; //Respectively Kp, Ki and Kd for left motor PID
const double PID_right_param[] = { 0.01, 0, 0.1 }; //Respectively Kp, Ki and Kd for right motor PID

//const double PID_left_param[] = { 0.6, 0.3, 0.5 }; //Respectively Kp, Ki and Kd for left motor PID
//const double PID_right_param[] = { 0.6, 0.3, 0.5 }; //Respectively Kp, Ki and Kd for right motor PID


volatile float pos_left = 0;       //Left motor encoder position
volatile float pos_right = 0;      //Right motor encoder position

PID PID_leftMotor(&speed_act_left, &speed_cmd_left, &speed_req_left, PID_left_param[0], PID_left_param[1], PID_left_param[2], DIRECT);          //Setting up the PID for left motor
PID PID_rightMotor(&speed_act_right, &speed_cmd_right, &speed_req_right, PID_right_param[0], PID_right_param[1], PID_right_param[2], DIRECT);   //Setting up the PID for right motor

  
ros::NodeHandle nh;

//function that will be called when receiving command from host
void handle_cmd (const geometry_msgs::Twist& cmd_vel) {
  noCommLoops = 0;                                                  //Reset the counter for number of main loops without communication
  
  speed_req = cmd_vel.linear.x;                                     //Extract the commanded linear speed from the message

  angular_speed_req = cmd_vel.angular.z;                            //Extract the commanded angular speed from the message
  
  speed_req_left = speed_req - angular_speed_req*(wheelbase/2);     //Calculate the required speed for the left motor to comply with commanded linear and angular speeds
  speed_req_right = speed_req + angular_speed_req*(wheelbase/2);    //Calculate the required speed for the right motor to comply with commanded linear and angular speeds

}

ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", handle_cmd);   //create a subscriber to ROS topic for velocity commands (will execute "handle_cmd" function when receiving data)
geometry_msgs::Vector3Stamped speed_msg;                                //create a "speed_msg" ROS message
ros::Publisher speed_pub("speed", &speed_msg);                          //create a publisher to ROS topic "speed" using the "speed_msg" type

void left_Fwd(const size_t speed) {
  analogWrite(left_pwm_fwd, speed);
  analogWrite(left_pwm_bwd, 0);
}
void left_Bwd(const size_t speed) {
  analogWrite(left_pwm_bwd, speed);
  analogWrite(left_pwm_fwd, 0);
  }
void left_Stop() {
  analogWrite(left_pwm_bwd, 0);
  analogWrite(left_pwm_fwd, 0);
  }

void right_Fwd(const size_t speed) {
  analogWrite(right_pwm_fwd, speed);
  analogWrite(right_pwm_bwd, 0);
}
void right_Bwd(const size_t speed) {
  analogWrite(right_pwm_bwd, speed);
  analogWrite(right_pwm_fwd, 0);
}
void right_Stop() {
    analogWrite(right_pwm_bwd, 0);
  analogWrite(right_pwm_fwd, 0);
}


//__________________________________________________________________________

void setup() {

  pinMode(right_pwm_fwd, OUTPUT);    // sets the digital pin 13 as output
  pinMode(right_pwm_bwd, OUTPUT);
  pinMode(left_pwm_fwd, OUTPUT);
  pinMode(left_pwm_bwd, OUTPUT);

  
  nh.initNode();                            //init ROS node
  nh.getHardware()->setBaud(57600);         //set baud for ROS serial communication
  nh.subscribe(cmd_vel);                    //suscribe to ROS topic for velocity commands
  nh.advertise(speed_pub);                  //prepare to publish speed in ROS topic
 
  
  //setting motor speeds to zero
  left_Stop();
  right_Stop();
  //setting PID parameters
  //PID_leftMotor.SetSampleTime(5);
  //PID_rightMotor.SetSampleTime(5);
  PID_leftMotor.SetOutputLimits(-max_speed, max_speed);
  PID_rightMotor.SetOutputLimits(-max_speed, max_speed);
  PID_leftMotor.SetMode(AUTOMATIC);
  PID_rightMotor.SetMode(AUTOMATIC);
    
  // Define the rotary encoder for left motor
  pinMode(PIN_ENCOD_A_MOTOR_LEFT, INPUT); 
  pinMode(PIN_ENCOD_B_MOTOR_LEFT, INPUT); 
  digitalWrite(PIN_ENCOD_A_MOTOR_LEFT, HIGH);                // turn on pullup resistor
  digitalWrite(PIN_ENCOD_B_MOTOR_LEFT, HIGH);
  attachInterrupt(0, encoderLeftMotor, RISING);

  // Define the rotary encoder for right motor
  pinMode(PIN_ENCOD_A_MOTOR_RIGHT, INPUT); 
  pinMode(PIN_ENCOD_B_MOTOR_RIGHT, INPUT); 
  digitalWrite(PIN_ENCOD_A_MOTOR_RIGHT, HIGH);                // turn on pullup resistor
  digitalWrite(PIN_ENCOD_B_MOTOR_RIGHT, HIGH);
  attachInterrupt(1, encoderRightMotor, RISING);
}

//_________________________________________________________________________

void loop() {
  nh.spinOnce();
  if((millis()-lastMilli) >= LOOPTIME)   
  {                                                                           // enter timed loop
    lastMilli = millis();
       
    if (abs(pos_left) < 5){                                                   //Avoid taking in account small disturbances
      speed_act_left = 0;
    }
    else {
      speed_act_left=((pos_left/encoder_cpr)*2*PI)*(1000/LOOPTIME)*radius;           // calculate speed of left wheel
    }
    
    if (abs(pos_right) < 5){                                                  //Avoid taking in account small disturbances
      speed_act_right = 0;
    }
    else {
    speed_act_right=((pos_right/encoder_cpr)*2*PI)*(1000/LOOPTIME)*radius;          // calculate speed of right wheel
    }

        // publishing encoder values for debugging the robot
          String LS = String(pos_left);
    String RS = String(pos_right);

    String FS= "LE  " + LS + "     " + "RE  "+ RS;
    char FA[FS.length() + 1];
    FS.toCharArray(FA,FS.length() + 1);

    nh.loginfo(FA);
    // debugging part complete
    
    pos_left = 0;
    pos_right = 0;

    speed_cmd_left = constrain(speed_cmd_left, -max_speed, max_speed);
    PID_leftMotor.Compute();                                                 
    // compute PWM value for left motor. Check constant definition comments for more information.
    PWM_leftMotor = constrain(((speed_req_left+sgn(speed_req_left)*min_speed_cmd)/speed_to_pwm_ratio) + (speed_cmd_left/speed_to_pwm_ratio), -255, 255); //

    speed_cmd_right = constrain(speed_cmd_right, -max_speed, max_speed);    
    PID_rightMotor.Compute();                                                 
    // compute PWM value for right motor. Check constant definition comments for more information.
    PWM_rightMotor = constrain(((speed_req_right+sgn(speed_req_right)*min_speed_cmd)/speed_to_pwm_ratio) + (speed_cmd_right/speed_to_pwm_ratio), -255, 255); // 


    
    /*
    // logging pid value for DEBUGGING ,  use rosserial_python serialnode.py port no 
    String LS = String(PWM_leftMotor);
    String RS = String(PWM_rightMotor);

    String FS= "LM  " + LS + "     " + "RM  "+ RS;
    
    char FA[FS.length() + 1];
    FS.toCharArray(FA,FS.length() + 1);
    
    nh.loginfo(FA);
    */  
    
    if (noCommLoops >= noCommLoopMax) {                   //Stopping if too much time without command
        left_Stop();

    }
    else if (speed_req_left == 0){                        //Stopping
      left_Stop();
    }
    else if (PWM_leftMotor > 0){    
        left_Bwd(abs(PWM_leftMotor));                   //Going forward
    }
    else {              
        left_Fwd(abs(PWM_leftMotor));                   //Going backward
      
    }
    

    if (noCommLoops >= noCommLoopMax) {                   //Stopping if too much time without command
      right_Stop();
    }
    else if (speed_req_right == 0){                       //Stopping
      right_Stop();
    }
    else if (PWM_rightMotor > 0){                         //Going forward
      right_Fwd(abs(PWM_rightMotor));
    }
    else {                                                //Going backward
      right_Bwd(abs(PWM_rightMotor));
    }

    if((millis()-lastMilli) >= LOOPTIME){         //write an error if execution time of the loop in longer than the specified looptime
      nh.loginfo(" TOO LONG ");
    }

    noCommLoops++;
    if (noCommLoops == 65535){
      noCommLoops = noCommLoopMax;
    }
    
    publishSpeed(LOOPTIME);   //Publish odometry on ROS topic
  }
 }

//Publish function for odometry, uses a vector type message to send the data (message type is not meant for that but that's easier than creating a specific message type)
void publishSpeed(double time) {
  speed_msg.header.stamp = nh.now();      //timestamp for odometry data
  speed_msg.vector.x = speed_act_left;    //left wheel speed (in m/s)
  speed_msg.vector.y = speed_act_right;   //right wheel speed (in m/s)
  speed_msg.vector.z = time/1000;         //looptime, should be the same as specified in LOOPTIME (in s)
  speed_pub.publish(&speed_msg);
  nh.spinOnce();
  nh.loginfo("Publishing odometry");
}

//Left motor encoder counter
void encoderLeftMotor() {
  if (digitalRead(PIN_ENCOD_A_MOTOR_LEFT) == digitalRead(PIN_ENCOD_B_MOTOR_LEFT)) pos_left++;
  else pos_left--;
}

//Right motor encoder counter
void encoderRightMotor() {
  if (digitalRead(PIN_ENCOD_A_MOTOR_RIGHT) == digitalRead(PIN_ENCOD_B_MOTOR_RIGHT)) pos_right--;
  else pos_right++;
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
