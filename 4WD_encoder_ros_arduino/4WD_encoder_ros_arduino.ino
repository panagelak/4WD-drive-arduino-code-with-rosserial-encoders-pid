#define USE_USBCON
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <quadrature.h>
#include <geometry_msgs/Twist.h>
#include <PID_v1.h>

// Initialize PID paramaters

double Setpoint_fl, Input_fl, Output_fl;
double Setpoint_fr, Input_fr, Output_fr;
double Setpoint_bl, Input_bl, Output_bl;
double Setpoint_br, Input_br, Output_br;

double aggKp=550, aggKi=280, aggKd=2;
double consKp=300, consKi=120, consKd=0.25;

PID myPID_fl(&Input_fl, &Output_fl, &Setpoint_fl, aggKp, aggKi, aggKd, DIRECT);
PID myPID_fr(&Input_fr, &Output_fr, &Setpoint_fr, aggKp, aggKi, aggKd, DIRECT);
PID myPID_bl(&Input_bl, &Output_bl, &Setpoint_bl, aggKp, aggKi, aggKd, DIRECT);
PID myPID_br(&Input_br, &Output_br, &Setpoint_br, aggKp, aggKi, aggKd, DIRECT);

// Initialize quadrature encoder paramaters

Quadrature_encoder<46,47> encoder_fright(Board::due);
Quadrature_encoder<48,49> encoder_fleft(Board::due);
Quadrature_encoder<42,43> encoder_bright(Board::due);
Quadrature_encoder<44,45> encoder_bleft(Board::due);

// Initialize pin numbers

const uint8_t RF_PWM = 11;
const uint8_t RF_BACK = 27;
const uint8_t RF_FORW = 26;
const uint8_t LF_BACK = 24;
const uint8_t LF_FORW = 25;
const uint8_t LF_PWM = 10;

const uint8_t RB_PWM = 13;
const uint8_t RB_BACK = 30;
const uint8_t RB_FORW = 31;
const uint8_t LB_BACK = 29;
const uint8_t LB_FORW = 28;
const uint8_t LB_PWM = 12;

// speed = 0? help? pid not reset?
bool wtf;



// Initialize ROS paramaters

ros::NodeHandle nh;

std_msgs::Int32 lfcount;
std_msgs::Int32 rfcount;
std_msgs::Int32 lbcount;
std_msgs::Int32 rbcount;

ros::Publisher lfwheel("lfwheel", &lfcount);
ros::Publisher rfwheel("rfwheel", &rfcount);
ros::Publisher lbwheel("lbwheel", &lbcount);
ros::Publisher rbwheel("rbwheel", &rbcount);

// Cmd_vel Callback
// Sets the setpoints of the pid for each wheel

void onTwist(const geometry_msgs::Twist &msg)
{
  //nh.loginfo("Inside Callback");
  float x = msg.linear.x;
  float z = msg.angular.z;
  float w = 0.31;
  if(!(x==0 && z==0)){
  wtf=false;
  float fright = x + (z * w / 2.0)/0.11;
  float fleft = x - (z * w / 2.0)/0.11;
  float bright = x + (z * w / 2.0)/0.11;
  float bleft = x - (z * w / 2.0)/0.11;
  Setpoint_fl = fleft;
  Setpoint_fr = fright;
  Setpoint_bl = bleft;
  Setpoint_br = bright;
  }
  else{
    wtf=true;
    Setpoint_fl = 0;
    Setpoint_fr = 0;
    Setpoint_bl = 0;
    Setpoint_br = 0;
    }
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &onTwist);

// Move any motor function with speed_pwm value and pin numbers

void Move_motor(int speed_pwm,const uint8_t pwm,const uint8_t forw,const uint8_t back)
{
  if(speed_pwm >= 0)
  {
    digitalWrite(forw, HIGH);
    digitalWrite(back, LOW);
    analogWrite(pwm, abs(speed_pwm));
  }
  else if(speed_pwm < 0)
  {
    digitalWrite(forw, LOW);
    digitalWrite(back, HIGH);
    analogWrite(pwm, abs(speed_pwm));
  }
}



// Initialize pins for forward movement

void setpins()
{
  pinMode(LF_FORW,OUTPUT);
  pinMode(LF_BACK,OUTPUT);
  pinMode(RF_FORW,OUTPUT);
  pinMode(RF_BACK,OUTPUT);
  pinMode(LF_PWM,OUTPUT);
  pinMode(RF_PWM,OUTPUT);
  pinMode(LB_FORW,OUTPUT);
  pinMode(LB_BACK,OUTPUT);
  pinMode(RB_FORW,OUTPUT);
  pinMode(RB_BACK,OUTPUT);
  pinMode(LB_PWM,OUTPUT);
  pinMode(RB_PWM,OUTPUT);

  digitalWrite(RF_FORW, HIGH);
  digitalWrite(RF_BACK, LOW);
  digitalWrite(LF_FORW, HIGH);
  digitalWrite(LF_BACK, LOW);
  digitalWrite(RB_FORW, HIGH);
  digitalWrite(RB_BACK, LOW);
  digitalWrite(LB_FORW, HIGH);
  digitalWrite(LB_BACK, LOW);
}

// Encoders tend to reverse regarding of the pins??
// This way we move the robot forward a bit on startup
// And if an encoder has negative value we reverse it.

void fix_encoder_ori_on_start(){

  analogWrite(RF_PWM, 180);
  analogWrite(LF_PWM, 180);
  analogWrite(RB_PWM, 180);
  analogWrite(LB_PWM, 180);
  
  delay(150);

  analogWrite(RF_PWM, 0);
  analogWrite(LF_PWM, 0);
  analogWrite(RB_PWM, 0);
  analogWrite(LB_PWM, 0);

  int ct1 = encoder_fleft.count();
  int ct2 = encoder_fright.count();
  int ct3 = encoder_bleft.count();
  int ct4 = encoder_bright.count();
  if(ct1 < 0) {encoder_fleft.reverse();}
  if(ct2 < 0) {encoder_fright.reverse();}
  if(ct3 < 0) {encoder_bleft.reverse();}
  if(ct4 < 0) {encoder_bright.reverse();}
  
}

// stop movement

void stop()
{
  digitalWrite(LF_FORW, 0);
  digitalWrite(LF_BACK, 0);
  digitalWrite(RF_FORW, 0);
  digitalWrite(RF_BACK, 0);
  analogWrite(LF_PWM, 0);
  analogWrite(RF_PWM, 0);
  digitalWrite(LB_FORW, 0);
  digitalWrite(LB_BACK, 0);
  digitalWrite(RB_FORW, 0);
  digitalWrite(RB_BACK, 0);
  analogWrite(LB_PWM, 0);
  analogWrite(RB_PWM, 0);

}

void setup() {

  // 115200 baud rate
  nh.getHardware()->setBaud(115200);

  // Pid setup
  
  myPID_fl.SetOutputLimits(-255, 255);
  myPID_fr.SetOutputLimits(-255, 255);
  myPID_bl.SetOutputLimits(-255, 255);
  myPID_br.SetOutputLimits(-255, 255);

  myPID_fl.SetMode(AUTOMATIC);
  myPID_fr.SetMode(AUTOMATIC);
  myPID_bl.SetMode(AUTOMATIC);
  myPID_br.SetMode(AUTOMATIC);

  myPID_fl.SetSampleTime(25);
  myPID_fr.SetSampleTime(25);
  myPID_bl.SetSampleTime(25);
  myPID_br.SetSampleTime(25);

  // Encoder setup
  
  encoder_fright.begin();
  encoder_fleft.begin();
  encoder_bright.begin();
  encoder_bleft.begin();
  
  // setup pins and fix encoders
   
  setpins();
  fix_encoder_ori_on_start();

  stop();

  // ros node setup
  
  nh.initNode();
  nh.advertise(lfwheel);
  nh.advertise(rfwheel);
  nh.advertise(lbwheel);
  nh.advertise(rbwheel);
  nh.subscribe(sub);
  
}

// Initialize starting loop paramaters for calculating velocity and time

unsigned long prev = 0;
int old_ct1=0;
int old_ct2=0;
int old_ct3=0;
int old_ct4=0;
float ticks_per_meter = 33000.0;

void loop() {
  
  // count encoder ticks
  int ct1 = encoder_fleft.count();
  int ct2 = encoder_fright.count();
  int ct3 = encoder_bleft.count();
  int ct4 = encoder_bright.count();
  // for some reason if i omit this it does not work properly
  if (ct1!=-1){
    lfcount.data = ct1;}
  if (ct2!=-1){
    rfcount.data = ct2;}
  if (ct3!=-1){
    lbcount.data = ct3;}
  if (ct4!=-1){
    rbcount.data = ct4;}
  // Publish encoder ticks to calculate odom on Jetson Nano side
  lfwheel.publish(&lfcount);
  rfwheel.publish(&rfcount);
  lbwheel.publish(&lbcount);
  rbwheel.publish(&rbcount);

 // calculate time and current velocity
  
  unsigned long now = millis();
  //unsigned long Elapsed_Time = (now - prev) / 1000.0;
  Input_fl = ((float(ct1 - old_ct1))/ 33000.1) / ((now - prev) / 1000.0);
  Input_fr = ((float(ct2 - old_ct2))/ 33000.1) /((now - prev) / 1000.0);
  Input_bl = ((float(ct3 - old_ct3))/ 33000.1) /((now - prev) / 1000.0);
  Input_br = ((float(ct4 - old_ct4))/ 33000.1) /((now - prev) / 1000.0);
 
  // Use aggresive pid paramaters if gap > 0.1 else conservative
  bool gap1 = abs(Setpoint_fl - Input_fl) < 0.1;
  bool gap2 = abs(Setpoint_fr - Input_fr) < 0.1;
  bool gap3 = abs(Setpoint_bl - Input_bl) < 0.1;
  bool gap4 = abs(Setpoint_br - Input_br) < 0.1;
  
  if(gap1){
    myPID_fl.SetTunings(consKp, consKi, consKd);
  }
  else{
    myPID_fl.SetTunings(aggKp, aggKi, aggKd);
  }
  if(gap2){
    myPID_fr.SetTunings(consKp, consKi, consKd);
  }
  else{
    myPID_fr.SetTunings(aggKp, aggKi, aggKd);
  }
  if(gap3){
    myPID_bl.SetTunings(consKp, consKi, consKd);
  }
  else{
    myPID_bl.SetTunings(aggKp, aggKi, aggKd);
  }
  if(gap4){
    myPID_br.SetTunings(consKp, consKi, consKd);
  }
  else{
    myPID_br.SetTunings(aggKp, aggKi, aggKd);
  }
  // Compute  Pid
  myPID_fl.Compute();
  myPID_fr.Compute();
  myPID_bl.Compute();
  myPID_br.Compute();


  // HELP
  if(wtf){
    Output_fl=0;
    Output_fr=0;
    Output_bl=0;
    Output_br=0;
    }

  // Move the motors with the output of the pid
  
  Move_motor(Output_fl,LF_PWM,LF_FORW,LF_BACK);
  Move_motor(Output_fr,RF_PWM,RF_FORW,RF_BACK);
  Move_motor(Output_bl,LB_PWM,LB_FORW,LB_BACK);
  Move_motor(Output_br,RB_PWM,RB_FORW,RB_BACK);

  // spin the ros node
  
  nh.spinOnce();
  // take the old encoder ticks and time for calculating velocity
  old_ct1 = encoder_fleft.count();
  old_ct2 = encoder_fright.count();
  old_ct3 = encoder_bleft.count();
  old_ct4 = encoder_bright.count();
  prev = now;
  
  delay(40);

}
