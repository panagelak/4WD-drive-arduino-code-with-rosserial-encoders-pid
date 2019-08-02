#define USE_USBCON
#include <ros.h>
#include <std_msgs/Int32.h>
#include <quadrature.h>


Quadrature_encoder<46,47> encoder_fright(Board::due);
Quadrature_encoder<48,49> encoder_fleft(Board::due);
Quadrature_encoder<42,43> encoder_bright(Board::due);
Quadrature_encoder<44,45> encoder_bleft(Board::due);

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

std_msgs::Int32 lfcount;
std_msgs::Int32 rfcount;
std_msgs::Int32 lbcount;
std_msgs::Int32 rbcount;
ros::NodeHandle nh;

// Declare functions
void onMotorFLeft(const std_msgs::Int32& msg)
{
  //nh.loginfo("Inside Callback");
  if(msg.data > 0)
  {
    digitalWrite(LF_FORW, HIGH);
    digitalWrite(LF_BACK, LOW);
    analogWrite(LF_PWM, abs(msg.data));
  }
  else if(msg.data < 0)
  {
    digitalWrite(LF_FORW, LOW);
    digitalWrite(LF_BACK, HIGH);
    analogWrite(LF_PWM, abs(msg.data));
  }
  else if(msg.data == 0)
  {
    digitalWrite(LF_FORW, HIGH);
    digitalWrite(LF_BACK, HIGH);
  }

}
void onMotorFRight(const std_msgs::Int32& msg)
{
  //nh.loginfo("Inside Callback");
  if(msg.data > 0)
  {
    digitalWrite(RF_FORW, HIGH);
    digitalWrite(RF_BACK, LOW);
    analogWrite(RF_PWM, abs(msg.data));
  }
  else if(msg.data < 0)
  {
    digitalWrite(RF_FORW, LOW);
    digitalWrite(RF_BACK, HIGH);
    analogWrite(RF_PWM, abs(msg.data));
  }
  else if(msg.data == 0)
  {
    digitalWrite(RF_FORW, HIGH);
    digitalWrite(RF_BACK, HIGH);
  }

}

void onMotorBLeft(const std_msgs::Int32& msg)
{
  //nh.loginfo("Inside Callback");
  if(msg.data > 0)
  {
    digitalWrite(LB_FORW, HIGH);
    digitalWrite(LB_BACK, LOW);
    analogWrite(LB_PWM, abs(msg.data));
  }
  else if(msg.data < 0)
  {
    digitalWrite(LB_FORW, LOW);
    digitalWrite(LB_BACK, HIGH);
    analogWrite(LB_PWM, abs(msg.data));
  }
  else if(msg.data == 0)
  {
    digitalWrite(LB_FORW, HIGH);
    digitalWrite(LB_BACK, HIGH);
  }

}
void onMotorBRight(const std_msgs::Int32& msg)
{
  //nh.loginfo("Inside Callback");
  if(msg.data > 0)
  {
    digitalWrite(RB_FORW, HIGH);
    digitalWrite(RB_BACK, LOW);
    analogWrite(RB_PWM, abs(msg.data));
  }
  else if(msg.data < 0)
  {
    digitalWrite(RB_FORW, LOW);
    digitalWrite(RB_BACK, HIGH);
    analogWrite(RB_PWM, abs(msg.data));
  }
  else if(msg.data == 0)
  {
    digitalWrite(RB_FORW, HIGH);
    digitalWrite(RB_BACK, HIGH);
  }

}

ros::Publisher lfwheel("lfwheel", &lfcount);
ros::Publisher rfwheel("rfwheel", &rfcount);
ros::Publisher lbwheel("lbwheel", &lbcount);
ros::Publisher rbwheel("rbwheel", &rbcount);
ros::Subscriber<std_msgs::Int32> sub_fleft("/left_front_wheel_speed", onMotorFLeft);
ros::Subscriber<std_msgs::Int32> sub_fright("/right_front_wheel_speed", onMotorFRight);
ros::Subscriber<std_msgs::Int32> sub_bleft("/left_back_wheel_speed", onMotorBLeft);
ros::Subscriber<std_msgs::Int32> sub_bright("/right_back_wheel_speed", onMotorBRight);



void setup() {
  // put your setup code here, to run once:
  nh.getHardware()->setBaud(115200);
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
  encoder_fright.begin();
  encoder_fleft.begin();
  encoder_bright.begin();
  encoder_bleft.begin();
  

  digitalWrite(RF_FORW, HIGH);
  digitalWrite(RF_BACK, LOW);
  analogWrite(RF_PWM, 180);
  digitalWrite(LF_FORW, HIGH);
  digitalWrite(LF_BACK, LOW);
  analogWrite(LF_PWM, 180);
  digitalWrite(RB_FORW, HIGH);
  digitalWrite(RB_BACK, LOW);
  analogWrite(RB_PWM, 180);
  digitalWrite(LB_FORW, HIGH);
  digitalWrite(LB_BACK, LOW);
  analogWrite(LB_PWM, 180);
  delay(450);
  digitalWrite(RF_FORW, HIGH);
  digitalWrite(RF_BACK, LOW);
  analogWrite(RF_PWM, 0);
  digitalWrite(LF_FORW, HIGH);
  digitalWrite(LF_BACK, LOW);
  analogWrite(LF_PWM, 0);
  digitalWrite(RB_FORW, HIGH);
  digitalWrite(RB_BACK, LOW);
  analogWrite(RB_PWM, 0);
  digitalWrite(LB_FORW, HIGH);
  digitalWrite(LB_BACK, LOW);
  analogWrite(LB_PWM, 0);

  int ct1 = encoder_fleft.count();
  int ct2 = encoder_fright.count();
  int ct3 = encoder_bleft.count();
  int ct4 = encoder_bright.count();
  if(ct1 < 0) encoder_fleft.reverse();
  if(ct2 < 0) encoder_fright.reverse();
  if(ct3 < 0) encoder_bleft.reverse();
  if(ct4 < 0) encoder_bright.reverse();

  stop();
  nh.initNode();
  nh.advertise(lfwheel);
  nh.advertise(rfwheel);
  nh.advertise(lbwheel);
  nh.advertise(rbwheel);
  nh.subscribe(sub_fleft);
  nh.subscribe(sub_fright);
  nh.subscribe(sub_bleft);
  nh.subscribe(sub_bright);
}

void loop() {
  
  int ct1 = encoder_fleft.count();
  int ct2 = encoder_fright.count();
  int ct3 = encoder_bleft.count();
  int ct4 = encoder_bright.count();
  
  if (ct1!=-1){
    lfcount.data = ct1;}
  if (ct2!=-1){
    rfcount.data = ct2;}
  if (ct3!=-1){
    lbcount.data = ct3;}
  if (ct4!=-1){
    rbcount.data = ct4;}
  lfwheel.publish(&lfcount);
  rfwheel.publish(&rfcount);
  lbwheel.publish(&lbcount);
  rbwheel.publish(&rbcount);
  nh.spinOnce();
  delay(33);

}



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
