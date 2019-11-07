/* 
 * Button Example for Rosserial
 */

#include <ros.h>
#include <std_msgs/Bool.h>


ros::NodeHandle nh;

std_msgs::Bool pushed_msg;
ros::Publisher pub_button("odrive1_low_tib", &pushed_msg); //old topic name: odrive1_lims
ros::Publisher pub_button("odrive1_high_tib", &pushed_msg);
ros::Publisher pub_button("odrive1_low_fem", &pushed_msg);
ros::Publisher pub_button("odrive1_high_fem", &pushed_msg);

ros::Publisher pub_button("odrive2_low_tib", &pushed_msg);
ros::Publisher pub_button("odrive2_high_tib", &pushed_msg);
ros::Publisher pub_button("odrive2_low_fem", &pushed_msg);
ros::Publisher pub_button("odrive2_high_fem", &pushed_msg);

ros::Publisher pub_button("odrive3_low_tib", &pushed_msg); 
ros::Publisher pub_button("odrive3_high_tib", &pushed_msg);
ros::Publisher pub_button("odrive3_low_fem", &pushed_msg);
ros::Publisher pub_button("odrive3_high_fem", &pushed_msg);

ros::Publisher pub_button("odrive4_low_tib", &pushed_msg); 
ros::Publisher pub_button("odrive4_high_tib", &pushed_msg);
ros::Publisher pub_button("odrive4_low_fem", &pushed_msg);
ros::Publisher pub_button("odrive4_high_fem", &pushed_msg);

ros::Publisher pub_button("odrive5_low_tib", &pushed_msg); 
ros::Publisher pub_button("odrive5_high_tib", &pushed_msg);
ros::Publisher pub_button("odrive5_low_fem", &pushed_msg);
ros::Publisher pub_button("odrive5_high_fem", &pushed_msg);

ros::Publisher pub_button("odrive6_low_tib", &pushed_msg); 
ros::Publisher pub_button("odrive6_high_tib", &pushed_msg);
ros::Publisher pub_button("odrive6_low_fem", &pushed_msg);
ros::Publisher pub_button("odrive6_high_fem", &pushed_msg);

const int odrive1_low_tib = 1;
const int odrive1_high_tib = 2;
const int odrive1_low_feb = 3;
const int odrive1_high_feb = 4;

const int odrive2_low_tib = 5;
const int odrive2_high_tib = 6;
const int odrive2_low_fem = 7;
const int odrive2_high_fem = 8;

const int odrive3_low_tib = 9;
const int odrive3_high_tib = 10;
const int odrive3_low_fem = 11;
const int odrive3_high_fem = 12;

const int odrive4_low_tib = 13;
const int odrive4_high_tib = 13; //fix pins when we get our hands on a mega 
const int odrive4_low_fem = 13;
const int odrive4_high_fem = 13;

const int odrive5_low_tib = 13;
const int odrive5_high_tib = 13;
const int odrive5_low_fem = 13;
const int odrive5_high_fem = 13;

const int odrive6_low_tib = 13;
const int odrive6_high_tib = 13;
const int odrive6_low_fem = 13;
const int odrive6_high_fem = 13;

bool last_reading;
long last_debounce_time=0;
long debounce_delay=50;
bool published = true;

void setup()
{
  nh.initNode();
  nh.advertise(pub_button);
  
  //initialize an LED output pin 
  //and a input pin for our push button
  
  pinMode(odrive1_low_tib, INPUT_PULLUP);
  pinMode(odrive1_high_tib, INPUT_PULLUP);
  pinMode(odrive1_low_fem, INPUT_PULLUP);
  pinMode(odrive1_high_fem, INPUT_PULLUP);
  
  pinMode(odrive2_low_tib, INPUT_PULLUP);
  pinMode(odrive2_high_tib, INPUT_PULLUP);
  pinMode(odrive2_low_fem, INPUT_PULLUP);
  pinMode(odrive2_high_fem, INPUT_PULLUP);

  pinMode(odrive3_low_tib, INPUT_PULLUP);
  pinMode(odrive3_high_tib, INPUT_PULLUP);
  pinMode(odrive3_low_fem, INPUT_PULLUP);
  pinMode(odrive3_high_fem, INPUT_PULLUP);

  pinMode(odrive4_low_tib, INPUT_PULLUP);
  pinMode(odrive4_high_tib, INPUT_PULLUP);
  pinMode(odrive4_low_fem, INPUT_PULLUP);
  pinMode(odrive4_high_fem, INPUT_PULLUP);

  pinMode(odrive5_low_tib, INPUT_PULLUP);
  pinMode(odrive5_high_tib, INPUT_PULLUP);
  pinMode(odrive5_low_fem, INPUT_PULLUP);
  pinMode(odrive5_high_fem, INPUT_PULLUP);

  pinMode(odrive6_low_tib, INPUT_PULLUP);
  pinMode(odrive6_high_tib, INPUT_PULLUP);
  pinMode(odrive6_low_fem, INPUT_PULLUP);
  pinMode(odrive6_high_fem, INPUT_PULLUP);



  
  
  //Enable the pullup resistor on the button
  digitalWrite(odrive1_low_tib, HIGH);
  digitalWrite(odrive1_high_tib, HIGH);
  digitalWrite(odrive1_low_fem, HIGH);
  digitalWrite(odrive1_high_fem, HIGH);

  digitalWrite(odrive2_low_tib, HIGH);
  digitalWrite(odrive2_high_tib, HIGH);
  digitalWrite(odrive2_low_fem, HIGH);
  digitalWrite(odrive2_high_fem, HIGH);
  
  digitalWrite(odrive3_low_tib, HIGH);
  digitalWrite(odrive3_high_tib, HIGH);
  digitalWrite(odrive3_low_fem, HIGH);
  digitalWrite(odrive3_high_fem, HIGH);
  
  digitalWrite(odrive4_low_tib, HIGH);
  digitalWrite(odrive4_high_tib, HIGH);
  digitalWrite(odrive4_low_fem, HIGH);
  digitalWrite(odrive4_high_fem, HIGH);

  digitalWrite(odrive5_low_tib, HIGH);
  digitalWrite(odrive5_high_tib, HIGH); //5 and 6 r for hip
  digitalWrite(odrive5_low_fem, HIGH);
  digitalWrite(odrive5_high_fem, HIGH);

  digitalWrite(odrive6_low_tib, HIGH);
  digitalWrite(odrive6_high_tib, HIGH);
  digitalWrite(odrive6_low_fem, HIGH);
  digitalWrite(odrive6_high_fem, HIGH);
  
  //The button is a normally button
  last_reading1_low_tib = ! digitalRead(odrive1_low_tib);
  last_reading1_high_tib = ! digitalRead(odrive1_high_tib);
  last_reading1_low_fem = ! digitalRead(odrive1_low_fem);
  last_reading1_high_fem = ! digitalRead(odrive1_hgih_fem);

  last_reading2_low_tib = ! digitalRead(odrive2_low_tib);
  last_reading2_high_tib = ! digitalRead(odrive2_high_tib);
  last_reading2_low_fem = ! digitalRead(odrive2_low_fem);
  last_reading2_high_fem = ! digitalRead(odrive2_hgih_fem);

  last_reading3_low_tib = ! digitalRead(odrive3_low_tib);
  last_reading3_high_tib = ! digitalRead(odrive3_high_tib);
  last_reading3_low_fem = ! digitalRead(odrive3_low_fem);
  last_reading3_high_fem = ! digitalRead(odrive3_hgih_fem);

  last_reading4_low_tib = ! digitalRead(odrive4_low_tib);
  last_reading4_high_tib = ! digitalRead(odrive4_high_tib);
  last_reading4_low_fem = ! digitalRead(odrive4_low_fem);
  last_reading4_high_fem = ! digitalRead(odrive4_hgih_fem);

  

 
}

void loop()
{
  
  bool reading = ! digitalRead(button_pin);
  
  if (last_reading!= reading){
      last_debounce_time = millis();
      published = false;
  }
  
  //if the button value has not changed for the debounce delay, we know its stable
  if ( !published && (millis() - last_debounce_time)  > debounce_delay) {
    digitalWrite(led_pin, reading);
    pushed_msg.data = reading;
    pub_button.publish(&pushed_msg);
    published = true;
  }

  last_reading = reading;
  
  nh.spinOnce();
}
