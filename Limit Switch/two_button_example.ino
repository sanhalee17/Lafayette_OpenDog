/* 
 * Button Example for Rosserial
 */

#include <ros.h>
#include <std_msgs/ByteMultiArray.h>


ros::NodeHandle nh;

std_msgs::ByteMultiArray odrive1_msg;
std_msgs::MultiArrayLayout layout;
std_msgs::MultiArrayDimension dim;
int offset = 0;
ros::Publisher pub_button("odrive1_lims", &odrive1_msg,1);

const int odrive1_low_pin = 3;
const int odrive1_high_pin = 4;
const int led_pin = 13;

bool last_readinglow;
bool last_readinghigh;

long last_debounce_time=0;
long debounce_delay=50;
bool published = true;

byte odrive1data[] = {0,0};


void setup()
{
  Serial.begin(9600);
  nh.initNode();
  nh.advertise(pub_button);
  
  //initialize an LED output pin 
  //and a input pin for our push button
  //pinMode(led_pin, OUTPUT);
  pinMode(odrive1_low_pin, INPUT_PULLUP);
  pinMode(odrive1_high_pin, INPUT_PULLUP);
  

  
  //The button is a normally button
  last_readinglow = ! digitalRead(odrive1_low_pin);
  last_readinghigh = ! digitalRead(odrive1_high_pin);

  //set up the layout for the message
//layout.data_offset = offset;
//dim.size=2;
//  dim.stride=2;
//  layout.dim = dim;
 
}

void loop()
{
  
  bool readinglow = ! digitalRead(odrive1_low_pin);
  bool readinghigh = ! digitalRead(odrive1_high_pin);
  Serial.print(readinglow);
  Serial.println();
  
  if (last_readinglow!= readinglow){
      last_debounce_time = millis();
      published = false;
      
  }
  
  //if the button value has not changed for the debounce delay, we know its stable
  if ( !published && (millis() - last_debounce_time)  > debounce_delay) {
    digitalWrite(led_pin, readinglow);

    odrive1_msg.layout.data_offset = 0;
    odrive1_msg.layout.dim->size=2;
    odrive1_msg.layout.dim->stride=2;
    odrive1data[0] = readinglow;
    odrive1data[1] = readinghigh;
    odrive1_msg.data = odrive1data;
    pub_button.publish(&odrive1_msg);
    published = true;
  }

  last_readinglow = readinglow;
  last_readinghigh = readinghigh;
  
  nh.spinOnce();
}
