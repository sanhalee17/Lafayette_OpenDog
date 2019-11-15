/* 
 * Button Example for Rosserial
 */

#include <ros.h>
#include <std_msgs/Bool.h>


ros::NodeHandle nh;

std_msgs::Bool pushed_msg;
ros::Publisher pub_button1("odrive1_low_tib", &pushed_msg); //old topic name: odrive1_lims
ros::Publisher pub_button2("odrive1_high_tib", &pushed_msg);
ros::Publisher pub_button3("odrive1_low_fem", &pushed_msg);
ros::Publisher pub_button4("odrive1_high_fem", &pushed_msg);

ros::Publisher pub_button5("odrive2_low_tib", &pushed_msg);
ros::Publisher pub_button6("odrive2_high_tib", &pushed_msg);
ros::Publisher pub_button7("odrive2_low_fem", &pushed_msg);
ros::Publisher pub_button8("odrive2_high_fem", &pushed_msg);

ros::Publisher pub_button9("odrive3_low_tib", &pushed_msg); 
ros::Publisher pub_button10("odrive3_high_tib", &pushed_msg);
ros::Publisher pub_button11("odrive3_low_fem", &pushed_msg);
ros::Publisher pub_button12("odrive3_high_fem", &pushed_msg);

ros::Publisher pub_button13("odrive4_low_tib", &pushed_msg); 
ros::Publisher pub_button14("odrive4_high_tib", &pushed_msg);
ros::Publisher pub_button15("odrive4_low_fem", &pushed_msg);
ros::Publisher pub_button16("odrive4_high_fem", &pushed_msg);

ros::Publisher pub_button17("odrive5_low_tib", &pushed_msg); 
ros::Publisher pub_button18("odrive5_high_tib", &pushed_msg);
ros::Publisher pub_button19("odrive5_low_fem", &pushed_msg);
ros::Publisher pub_button20("odrive5_high_fem", &pushed_msg);

ros::Publisher pub_button21("odrive6_low_tib", &pushed_msg); 
ros::Publisher pub_button22("odrive6_high_tib", &pushed_msg);
ros::Publisher pub_button23("odrive6_low_fem", &pushed_msg);
ros::Publisher pub_button24("odrive6_high_fem", &pushed_msg);

const int odrive1_low_tib = 7;
const int odrive1_high_tib = 2;
const int odrive1_low_fem = 3;
const int odrive1_high_fem = 4;

const int odrive2_low_tib = 5;
const int odrive2_high_tib = 6;
const int odrive2_low_fem = 25;
const int odrive2_high_fem = 8;

const int odrive3_low_tib = 9;
const int odrive3_high_tib = 10;
const int odrive3_low_fem = 11;
const int odrive3_high_fem = 12;

const int odrive4_low_tib = 13;
const int odrive4_high_tib = 14; //fix pins when we get our hands on a mega 
const int odrive4_low_fem = 37; // not working change pin? -> changed to pin 37 Nov. 13.2019 by SL
const int odrive4_high_fem = 27;

const int odrive5_low_tib = 28;
const int odrive5_high_tib = 29;
const int odrive5_low_fem = 30;
const int odrive5_high_fem = 31;

const int odrive6_low_tib = 32;
const int odrive6_high_tib = 33;
const int odrive6_low_fem = 34;
const int odrive6_high_fem = 35;

bool last_reading1_low_tib = false;
bool last_reading1_high_tib = false;
bool last_reading1_low_fem = false;
bool last_reading1_high_fem = false;

bool last_reading2_low_tib = false;
bool last_reading2_high_tib = false;
bool last_reading2_low_fem = false;
bool last_reading2_high_fem = false;

bool last_reading3_low_tib = false;
bool last_reading3_high_tib = false;
bool last_reading3_low_fem = false;
bool last_reading3_high_fem = false;

bool last_reading4_low_tib = false;
bool last_reading4_high_tib = false;
bool last_reading4_low_fem = false;
bool last_reading4_high_fem = false;

bool last_reading5_low_tib = false;
bool last_reading5_high_tib = false;
bool last_reading5_low_fem = false;
bool last_reading5_high_fem = false;

bool last_reading6_low_tib = false;
bool last_reading6_high_tib = false;
bool last_reading6_low_fem = false;
bool last_reading6_high_fem = false;

bool  reading1_low_tib = false;
bool  reading1_high_tib = false;
bool  reading1_low_fem = false;
bool  reading1_high_fem = false;

bool  reading2_low_tib = false;
bool  reading2_high_tib = false;
bool  reading2_low_fem = false;
bool  reading2_high_fem = false;

bool  reading3_low_tib = false;
bool  reading3_high_tib = false;
bool  reading3_low_fem = false;
bool  reading3_high_fem = false;

bool  reading4_low_tib = false;
bool  reading4_high_tib = false;
bool  reading4_low_fem = false;
bool  reading4_high_fem = false;

bool  reading5_low_tib = false;
bool  reading5_high_tib = false;
bool  reading5_low_fem = false;
bool  reading5_high_fem = false;

bool  reading6_low_tib = false;
bool  reading6_high_tib = false;
bool  reading6_low_fem = false;
bool  reading6_high_fem = false;

long last_debounce_time=0;
long debounce_delay = 50;


//declare publishers here

//odrive 1
bool published = true;
bool published2 = true;
bool published3 = true;
bool published4 = true;

//odrive 2
bool published5 = true;
bool published6 = true;
bool published7 = true;
bool published8 = true;

//odrive 3
bool published9 = true;
bool published10 = true;
bool published11 = true;
bool published12 = true;

//odrive 4
bool published13 = true;
bool published14 = true;
bool published15 = true;
bool published16 = true;

//odrive 5
bool published17 = true;
bool published18 = true;
bool published19 = true;
bool published20 = true;

//odrive 6
bool published21 = true;
bool published22 = true;
bool published23 = true;
bool published24 = true;

void setup()
{
  nh.initNode();

  //odrive 1
  nh.advertise(pub_button1);
  nh.advertise(pub_button2);
  nh.advertise(pub_button3);
  nh.advertise(pub_button4);
  
  //odrive 2
  nh.advertise(pub_button5);
  nh.advertise(pub_button6);
  nh.advertise(pub_button7);
  nh.advertise(pub_button8);

  //odrive 3
  nh.advertise(pub_button9);
  nh.advertise(pub_button10);
  nh.advertise(pub_button11);
  nh.advertise(pub_button12);

  //odrive 4
  nh.advertise(pub_button13);
  nh.advertise(pub_button14);
  nh.advertise(pub_button15);
  nh.advertise(pub_button16);

  //odrive 5
  nh.advertise(pub_button17);
  nh.advertise(pub_button18);
  nh.advertise(pub_button19);
  nh.advertise(pub_button20);

  //odrive 6
  nh.advertise(pub_button21);
  nh.advertise(pub_button22);
  nh.advertise(pub_button23);
  nh.advertise(pub_button24);
  
  
  pinMode(odrive1_low_tib, INPUT);
  pinMode(odrive1_high_tib, INPUT);
  pinMode(odrive1_low_fem, INPUT);
  pinMode(odrive1_high_fem, INPUT);
  
  pinMode(odrive2_low_tib, INPUT);
  pinMode(odrive2_high_tib, INPUT);
  pinMode(odrive2_low_fem, INPUT);
  pinMode(odrive2_high_fem, INPUT);

  pinMode(odrive3_low_tib, INPUT);
  pinMode(odrive3_high_tib, INPUT);
  pinMode(odrive3_low_fem, INPUT);
  pinMode(odrive3_high_fem, INPUT);

  pinMode(odrive4_low_tib, INPUT);
  pinMode(odrive4_high_tib, INPUT);
  pinMode(odrive4_low_fem, INPUT);
  pinMode(odrive4_high_fem, INPUT);

  pinMode(odrive5_low_tib, INPUT);
  pinMode(odrive5_high_tib, INPUT);
  pinMode(odrive5_low_fem, INPUT);
  pinMode(odrive5_high_fem, INPUT);

  pinMode(odrive6_low_tib, INPUT);
  pinMode(odrive6_high_tib, INPUT);
  pinMode(odrive6_low_fem, INPUT);
  pinMode(odrive6_high_fem, INPUT);
  
//  
//  //Enable the pullup resistor on the button
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
  last_reading1_high_fem = ! digitalRead(odrive1_high_fem);

  last_reading2_low_tib = ! digitalRead(odrive2_low_tib);
  last_reading2_high_tib = ! digitalRead(odrive2_high_tib);
  last_reading2_low_fem = ! digitalRead(odrive2_low_fem);
  last_reading2_high_fem = ! digitalRead(odrive2_high_fem);

  last_reading3_low_tib = ! digitalRead(odrive3_low_tib);
  last_reading3_high_tib = ! digitalRead(odrive3_high_tib);
  last_reading3_low_fem = ! digitalRead(odrive3_low_fem);
  last_reading3_high_fem = ! digitalRead(odrive3_high_fem);

  last_reading4_low_tib = ! digitalRead(odrive4_low_tib);
  last_reading4_high_tib = ! digitalRead(odrive4_high_tib);
  last_reading4_low_fem = ! digitalRead(odrive4_low_fem);
  last_reading4_high_fem = ! digitalRead(odrive4_high_fem);

  last_reading5_low_tib = ! digitalRead(odrive5_low_tib);
  last_reading5_high_tib = ! digitalRead(odrive5_high_tib);
  last_reading5_low_fem = ! digitalRead(odrive5_low_fem);
  last_reading5_high_fem = ! digitalRead(odrive5_high_fem);

  last_reading6_low_tib = ! digitalRead(odrive6_low_tib);
  last_reading6_high_tib = ! digitalRead(odrive6_high_tib);
  last_reading6_low_fem = ! digitalRead(odrive6_low_fem);
  last_reading6_high_fem = ! digitalRead(odrive6_high_fem);

  

 
}

void loop()
{


  bool reading1_low_tib = ! digitalRead(odrive1_low_tib);
  reading1_high_tib = !digitalRead(odrive1_high_tib);
  reading1_low_fem = !digitalRead(odrive1_low_fem);
  reading1_high_fem = !digitalRead(odrive1_high_fem);

  reading2_low_tib = ! digitalRead(odrive2_low_tib);
  reading2_high_tib = !digitalRead(odrive2_high_tib);
  reading2_low_fem = !digitalRead(odrive2_low_fem);
  reading2_high_fem = !digitalRead(odrive2_high_fem);
  
  reading3_low_tib = ! digitalRead(odrive3_low_tib);
  reading3_high_tib = !digitalRead(odrive3_high_tib);
  reading3_low_fem = !digitalRead(odrive3_low_fem);
  reading3_high_fem = !digitalRead(odrive3_high_fem);
  
  reading4_low_tib = ! digitalRead(odrive4_low_tib);
  reading4_high_tib = !digitalRead(odrive4_high_tib);
  reading4_low_fem = !digitalRead(odrive4_low_fem);
  reading4_high_fem = !digitalRead(odrive4_high_fem);
  
  reading5_low_tib = ! digitalRead(odrive5_low_tib);
  reading5_high_tib = !digitalRead(odrive5_high_tib);
  reading5_low_fem = !digitalRead(odrive5_low_fem);
  reading5_high_fem = !digitalRead(odrive5_high_fem);
  
  reading6_low_tib = ! digitalRead(odrive6_low_tib);
  reading6_high_tib = !digitalRead(odrive6_high_tib);
  reading6_low_fem = !digitalRead(odrive6_low_fem);
  reading6_high_fem = !digitalRead(odrive6_high_fem);



  if (last_reading1_low_tib!= reading1_low_tib){
      last_debounce_time = millis();
      published = false;
  }


  if ( !published && (millis() - last_debounce_time)  > debounce_delay) {
    digitalWrite(odrive1_low_tib, reading1_low_tib);
    pushed_msg.data = !digitalRead(odrive1_low_tib);
    pub_button2.publish(&pushed_msg);
    published = true;
  }

//odrive 1 high tibia
  if (last_reading1_high_tib!= reading1_high_tib){
      last_debounce_time = millis();
      published2 = false;
  }


    if ( !published2 && (millis() - last_debounce_time)  > debounce_delay) {
    digitalWrite(odrive1_high_tib, reading1_high_tib);
    pushed_msg.data = !digitalRead(odrive1_high_tib);
    pub_button2.publish(&pushed_msg);
    published = true;
  }


//odrive 1 low femur
  if (last_reading1_low_fem!= reading1_low_fem){
      last_debounce_time = millis();
      published3 = false;
  }


    if ( !published3 && (millis() - last_debounce_time)  > debounce_delay) {
    digitalWrite(odrive1_low_fem, reading1_low_fem);
    pushed_msg.data = !digitalRead(odrive1_low_fem);
    pub_button3.publish(&pushed_msg);
    published3 = true;
  }

//odrive 1 high femur
  if (last_reading1_high_fem!= reading1_high_fem){
      last_debounce_time = millis();
      published4 = false;
  }


    if ( !published4 && (millis() - last_debounce_time)  > debounce_delay) {
    digitalWrite(odrive1_high_fem, reading1_high_fem);
    pushed_msg.data = !digitalRead(odrive1_high_fem);
    pub_button4.publish(&pushed_msg);
    published4 = true;
  }


//odrive 2

//odrive 2 low tibia
  if (last_reading2_low_tib!= reading2_low_tib){
      last_debounce_time = millis();
      published5 = false;
  }


  if ( !published5 && (millis() - last_debounce_time)  > debounce_delay) {
    digitalWrite(odrive2_low_tib, reading2_low_tib);
    pushed_msg.data = !digitalRead(odrive2_low_tib);
    pub_button5.publish(&pushed_msg);
    published5 = true;
  }

//odrive 2 high tibia
  if (last_reading2_high_tib!= reading2_high_tib){
      last_debounce_time = millis();
      published6 = false;
  }


    if ( !published6 && (millis() - last_debounce_time)  > debounce_delay) {
    digitalWrite(odrive2_high_tib, reading2_high_tib);
    pushed_msg.data = !digitalRead(odrive2_high_tib);
    pub_button6.publish(&pushed_msg);
    published = true;
  }


//odrive 2 low femur
  if (last_reading2_low_fem!= reading2_low_fem){
      last_debounce_time = millis();
      published7 = false;
  }


    if ( !published7 && (millis() - last_debounce_time)  > debounce_delay) {
    digitalWrite(odrive2_low_fem, reading2_low_fem);
    pushed_msg.data = !digitalRead(odrive2_low_fem);
    pub_button7.publish(&pushed_msg);
    published7 = true;
  }

//odrive 2 high femur
  if (last_reading2_high_fem!= reading2_high_fem){
      last_debounce_time = millis();
      published8 = false;
  }


    if ( !published8 && (millis() - last_debounce_time)  > debounce_delay) {
    digitalWrite(odrive2_high_fem, reading2_high_fem);
    pushed_msg.data = !digitalRead(odrive2_high_fem);
    pub_button8.publish(&pushed_msg);
    published8 = true;
  }

  //Odrive 3

  //odrive 3 low tibia
    if (last_reading3_low_tib!= reading3_low_tib){
      last_debounce_time = millis();
      published9 = false;
  }


  if ( !published9 && (millis() - last_debounce_time)  > debounce_delay) {
    digitalWrite(odrive3_low_tib, reading3_low_tib);
    pushed_msg.data = !digitalRead(odrive3_low_tib);
    pub_button9.publish(&pushed_msg);
    published9 = true;
  }

//odrive 3 high tibia
  if (last_reading3_high_tib!= reading3_high_tib){
      last_debounce_time = millis();
      published10 = false;
  }


    if ( !published10 && (millis() - last_debounce_time)  > debounce_delay) {
    digitalWrite(odrive3_high_tib, reading3_high_tib);
    pushed_msg.data = !digitalRead(odrive3_high_tib);
    pub_button10.publish(&pushed_msg);
    published10 = true;
  }


//odrive 3 low femur
  if (last_reading3_low_fem!= reading3_low_fem){
      last_debounce_time = millis();
      published11 = false;
  }


    if ( !published11 && (millis() - last_debounce_time)  > debounce_delay) {
    digitalWrite(odrive3_low_fem, reading3_low_fem);
    pushed_msg.data = !digitalRead(odrive3_low_fem);
    pub_button11.publish(&pushed_msg);
    published11 = true;
  }

//odrive 3 high femur
  if (last_reading3_high_fem!= reading3_high_fem){
      last_debounce_time = millis();
      published12 = false;
  }


    if ( !published12 && (millis() - last_debounce_time)  > debounce_delay) {
    digitalWrite(odrive3_high_fem, reading3_high_fem);
    pushed_msg.data = !digitalRead(odrive3_high_fem);
    pub_button12.publish(&pushed_msg);
    published12 = true;
  }

    //Odrive 4

  //odrive 4 low tibia
    if (last_reading4_low_tib!= reading4_low_tib){
      last_debounce_time = millis();
      published13 = false;
  }


  if ( !published13 && (millis() - last_debounce_time)  > debounce_delay) {
    digitalWrite(odrive4_low_tib, reading4_low_tib);
    pushed_msg.data = !digitalRead(odrive4_low_tib);
    pub_button13.publish(&pushed_msg);
    published13 = true;
  }

//odrive 4 high tibia
  if (last_reading4_high_tib!= reading4_high_tib){
      last_debounce_time = millis();
      published14 = false;
  }


    if ( !published14 && (millis() - last_debounce_time)  > debounce_delay) {
    digitalWrite(odrive4_high_tib, reading4_high_tib);
    pushed_msg.data = !digitalRead(odrive4_high_tib);
    pub_button14.publish(&pushed_msg);
    published14 = true;
  }


//odrive 4 low femur
  if (last_reading4_low_fem!= reading4_low_fem){
      last_debounce_time = millis();
      published15 = false;
  }


    if ( !published4 && (millis() - last_debounce_time)  > debounce_delay) {
    digitalWrite(odrive4_low_fem, reading4_low_fem);
    pushed_msg.data = !digitalRead(odrive4_low_fem);
    pub_button15.publish(&pushed_msg);
    published15 = true;
  }

//odrive 4 high femur
  if (last_reading4_high_fem!= reading4_high_fem){
      last_debounce_time = millis();
      published16 = false;
  }


    if ( !published16 && (millis() - last_debounce_time)  > debounce_delay) {
    digitalWrite(odrive4_high_fem, reading4_high_fem);
    pushed_msg.data = !digitalRead(odrive4_high_fem);
    pub_button16.publish(&pushed_msg);
    published16 = true;
  }

    //Odrive 5

  //odrive 5 low tibia
    if (last_reading5_low_tib!= reading5_low_tib){
      last_debounce_time = millis();
      published17 = false;
  }


  if ( !published17 && (millis() - last_debounce_time)  > debounce_delay) {
    digitalWrite(odrive5_low_tib, reading5_low_tib);
    pushed_msg.data = !digitalRead(odrive5_low_tib);
    pub_button17.publish(&pushed_msg);
    published17 = true;
  }

//odrive 5 high tibia
  if (last_reading5_high_tib!= reading5_high_tib){
      last_debounce_time = millis();
      published18 = false;
  }


    if ( !published18 && (millis() - last_debounce_time)  > debounce_delay) {
    digitalWrite(odrive5_high_tib, reading5_high_tib);
    pushed_msg.data = !digitalRead(odrive5_high_tib);
    pub_button18.publish(&pushed_msg);
    published18 = true;
  }


//odrive 5 low femur
  if (last_reading5_low_fem!= reading5_low_fem){
      last_debounce_time = millis();
      published19 = false;
  }


    if ( !published19 && (millis() - last_debounce_time)  > debounce_delay) {
    digitalWrite(odrive5_low_fem, reading5_low_fem);
    pushed_msg.data = !digitalRead(odrive5_low_fem);
    pub_button19.publish(&pushed_msg);
    published19 = true;
  }

//odrive 5 high femur
  if (last_reading5_high_fem!= reading5_high_fem){
      last_debounce_time = millis();
      published20 = false;
  }


    if ( !published20 && (millis() - last_debounce_time)  > debounce_delay) {
    digitalWrite(odrive5_high_fem, reading5_high_fem);
    pushed_msg.data = !digitalRead(odrive5_high_fem);
    pub_button20.publish(&pushed_msg);
    published20 = true;
  }

   //Odrive 6

  //odrive 6 low tibia
    if (last_reading6_low_tib!= reading6_low_tib){
      last_debounce_time = millis();
      published21 = false;
  }


  if ( !published21 && (millis() - last_debounce_time)  > debounce_delay) {
    digitalWrite(odrive6_low_tib, reading6_low_tib);
    pushed_msg.data = !digitalRead(odrive6_low_tib);
    pub_button21.publish(&pushed_msg);
    published21 = true;
  }

//odrive 6 high tibia
  if (last_reading6_high_tib!= reading6_high_tib){
      last_debounce_time = millis();
      published22 = false;
  }


    if ( !published22 && (millis() - last_debounce_time)  > debounce_delay) {
    digitalWrite(odrive6_high_tib, reading6_high_tib);
    pushed_msg.data = !digitalRead(odrive6_high_tib);
    pub_button22.publish(&pushed_msg);
    published22 = true;
  }


//odrive 6 low femur
  if (last_reading6_low_fem!= reading6_low_fem){
      last_debounce_time = millis();
      published23 = false;
  }


    if ( !published23 && (millis() - last_debounce_time)  > debounce_delay) {
    digitalWrite(odrive6_low_fem, reading6_low_fem);
    pushed_msg.data = !digitalRead(odrive6_low_fem);
    pub_button23.publish(&pushed_msg);
    published23 = true;
  }

//odrive 6 high femur
  if (last_reading6_high_fem!= reading6_high_fem){
      last_debounce_time = millis();
      published24 = false;
  }


    if ( !published24 && (millis() - last_debounce_time)  > debounce_delay) {
    digitalWrite(odrive6_high_fem, reading6_high_fem);
    pushed_msg.data = !digitalRead(odrive6_high_fem);
    pub_button24.publish(&pushed_msg);
    published24 = true;
  }
  

//pushed_msg.data = false;//!digitalRead(odrive1_low_tib);
// pub_button1.publish(&pushed_msg);
  last_reading1_low_tib = reading1_low_tib;
  last_reading1_high_tib = reading1_high_tib;
  last_reading1_low_fem = reading1_low_fem;
  last_reading1_high_fem = reading1_high_fem;

  last_reading2_low_tib = reading2_low_tib;
  last_reading2_high_tib = reading2_high_tib;
  last_reading2_low_fem = reading2_low_fem;
  last_reading2_high_fem = reading2_high_fem;

  last_reading3_low_tib = reading3_low_tib;
  last_reading3_high_tib = reading3_high_tib;
  last_reading3_low_fem = reading3_low_fem;
  last_reading3_high_fem = reading3_high_fem;

  last_reading4_low_tib = reading4_low_tib;
  last_reading4_high_tib = reading4_high_tib;
  last_reading4_low_fem = reading4_low_fem;
  last_reading4_high_fem = reading4_high_fem;

  last_reading5_low_tib = reading5_low_tib;
  last_reading5_high_tib = reading5_high_tib;
  last_reading5_low_fem = reading5_low_fem;
  last_reading5_high_fem = reading5_high_fem;

  last_reading6_low_tib = reading6_low_tib;
  last_reading6_high_tib = reading6_high_tib;
  last_reading6_low_fem = reading6_low_fem;
  last_reading6_high_fem = reading6_high_fem;
//  
  nh.spinOnce();
}
