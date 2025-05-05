//#include <Arduino.h>
#include <AccelStepper.h>

void all_stepper_homing();
void puri_barrel_rotor();
void puri_push();
void puri_pull();
void s_catcher_open1();
void s_catcher_close1();
void blower();
void conveyor();
void drill();
void masala();
void s_catcher_open2();
void s_catcher_close2();
void plate_cup_fwd();
void plate_cup_bwd();
void cup_disp();
void plt_disp_on();
void plt_disp_off();

void customer_fwd();

void customer_fwd1();
void customer_fwd2();


void customer_bwd();
void puri_rotation();



const int puri_catcher_step = 47; // catcher PUL+
const int puri_catcher_dir = 46;  // catcher DIR+

const int barrel_step = 49; // barrel PUL+
const int barrel_dir = 48;  // barrel DIR+

const int conveyor_home_sensor = A2;
const int conveyor_step = 41;
const int conveyor_dir = 40;

const int puri_catcher_home_sensor = A0;
const int ir = A1; // ir to detect presence of puri in the puri catcher

const int s_catcher_opener1_step = 45; // step of 1st silicon catcher i.e below puri catcher
const int s_catcher_opener1_dir = 44;
const int s_catcher_opener1_home_sensor = A3; // 1st silicon catcher's home sensor

const int s_catcher_opener2_step = 39;
const int s_catcher_opener2_dir = 38;
const int s_catcher_opener2_home_sensor = A5;

const int blower_pwm = 8;
const int blower_taco = 9;
const int blower_solenoid = 22;

const int drill_step = 43;
const int drill_dir = 42;
const int drill_home_sensor = A4;
const int drill_motor = 12;

const int alu = 11;
const int onion = 10;
const int channa = 7;
const int sev = 6;

const int pc_home_sensor = A6;  //LIZ
const int plate_step= 37;   //NBR step
const int plate_dir= 36;    //NBR dir

const int plate_disp_hmp = A7;  //LID
const int pc_disp_step =35;     //NPZ step//incomplete
const int pc_disp_dir= 34;      //NPZ dir
 
const int plt_mtr= 5;   //FIG plate vaccum motor
const int plt_ir = A8;  //LIY plate detection ir
const int cup_mtr= 4;   //FIC cup dispensor motor
const int cup_ir = A11; //LEX cup detection ir

const int plate_rot_step= 33; //NEX pul+
const int plate_rot_dir= 32;  //NEX dir+

AccelStepper puriCatcher(AccelStepper::DRIVER, puri_catcher_step, puri_catcher_dir);
AccelStepper puriBarrel(AccelStepper::DRIVER, barrel_step, barrel_dir);
AccelStepper catcherOpener1(AccelStepper::DRIVER, s_catcher_opener1_step, s_catcher_opener1_dir);
AccelStepper catcherOpener2(AccelStepper::DRIVER, s_catcher_opener2_step, s_catcher_opener2_dir);
AccelStepper puriConveyor(AccelStepper::DRIVER, conveyor_step, conveyor_dir);
AccelStepper puriDrill(AccelStepper::DRIVER, drill_step, drill_dir);
AccelStepper cup_plate(AccelStepper::DRIVER, plate_step, plate_dir);
AccelStepper puri_delivery(AccelStepper::DRIVER, pc_disp_step, pc_disp_dir);
AccelStepper plate_rotate(AccelStepper::DRIVER, plate_rot_step, plate_rot_dir);


void setup()
{
  pinMode(puri_catcher_home_sensor, INPUT); // Set sensor pin as input with pull-up resistor
  pinMode(ir, INPUT);
  pinMode(s_catcher_opener1_home_sensor, INPUT);
  pinMode(s_catcher_opener2_home_sensor, INPUT);

  pinMode(blower_pwm, OUTPUT);
  pinMode(blower_taco, INPUT);
  pinMode(blower_solenoid, OUTPUT);

  pinMode(conveyor_home_sensor, INPUT);

  pinMode(drill_home_sensor, INPUT);
  pinMode(drill_motor, OUTPUT);

  pinMode(alu, OUTPUT);
  pinMode(onion, OUTPUT);
  pinMode(channa, OUTPUT);
  pinMode(sev, OUTPUT);

  pinMode(pc_home_sensor, INPUT);

  pinMode(plt_ir, INPUT);
  pinMode(cup_ir, INPUT);
  pinMode(plt_mtr, OUTPUT);
  pinMode(cup_mtr, OUTPUT);

  pinMode(plate_disp_hmp, INPUT);


  puriCatcher.setMaxSpeed(5000);     // Set max speed  puri catcher
  puriCatcher.setAcceleration(4000); // Set acceleration

  puriBarrel.setMaxSpeed(2000); // barrel
  puriBarrel.setAcceleration(5000);

  catcherOpener1.setMaxSpeed(5000); // s_catcher_opener1
  catcherOpener1.setAcceleration(3000);

  catcherOpener2.setMaxSpeed(5000); // s_catcher_opener2
  catcherOpener2.setAcceleration(3000);

  puriConveyor.setMaxSpeed(5000);
  puriConveyor.setAcceleration(500);

  puriDrill.setMaxSpeed(5000);
  puriDrill.setAcceleration(3000);

  cup_plate.setMaxSpeed(9000);
  cup_plate.setAcceleration(5000);

  puri_delivery.setMaxSpeed(9000);
  puri_delivery.setAcceleration(5000);

  plate_rotate.setMaxSpeed(5000);
  plate_rotate.setAcceleration(3000);

}

void loop()
{
  
  all_stepper_homing(); /*
  delay(200);
  puri_barrel_rotor();
  delay(200);
  s_catcher_open1();
  delay(200);
  puri_push();
  blower();
  delay(200);
  s_catcher_close1();
  delay(200);
  puri_pull();
  delay(200);
  conveyor();
  delay(200);
  drill();
  delay(200);
  conveyor();
  delay(200);
  masala();
  delay(200);
  s_catcher_open2();
  delay(200);
  s_catcher_close2();
  delay(200);    */
  
  plate_cup_fwd();
  //plt_disp_on();
  //cup_disp();
  plate_cup_bwd(); 

  //customer_fwd();
  customer_fwd1();
  puri_rotation();
  customer_fwd2();
  //plt_disp_off();
  customer_bwd();
  



}

/* setting all stepper to therir starting pos */

void all_stepper_homing()
{

  /* puri pusher homing */

  if (digitalRead(puri_catcher_home_sensor) == LOW)
  {
    puriCatcher.setSpeed(-2000); // Set speed for moving backward
    while (digitalRead(puri_catcher_home_sensor) == LOW)
    {
      puriCatcher.runSpeed(); // Continue moving backward
    }
    puriCatcher.stop();
    delay(500);
    puriCatcher.setCurrentPosition(0); // Reset position for puriCatcher after homing
  }

  /* s_cathcher1 homing */

  if (digitalRead(s_catcher_opener1_home_sensor) == LOW)
  {
    catcherOpener1.setSpeed(-4000);
    while (digitalRead(s_catcher_opener1_home_sensor) == LOW)
    {
      catcherOpener1.runSpeed();
    }
    catcherOpener1.stop();
    delay(200);
    catcherOpener1.setCurrentPosition(0);
  }

   /* s_catcher2_homing */

  if (digitalRead(s_catcher_opener2_home_sensor) == LOW)
  {
    catcherOpener2.setSpeed(4000);
    while (digitalRead(s_catcher_opener2_home_sensor) == LOW)
    {
      catcherOpener2.runSpeed();
    }
    catcherOpener2.stop();
    delay(200);
    catcherOpener2.setCurrentPosition(0);
  }

  /* conveyor homing */

  if (digitalRead(conveyor_home_sensor) == LOW)
  {
    puriConveyor.setSpeed(-2000);
    while (digitalRead(conveyor_home_sensor) == LOW)
    {
      puriConveyor.runSpeed();
    }
    puriConveyor.stop();
    delay(200);
    puriConveyor.setCurrentPosition(0);
  }

  /* drill homing */

  if(digitalRead(drill_home_sensor) == LOW){
    puriDrill.setSpeed(-4000);
    while(digitalRead(drill_home_sensor) == LOW){
      puriDrill.runSpeed();
    }
    puriDrill.stop();
    delay(200);
    puriDrill.setCurrentPosition(0);
  }

  customer_bwd();
  plate_cup_bwd();
 
}

/* 60 degree rotation of puri barrel */

void puri_barrel_rotor()
{
  if (digitalRead(puri_catcher_home_sensor) == HIGH)
  {
    while (digitalRead(ir) == HIGH)
    {
      puriBarrel.move(500);       // Move 2200 steps forward
      puriBarrel.runToPosition(); // Run to the new position
      delay(200);                 // Add a small delay to prevent rapid cycling
    }
  }

  // Ensuring the puri dropper can operate again by resetting its position
  if (digitalRead(puri_catcher_home_sensor) == LOW || digitalRead(ir) == LOW)
  {
    puriBarrel.setCurrentPosition(0); // Reset position for puriBarrel
  }
}

void puri_push()
{
  if (digitalRead(puri_catcher_home_sensor) == HIGH && digitalRead(ir) == LOW)
  {
    delay(100); // Wait before start
    puriCatcher.moveTo(14000);
    while (puriCatcher.distanceToGo() != 0)
    {
      puriCatcher.run(); // Smooth ramped move
    }
  }
}

void puri_pull()
{

   if (digitalRead(puri_catcher_home_sensor) == LOW)
  {
    puriCatcher.setSpeed(-5000); // Set speed for moving backward
    while (digitalRead(puri_catcher_home_sensor) == LOW)
    {
      puriCatcher.runSpeed(); // Continue moving backward
    }
    puriCatcher.stop();
    delay(500);
    puriCatcher.setCurrentPosition(0); // Reset position for puriCatcher after homing
  }
 }

void s_catcher_open1()
{
  if (digitalRead(s_catcher_opener1_home_sensor) == HIGH)
  {

    // Move forward
    catcherOpener1.moveTo(5000);
    while (catcherOpener1.distanceToGo() != 0)
    {
      catcherOpener1.run();
    }
  }
}

void s_catcher_close1()
{
  if (digitalRead(s_catcher_opener1_home_sensor) == LOW)
  {
    catcherOpener1.setSpeed(-4000);
    while (digitalRead(s_catcher_opener1_home_sensor) == LOW)
    {
      catcherOpener1.runSpeed();
    }

    catcherOpener1.stop();
    delay(200);
    catcherOpener1.setCurrentPosition(0); // Reset position at home
  }
}

void blower()
{

  analogWrite(blower_pwm, 200);

  delay(4000);

  analogWrite(blower_pwm, 0);
  digitalWrite(blower_solenoid, HIGH);

  delay(3000);

  digitalWrite(blower_solenoid, LOW);
}

void conveyor()
{

  puriConveyor.setSpeed(4000);

  while(digitalRead(conveyor_home_sensor) == HIGH){
    puriConveyor.runSpeed();
  }

  while(digitalRead(conveyor_home_sensor) == LOW){
    puriConveyor.runSpeed();
  }

  puriConveyor.stop();
  delay(200);
  puriConveyor.setCurrentPosition(0);
  
}

void drill(){

  if(digitalRead(drill_home_sensor) == HIGH){
    digitalWrite(drill_motor, HIGH);
    puriDrill.moveTo(7000);
    while(puriDrill.distanceToGo() != 0){
      puriDrill.run();
    }

    delay(500);

    puriDrill.setSpeed(-4000);

    while(digitalRead(drill_home_sensor) == LOW){
      puriDrill.runSpeed();
    }

    puriDrill.stop();
    delay(100);
    digitalWrite(drill_motor, LOW);

    delay(2000);

    puriDrill.setCurrentPosition(0);

  }
}

void masala(){

  digitalWrite(alu, HIGH);
  digitalWrite(onion, HIGH);
  delay(2000);
  digitalWrite(alu, LOW);
  digitalWrite(onion, LOW);
  //delay(200);
  digitalWrite(channa, HIGH);
  digitalWrite(sev, HIGH);
  delay(1000);
  digitalWrite(channa, LOW);
  digitalWrite(sev,LOW);

}

void s_catcher_open2()
{
  if (digitalRead(s_catcher_opener2_home_sensor) == HIGH)
  {

    // Move forward
    catcherOpener2.moveTo(-5000);
    while (catcherOpener2.distanceToGo() != 0)
    {
      catcherOpener2.run();
    }
  }
}

void s_catcher_close2()
{
  if (digitalRead(s_catcher_opener2_home_sensor) == LOW)
  {
    catcherOpener2.setSpeed(4000);
    while (digitalRead(s_catcher_opener2_home_sensor) == LOW)
    {
      catcherOpener2.runSpeed();
    }

    catcherOpener2.stop();
    delay(200);
    catcherOpener2.setCurrentPosition(0); // Reset position at home
  }
}
void plate_cup_fwd(){
  if (digitalRead(pc_home_sensor) == HIGH && digitalRead(plate_disp_hmp) == HIGH)
  {
    plt_disp_on();    //testing
    // Move forward
    cup_plate.moveTo(36400);
    while (cup_plate.distanceToGo() != 0)
    {
      cup_plate.run();
    }
    
    cup_disp();     //testing

  }
}

void plate_cup_bwd(){
  if (digitalRead(pc_home_sensor) == LOW && digitalRead(plate_disp_hmp) == HIGH)
  {

    // Move forward
    cup_plate.setSpeed(-5000);
    while (digitalRead(pc_home_sensor) == LOW)
    {
      cup_plate.runSpeed();
    }
    cup_plate.stop();
    delay(200);
    cup_plate.setCurrentPosition(0); // Reset position at home
  }
}

void plt_disp_on(){

  /*
   if(digitalRead(plt_ir)== LOW){
    digitalWrite(plt_mtr , HIGH);
    //delay(1000);
    //digitalWrite(plt_mtr , LOW);
  } else{
    digitalWrite(plt_mtr , LOW);
  }   */
  digitalWrite(plt_mtr , HIGH);

}

void plt_disp_off(){

  digitalWrite(plt_mtr , LOW);

}


void cup_disp(){
 
  //plt_disp();

  while(digitalRead(cup_ir)== HIGH){
    digitalWrite(cup_mtr , HIGH);
  }
    digitalWrite(cup_mtr , LOW);
    delay(100);
  
}

void customer_fwd(){
   if (digitalRead(pc_home_sensor) == HIGH && digitalRead(plate_disp_hmp) == HIGH)
  {

    // Move forward
    puri_delivery.moveTo(-12400);
    while (puri_delivery.distanceToGo() != 0)
    {
      puri_delivery.run();
    }
    delay(5000);
  }
}

void customer_fwd1(){
   if (digitalRead(pc_home_sensor) == HIGH && digitalRead(plate_disp_hmp) == HIGH)
  {

    // Move forward
    puri_delivery.moveTo(-5400);
    while (puri_delivery.distanceToGo() != 0)
    {
      puri_delivery.run();
    }
    puri_delivery.setCurrentPosition(0); // Reset position at home
    //delay(5000);
  }
}

void customer_fwd2(){
   if (digitalRead(pc_home_sensor) == HIGH)
  {

    // Move forward
    puri_delivery.moveTo(-7000);
    while (puri_delivery.distanceToGo() != 0)
    {
      puri_delivery.run();
    }
    //puri_delivery.setCurrentPosition(0); // Reset position at home
    plt_disp_off();
    delay(5000);
  }
}

void customer_bwd(){
  if (digitalRead(pc_home_sensor) == HIGH && digitalRead(plate_disp_hmp) == LOW)
  {

    // Move forward
    puri_delivery.setSpeed(3000);
    while (digitalRead(plate_disp_hmp) == LOW)
    {
      puri_delivery.runSpeed();
    }
    puri_delivery.stop();
    delay(200);
    puri_delivery.setCurrentPosition(0); // Reset position at home
  }
}

void puri_rotation(){
  for (int i = 0; i < 6; i++) {
      plate_rotate.setCurrentPosition(0);
      plate_rotate.move(25);  // Command 2 steps

      // Wait until the move is completed
      while (plate_rotate.distanceToGo() != 0) {
        plate_rotate.run();
      }

      delay(3000);  // Wait 1 second after each move
      //stepper.setCurrentPosition(0);
    }
   // delay(5000); testing purpose after completion of for loop whether its starting the for loop after 5 seconds or not
}