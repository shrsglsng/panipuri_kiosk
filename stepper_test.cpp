#include <Arduino.h>
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

void puri_1();
void puri_2();
void puri_3_to_5();
void puri_6();

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

AccelStepper puriCatcher(AccelStepper::DRIVER, puri_catcher_step, puri_catcher_dir);
AccelStepper puriBarrel(AccelStepper::DRIVER, barrel_step, barrel_dir);
AccelStepper catcherOpener1(AccelStepper::DRIVER, s_catcher_opener1_step, s_catcher_opener1_dir);
AccelStepper catcherOpener2(AccelStepper::DRIVER, s_catcher_opener2_step, s_catcher_opener2_dir);
AccelStepper puriConveyor(AccelStepper::DRIVER, conveyor_step, conveyor_dir);
AccelStepper puriDrill(AccelStepper::DRIVER, drill_step, drill_dir);

void setup()
{
  pinMode(puri_catcher_home_sensor, INPUT_PULLUP); // Set sensor pin as input with pull-up resistor
  pinMode(ir, INPUT_PULLUP);
  pinMode(s_catcher_opener1_home_sensor, INPUT);

  pinMode(blower_pwm, OUTPUT);
  pinMode(blower_taco, INPUT_PULLUP);
  pinMode(blower_solenoid, OUTPUT);

  pinMode(conveyor_home_sensor, INPUT_PULLUP);

  pinMode(drill_home_sensor, INPUT);
  pinMode(drill_motor, OUTPUT);

  pinMode(alu, OUTPUT);
  pinMode(onion, OUTPUT);
  pinMode(channa, OUTPUT);
  pinMode(sev, OUTPUT);

  puriCatcher.setMaxSpeed(5000);     // Set max speed  puri catcher
  puriCatcher.setAcceleration(4000); // Set acceleration

  puriBarrel.setMaxSpeed(2000); // barrel
  puriBarrel.setAcceleration(5000);

  catcherOpener1.setMaxSpeed(5000); // s_catcher_opener
  catcherOpener1.setAcceleration(3000);

  puriConveyor.setMaxSpeed(5000);
  puriConveyor.setAcceleration(500);

  puriDrill.setMaxSpeed(5000);
  puriDrill.setAcceleration(3000);
}

void loop()
{

  all_stepper_homing();
  delay(100);
  puri_1();
  delay(100);
  puri_2();
  delay(100);
  puri_3_to_5();
  delay(100);
  puri_3_to_5();
  delay(100);
  puri_3_to_5();
  delay(100);
  puri_6();
  delay(2000);
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

  /* conveyor homing */

  if (digitalRead(conveyor_home_sensor) == LOW)
  {
    puriConveyor.setSpeed(2000);
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

  /* s_catcher2_homing */

  if (digitalRead(s_catcher_opener2_home_sensor) == LOW)
  {
    catcherOpener2.setSpeed(-4000);
    while (digitalRead(s_catcher_opener2_home_sensor) == LOW)
    {
      catcherOpener2.runSpeed();
    }
    catcherOpener2.stop();
    delay(200);
    catcherOpener2.setCurrentPosition(0);
  }

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
    delay(200);
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

  delay(2500);

  analogWrite(blower_pwm, 0);
  digitalWrite(blower_solenoid, HIGH);

  delay(1500);

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

    delay(200);

    puriDrill.setCurrentPosition(0);

  }
}

void masala(){

  digitalWrite(alu, HIGH);
  digitalWrite(onion, HIGH);
  delay(2000);
  digitalWrite(alu, LOW);
  digitalWrite(onion, LOW);
  
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
    catcherOpener2.moveTo(5000);
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
    catcherOpener2.setSpeed(-4000);
    while (digitalRead(s_catcher_opener2_home_sensor) == LOW)
    {
      catcherOpener2.runSpeed();
    }

    catcherOpener2.stop();
    delay(200);
    catcherOpener2.setCurrentPosition(0); // Reset position at home
  }
}

/* nth-puri functions */


void puri_1(){
  puri_barrel_rotor();
  delay(100);
  s_catcher_open1();
  delay(100);
  puri_push();
  blower();
  delay(100);
  s_catcher_close1();
  delay(100);
  puri_pull();
  delay(100);
  conveyor();
}

void puri_2(){
  drill();  // puri 1 drilling
  delay(100);
  puri_barrel_rotor();
  delay(100);
  s_catcher_open1();
  delay(100);
  puri_push();
  blower();
  delay(100);
  s_catcher_close1();
  delay(100);
  puri_pull();
  delay(100);
  conveyor();

}

void puri_3_to_5(){
  masala(); //puri 1 filling
  delay(100);
  drill();  //puri 2 drilling
  delay(100);
  puri_barrel_rotor();
  delay(100);
  s_catcher_open1();
  delay(100);
  puri_push();
  blower();
  delay(100);
  s_catcher_close1();
  delay(100);
  puri_pull();
  delay(100);
  s_catcher_open2();  //puri 1 comes out
  delay(100);
  s_catcher_close2();
  delay(100);
  conveyor();

}

void puri_6(){
  masala(); //for puri 4 filling
  delay(100);
  drill();  //for puri 5 drilling
  delay(100);
  puri_barrel_rotor();
  delay(100);
  s_catcher_open1();
  delay(100);
  puri_push();
  blower();
  delay(100);
  s_catcher_close1();
  delay(100);
  puri_pull();
  delay(100);
  s_catcher_open2();  //puri 4 comes out
  delay(100);
  s_catcher_close2();
  delay(100);
  conveyor();
  delay(100);
  masala(); //for puri 5 filling
  delay(100);
  drill();  //for puri 6 drilling
  delay(100);
  s_catcher_open2();  //puri 5 comes out
  delay(100);
  s_catcher_close2();
  delay(100);
  conveyor();
  delay(100);
  masala(); //for puri 6 filling
  delay(100);
  s_catcher_open2();  // puri 6 comes out
  delay(100);
  s_catcher_close2();

}
