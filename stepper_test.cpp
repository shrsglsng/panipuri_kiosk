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

void plate_cup_up();
void plate_cup_down();
void plate_vaccum_on();
void plate_vaccum_off();
void cup_dispense();

void plate_cup_fwd_till_masala();
void plate_cup_fwd_final();
void plate_cup_bwd();
void plate_rotate();
void get_plate_cup();

void water_refill();
void sweet_pani();
void spicy_pani();

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

const int plate_cup_vertical_home_sensor = A6; // LIZ
const int plate_cup_vertical_step = 37;        // NBR step
const int plate_cup_vertical_dir = 36;         // NBR dir

const int plate_cup_horizontal_home_sensor = A7; // LID
const int plate_cup_horizontal_step = 35;        // NPZ step
const int plate_cup_horizontal_dir = 34;         // NPZ dir

const int plate_vaccum = 5;        // FIG
const int plate_detect_ir = A8;    // LIY
const int cup_dispensor_motor = 4; // FIC
const int cup_detect_ir = A11;     // LEX

const int plate_rotor_step = 33; // NEX
const int plate_rotor_dir = 32;

const int water_refill_pump = 3;         // FIA
const int water_tank_float_sensor = A14; // FLA

const int spicy_sol = 2;     // FLA
const int spicy_mixer = 16;  // FPC
const int spicy_flavor = 17; // FPW

const int sweet_sol = 18;    // FCD
const int sweet_mixer = 19;  // FX1
const int sweet_flavor = 23; // TDL

AccelStepper puriCatcher(AccelStepper::DRIVER, puri_catcher_step, puri_catcher_dir);
AccelStepper puriBarrel(AccelStepper::DRIVER, barrel_step, barrel_dir);
AccelStepper catcherOpener1(AccelStepper::DRIVER, s_catcher_opener1_step, s_catcher_opener1_dir);
AccelStepper catcherOpener2(AccelStepper::DRIVER, s_catcher_opener2_step, s_catcher_opener2_dir);
AccelStepper puriConveyor(AccelStepper::DRIVER, conveyor_step, conveyor_dir);
AccelStepper puriDrill(AccelStepper::DRIVER, drill_step, drill_dir);
AccelStepper pcVertical(AccelStepper::DRIVER, plate_cup_vertical_step, plate_cup_vertical_dir);
AccelStepper pcHorizontal(AccelStepper::DRIVER, plate_cup_horizontal_step, plate_cup_horizontal_dir);
AccelStepper plateRotor(AccelStepper::DRIVER, plate_rotor_step, plate_rotor_dir);

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

  pinMode(plate_cup_vertical_home_sensor, INPUT);
  pinMode(plate_cup_horizontal_home_sensor, INPUT);

  pinMode(plate_detect_ir, INPUT);
  pinMode(cup_detect_ir, INPUT);
  pinMode(plate_vaccum, OUTPUT);
  pinMode(cup_dispensor_motor, OUTPUT);

  pinMode(water_tank_float_sensor, INPUT);
  pinMode(water_refill_pump, OUTPUT);

  pinMode(spicy_sol, OUTPUT);
  pinMode(spicy_flavor, OUTPUT);
  pinMode(spicy_mixer, OUTPUT);

  pinMode(sweet_sol, OUTPUT);
  pinMode(sweet_flavor, OUTPUT);
  pinMode(sweet_mixer, OUTPUT);

  puriCatcher.setMaxSpeed(5000);     // Set max speed  puri catcher
  puriCatcher.setAcceleration(4000); // Set acceleration

  puriBarrel.setMaxSpeed(2000); // barrel
  puriBarrel.setAcceleration(5000);

  catcherOpener1.setMaxSpeed(5000); // s_catcher_opener
  catcherOpener1.setAcceleration(3000);

  catcherOpener2.setMaxSpeed(5000); // s_catcher_opener
  catcherOpener2.setAcceleration(3000);

  puriConveyor.setMaxSpeed(5000);
  puriConveyor.setAcceleration(500);

  puriDrill.setMaxSpeed(5000);
  puriDrill.setAcceleration(3000);

  pcVertical.setMaxSpeed(9000);
  pcVertical.setAcceleration(5000);

  pcHorizontal.setMaxSpeed(9000);
  pcHorizontal.setAcceleration(5000);

  plateRotor.setMaxSpeed(5000);
  plateRotor.setAcceleration(3000);
}

void loop()
{

  all_stepper_homing();
  delay(100);
  get_plate_cup();
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

/* setting all stepper to their starting pos... 2 action modular code can be homed by simple function call like push()-pull() */

void all_stepper_homing()
{

  /* puri pusher homing */

  puri_pull();

  /* s_cathcher1 homing */

  s_catcher_close1();

  /* s_catcher2_homing */

  s_catcher_close2();

  /* drill homing */

  if (digitalRead(drill_home_sensor) == LOW)
  {
    puriDrill.setSpeed(-4000);
    while (digitalRead(drill_home_sensor) == LOW)
    {
      puriDrill.runSpeed();
    }
    puriDrill.stop();
    delay(200);
    puriDrill.setCurrentPosition(0);
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

  /* plate_cup homing */

  plate_cup_down();
  plate_cup_bwd();

  /* water tank refill */

  water_refill();
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

  analogWrite(blower_pwm, 250);

  delay(2500);

  analogWrite(blower_pwm, 0);
  digitalWrite(blower_solenoid, HIGH);

  delay(1500);

  digitalWrite(blower_solenoid, LOW);
}

void conveyor()
{

  puriConveyor.setSpeed(4000);

  while (digitalRead(conveyor_home_sensor) == HIGH)
  {
    puriConveyor.runSpeed();
  }

  while (digitalRead(conveyor_home_sensor) == LOW)
  {
    puriConveyor.runSpeed();
  }

  puriConveyor.stop();
  delay(200);
  puriConveyor.setCurrentPosition(0);
}

void drill()
{

  if (digitalRead(drill_home_sensor) == HIGH)
  {
    digitalWrite(drill_motor, HIGH);
    puriDrill.moveTo(7000);
    while (puriDrill.distanceToGo() != 0)
    {
      puriDrill.run();
    }

    delay(500);

    puriDrill.setSpeed(-4000);

    while (digitalRead(drill_home_sensor) == LOW)
    {
      puriDrill.runSpeed();
    }

    puriDrill.stop();
    delay(100);
    digitalWrite(drill_motor, LOW);

    delay(200);

    puriDrill.setCurrentPosition(0);
  }
}

void masala()
{

  digitalWrite(alu, HIGH);
  digitalWrite(onion, HIGH);
  delay(2000);
  digitalWrite(alu, LOW);
  digitalWrite(onion, LOW);

  digitalWrite(channa, HIGH);
  digitalWrite(sev, HIGH);
  delay(1000);
  digitalWrite(channa, LOW);
  digitalWrite(sev, LOW);
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

void plate_vaccum_on()
{
  digitalWrite(plate_vaccum, HIGH);
}

void plate_vaccum_off()
{
  digitalWrite(plate_vaccum, LOW);
}

void cup_dispense()
{
  while (digitalRead(cup_detect_ir) == HIGH)
  {
    digitalWrite(cup_dispensor_motor, HIGH);
  }
  digitalWrite(cup_dispensor_motor, LOW);
  delay(100);
}

void plate_cup_up()
{
  if (digitalRead(plate_cup_vertical_home_sensor) == HIGH)
  {
    plate_vaccum_on();
    pcVertical.moveTo(36400);

    while (pcVertical.distanceToGo() != 0)
    {
      pcVertical.run();
    }

    cup_dispense();
  }
}

void plate_cup_down()
{
  if (digitalRead(plate_cup_vertical_home_sensor) == LOW)
  {
    pcVertical.setSpeed(-4000);
    while (digitalRead(plate_cup_vertical_home_sensor) == LOW)
    {
      pcVertical.runSpeed();
    }
    pcVertical.stop();
    delay(100);
    pcVertical.setCurrentPosition(0);
  }
}

void plate_cup_fwd_till_masala()
{
  if (digitalRead(plate_cup_horizontal_home_sensor) == HIGH)
  {

    pcHorizontal.moveTo(-5400);

    while (pcHorizontal.distanceToGo() != 0)
    {
      pcHorizontal.run();
    }

    pcHorizontal.setCurrentPosition(0);
  }
}

void plate_cup_fwd_final()
{

  pcHorizontal.moveTo(-7000);

  while (pcHorizontal.distanceToGo() != 0)
  {
    pcHorizontal.run();
  }

  delay(100);

  plate_vaccum_off();

  while (digitalRead(plate_detect_ir) == LOW && digitalRead(cup_detect_ir) == LOW)
  {
  }
}

void plate_cup_bwd()
{
  if (digitalRead(plate_cup_horizontal_home_sensor) == LOW)
  {
    pcHorizontal.setSpeed(4000);
    while (digitalRead(plate_cup_horizontal_home_sensor) == LOW)
    {
      pcHorizontal.runSpeed();
    }

    pcHorizontal.stop();
    delay(100);
    pcHorizontal.setCurrentPosition(0);
  }
}

void plate_rotate()
{
  plateRotor.setCurrentPosition(0);
  plateRotor.move(30);

  while (plateRotor.distanceToGo() != 0)
  {
    plateRotor.run();
  }
}

/* nth-puri functions */

void puri_1()
{
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

void puri_2()
{
  drill(); // puri 1 drilling
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

void puri_3_to_5()
{
  masala(); // puri 1 filling
  delay(100);
  drill(); // puri 2 drilling
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
  s_catcher_open2(); // puri 1 comes out
  delay(400);
  s_catcher_close2();
  delay(100);
  plate_rotate();
  delay(100);
  conveyor();
}

void puri_6()
{
  masala(); // for puri 4 filling
  delay(100);
  drill(); // for puri 5 drilling
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
  s_catcher_open2(); // puri 4 comes out
  delay(100);
  s_catcher_close2();
  delay(100);
  plate_rotate();
  delay(100);
  conveyor();
  delay(100);
  masala(); // for puri 5 filling
  delay(100);
  drill(); // for puri 6 drilling
  delay(100);
  s_catcher_open2(); // puri 5 comes out
  delay(100);
  s_catcher_close2();
  delay(100);
  plate_rotate();
  delay(100);
  conveyor();
  delay(100);
  masala(); // for puri 6 filling
  delay(100);
  s_catcher_open2(); // puri 6 comes out
  delay(100);
  s_catcher_close2();
  delay(100);
  plate_cup_fwd_final();
}

void get_plate_cup()
{
  plate_cup_up();
  delay(100);
  plate_cup_down();
  delay(100);
  plate_cup_fwd_till_masala();
  delay(100);
  spicy_pani();
}

void water_refill()
{
  if (digitalRead(water_tank_float_sensor) == LOW)
  {
    while (digitalRead(water_tank_float_sensor) == LOW)
    {
      digitalWrite(water_refill_pump, HIGH);
    }

    digitalWrite(water_refill_pump, LOW);
  }
  else
  {
    digitalWrite(water_refill_pump, LOW);
  }
}

void sweet_pani(){

  digitalWrite(sweet_flavor, HIGH); 
  delay(1000);  //change taste here
  digitalWrite(sweet_flavor,LOW);
  digitalWrite(sweet_sol,HIGH);
  delay(300);
  digitalWrite(sweet_mixer, HIGH);
  delay(6500);
  digitalWrite(sweet_mixer, LOW);
  digitalWrite(sweet_sol, LOW);

}

void spicy_pani(){

  digitalWrite(spicy_flavor, HIGH);
  delay(1000);  //change taste here
  digitalWrite(spicy_flavor, LOW);
  digitalWrite(spicy_sol, HIGH);
  delay(300);
  digitalWrite(spicy_mixer, HIGH);
  delay(6500);
  digitalWrite(spicy_sol, LOW);
  digitalWrite(spicy_mixer, LOW);
}