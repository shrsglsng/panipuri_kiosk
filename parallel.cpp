#include <Arduino.h>
#include <AccelStepper.h>

#define MAX_POSITION -5000

void all_axis_homing();
void s1_s2_catcher_open();
void s1_s2_catcher_close();
void puri_push_drill_down();
void puri_pull_drill_up();

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

    puriCatcher.setMaxSpeed(5000);
    puriCatcher.setAcceleration(500);

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
    puri_push_drill_down();
    delay(2000);
    puri_pull_drill_up();
    delay(2000);
}

void all_axis_homing()
{

    while (digitalRead(puri_catcher_home_sensor) == LOW || digitalRead(s_catcher_opener1_home_sensor) == LOW || digitalRead(s_catcher_opener2_home_sensor) == LOW || digitalRead(drill_home_sensor) == LOW || digitalRead(plate_cup_horizontal_home_sensor) == LOW || digitalRead(plate_cup_vertical_home_sensor) == LOW)
    {

        if (digitalRead(puri_catcher_home_sensor) == LOW)
        {

            puriCatcher.setSpeed(-4000);
            puriCatcher.runSpeed();
        }

        if (digitalRead(s_catcher_opener1_home_sensor) == LOW)
        {
            catcherOpener1.setSpeed(-4000);
            catcherOpener1.runSpeed();
        }

        if (digitalRead(s_catcher_opener2_home_sensor) == LOW)
        {
            catcherOpener2.setSpeed(4000);
            catcherOpener2.runSpeed();
        }

        if (digitalRead(drill_home_sensor) == LOW)
        {
            puriDrill.setSpeed(-4000);
            puriDrill.runSpeed();
        }

        if (digitalRead(plate_cup_horizontal_home_sensor) == LOW)
        {
            pcHorizontal.setSpeed(3000);
            pcHorizontal.runSpeed();
        }

        if (digitalRead(plate_cup_vertical_home_sensor) == LOW)
        {
            pcVertical.setSpeed(-5000);
            pcVertical.runSpeed();
        }
    }

    while (digitalRead(conveyor_home_sensor) == LOW)
    {
        if (digitalRead(conveyor_home_sensor) == LOW)
        {
            puriCatcher.setSpeed(4000);
            puriCatcher.runSpeed();
        }
    }

    puriCatcher.setCurrentPosition(0);
    catcherOpener1.setCurrentPosition(0);
    catcherOpener2.setCurrentPosition(0);
    puriDrill.setCurrentPosition(0);
    pcHorizontal.setCurrentPosition(0);
    pcVertical.setCurrentPosition(0);
}

void s1_s2_catcher_open()
{

    if (digitalRead(s_catcher_opener1_home_sensor) == HIGH && digitalRead(s_catcher_opener2_home_sensor) == HIGH)
    {
        catcherOpener1.moveTo(5000);
        catcherOpener2.moveTo(-5000);

        while (catcherOpener1.distanceToGo() != 0 && catcherOpener2.distanceToGo() != 0)
        {
            catcherOpener1.run();
            catcherOpener2.run();
        }
    }
    else
    {
        all_axis_homing();
    }
}

void s1_s2_catcher_close()
{

    while (digitalRead(s_catcher_opener1_home_sensor) == LOW || digitalRead(s_catcher_opener2_home_sensor) == LOW)
    {

        if (digitalRead(s_catcher_opener1_home_sensor) == LOW)
        {
            catcherOpener1.setSpeed(-4000);
            catcherOpener1.runSpeed();
        }

        if (digitalRead(s_catcher_opener2_home_sensor) == LOW)
        {
            catcherOpener2.setSpeed(4000);
            catcherOpener2.runSpeed();
        }
    }

    catcherOpener1.setCurrentPosition(0);
    catcherOpener2.setCurrentPosition(0);
}

void puri_push_drill_down()
{

    if (digitalRead(puri_catcher_home_sensor) == HIGH && digitalRead(drill_home_sensor) == HIGH)
    {

        puriCatcher.moveTo(14000);
        puriDrill.moveTo(7000);

        digitalWrite(drill_motor, HIGH);

        while (puriCatcher.distanceToGo() != 0 || puriDrill.distanceToGo() != 0)
        {
            if (puriCatcher.distanceToGo() != 0)
            {
                puriCatcher.setSpeed(4000);
                puriCatcher.runSpeed();
            }
            if (puriDrill.distanceToGo() != 0)
            {
                puriDrill.setSpeed(4000);
                puriDrill.runSpeed();
            }
        }
    }
    else
    {
        all_axis_homing();
    }
}

void puri_pull_drill_up(){

    while(digitalRead(puri_catcher_home_sensor) == LOW || digitalRead(drill_home_sensor) == LOW){

        if(digitalRead(puri_catcher_home_sensor) == LOW){
            puriCatcher.setSpeed(-4000);
            puriCatcher.runSpeed();
        }
        if(digitalRead(drill_home_sensor) == LOW){
            puriDrill.setSpeed(-4000);
            puriDrill.runSpeed();
        }

    }

    digitalWrite(drill_motor, LOW);

    puriCatcher.setCurrentPosition(0);
    puriDrill.setCurrentPosition(0);

}