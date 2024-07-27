
#include "Command.hpp"
#include "TicksWrapper.hpp"
#include "PotWrapper.hpp"
#include "BoundingBox.hpp"
#include "../../common_rpiarduino/Common.hpp"

#include <ArduinoJson.h> // librairie de syntaxe JSON
#include <SPI.h> // librairie Communication SPI
#include <LibS3GRO.h>

#define DROP_DELAY 200 // ms
#define TAKE_DELAY 1000 // ms

#define PENDULUMPOT_PIN A7
#define CLAWSERVO_PIN 8
#define FORWARD_BTN_PIN 16 
#define BACKWARD_BTN_PIN 14 
#define LEFT_BTN_PIN 17
#define RIGHT_BTN_PIN 15
#define MOTOR_PIN 1
#define MAGNET_PIN 32

#define BORING_SWING

// Modelisation

const double pendulumLength = 0.4; // m
const double railHeight = 1.0; // m
const double ticksPerTurn = 3200;

const double obstaclePos = 0.5;

const double stabilization_coeff = 0.2;
const double boring_swing_coeff = -0.2;
const double pendulumSpeed_stabilized = 0.005; // rad/s
const double pendulumPos_stabilized = 0.05;

const double homePos = 0.0;

const double maxTorque = 2;

BoundingBox sendItBox(Position(obstaclePos-0.1, 0.2), Position(obstaclePos+0.2, 0.1));
BoundingBox dropBox(Position(obstaclePos+0.2, 0.2), Position(obstaclePos+0.5, 0.0));
BoundingBox homeBox(Position(homePos-0.05, 0.2), Position(homePos+0.05, 0.0));

// !Modelisation

ArduinoX AX_;                       // objet arduinoX
MegaServo servo_;                   // objet servomoteur
IMU9DOF imu_;                       // objet imu
PID pid_;                           // objet PID

Position EOTPos;

PotWrapper pendulumPot(-2.35619449, 2.35619449); // -135 to 135 deg
TicksWrapper wheelTicks((2.0*PI*wheelRadius)/(ticksPerTurn), startPos);

unsigned long last_send_time_ms = 0;
unsigned long state_start_ms = 0; // Point in time when the current state was set

Command command;
State state { State::Ready };

String serialReceived;

// Function that gets called at the end of the loop if
// a message is received on the serial buffer
// it updates the command given to the pid
// as well as other parameters
void serialEvent();
// Sends info to the raspberry pi so that it 
// can perform simulations and update the command
void sendMsg();
// Update the position of the pincer in the plane
// under the rail
void update_eot();
// Returns the motor pwm value used to
// damp-out the pendulum motion
double stabilize();

double boring_swing();

// Updates the state machine given 
// the current positions of things and stuff
void update_state();
// Sets the state machine and 
// sets flags
void set_state(State st);

void manageSerial();

void setup()
{
  Serial.begin(BAUD_RATE);

  AX_.init();                       // initialisation de la carte ArduinoX 
  
  pinMode(PENDULUMPOT_PIN, INPUT);
  pinMode(FORWARD_BTN_PIN, INPUT);
  pinMode(BACKWARD_BTN_PIN, INPUT);
  pinMode(LEFT_BTN_PIN, INPUT);
  pinMode(RIGHT_BTN_PIN, INPUT);
  pinMode(MAGNET_PIN, OUTPUT);
  
  pendulumPot.calibrate(analogRead(PENDULUMPOT_PIN));

  // Initialisation du PID
  pid_.setGains(15, 5, 2);
  // Attache des fonctions de retour
  pid_.setEpsilon(0.001);
  pid_.setPeriod(10);

  pid_.setMeasurementFunc([]() -> double { return wheelTicks.speed(); }); //acceleration lineaire
  pid_.setCommandFunc([](double pid_voltage){ AX_.setMotorPWM(MOTOR_PIN, pid_voltage); });

  // sendMsg();
}

void loop()
{
  if(millis()-last_send_time_ms < UPDATE_RATE_MS) {
    return;
  }
  update_state();

  switch(state) {
  case State::Swinging:
#ifdef BORING_SWING
    pid_.setGoal(boring_swing());
#else
    pid_.setGoal(command.get_accel(millis()));
#endif
    break;
  case State::ReturnHome:
    AX_.setMotorPWM(MOTOR_PIN, wheelTicks.position() < homePos ? 0.1 : -0.1);
    break;
  case State::Stabilize:
    AX_.setMotorPWM(MOTOR_PIN, stabilize());
    break;
  case State::TakingTree:
    AX_.setMotorPWM(MOTOR_PIN, 0.0);
    if(millis()-state_start_ms < TAKE_DELAY/2) {
      digitalWrite(MAGNET_PIN, LOW);
    } else {
      digitalWrite(MAGNET_PIN, HIGH);
    }
    break;
  case State::Drop:
    digitalWrite(MAGNET_PIN, LOW);
    break;
  case State::JustGonnaSendIt:
    pid_.setGoal(maxTorque);
    break;
  case State::ShortCircuitBackward:
    AX_.setMotorPWM(MOTOR_PIN, -0.1);
    break;
  case State::ShortCircuitForward:
    AX_.setMotorPWM(MOTOR_PIN, 0.1);
    break;
  case State::Ready:
    AX_.setMotorPWM(MOTOR_PIN, 0.0);
    break;
  case State::Error:
    AX_.setMotorPWM(MOTOR_PIN, 0.0);
    break;
  }

  pendulumPot.update(analogRead(PENDULUMPOT_PIN));
  wheelTicks.update(AX_.readEncoder(MOTOR_PIN));
  // Mise a jour du pid
  pid_.run();
  // AX_.setMotorPWM(MOTOR_PIN, 0.5);

  sendMsg();
  // Serial.println("Pouet");
  // Serial.println(pendulumPot.speed(), 8);
  // Serial.println(digitalRead(BACKWARD_PIN));
  last_send_time_ms = millis();
}

// Gets called at the end of each loop if there is
// data in the serial buffer
void serialEvent()
{
  while(Serial.available()) {
    char c = Serial.read();
    serialReceived += c;
    if(c == '\n') {
      manageSerial();
    }
  }
}
void manageSerial()
{
  // Serial.print("Got: ");
  // Serial.println(serialReceived);

  // Lecture du message Json
  StaticJsonDocument<500> doc;
  JsonVariant parse_msg;

  // Lecture sur le port Seriel
  DeserializationError error = deserializeJson(doc, serialReceived);
  serialReceived = "";

  // Si erreur dans le message
  if (error) {
    Serial.print("deserialize() failed: ");
    Serial.println(error.c_str());
    return;
  }

  // Parse msg for PID gains
  parse_msg = doc[JSON_PID_P];
  if(!parse_msg.isNull()) {
    pid_.disable();
    pid_.setKp(parse_msg.as<double>());
    pid_.enable();
  }
  parse_msg = doc[JSON_PID_I];
  if(!parse_msg.isNull()) {
    pid_.disable();
    pid_.setKi(parse_msg.as<double>());
    pid_.enable();
  }
  parse_msg = doc[JSON_PID_D];
  if(!parse_msg.isNull()) {
    pid_.disable();
    pid_.setKd(parse_msg.as<double>());
    pid_.enable();
  }

  // Parse msg for command input
  // first start time, then acceleration points
  parse_msg = doc[JSON_COMMAND_START];
  if(!parse_msg.isNull()) {
    command.startTime_ms = parse_msg.as<unsigned int>();
  }
  
  parse_msg = doc[JSON_COMMAND_VELOCITIES];
  if(!parse_msg.isNull()) {
    for(int i = 0; i < N_ACCELS_SAMPLES; ++i) {
      command.Tm[i] = parse_msg[i].as<double>();
      // Serial.print("Command: ");
      // Serial.println(command.Tm[i]);
    }
  }
  // Parse msg for state input
  parse_msg = doc[JSON_STATE];
  if(!parse_msg.isNull()) {
    // Serial.println("Set state!");
    set_state(static_cast<State>(doc[JSON_STATE].as<int>()));
  }

  // parse_msg = doc[JSON_SEND];
  // if(!parse_msg.isNull()) {
    
  //   sendArdState = true;
  // }
}
void sendMsg()
{
  /* Envoit du message Json sur le port seriel */
  StaticJsonDocument<500> doc;
  // Elements du message

  doc[JSON_TIME] = millis();

  doc[JSON_STATE] = static_cast<int>(state);

  // if(sendArdState) {
  doc[JSON_PID_P] = pid_.getKp();
  doc[JSON_PID_I] = pid_.getKi();
  doc[JSON_PID_D] = pid_.getKd();
    // sendArdState = false;
  // }
  
  doc[JSON_GOAL] = pid_.getGoal();

  doc[JSON_WHEEL] = wheelTicks.position();
  doc[JSON_DWHEEL] = wheelTicks.speed();
  doc[JSON_DDWHEEL] = wheelTicks.accel();
  // doc["dlin"] = wheelTicks.speed() * 2 * PI * wheelRadius;

  doc[JSON_PENDULUM] = pendulumPot.position();
  doc[JSON_DPENDULUM] = pendulumPot.speed();

  doc[JSON_ATGOAL] = pid_.isAtGoal();

  doc[JSON_VOLTAGE] = AX_.getVoltage();
  doc[JSON_CURRENT] = AX_.getCurrent();

  // Serialisation
  serializeJson(doc, Serial);
  // Envoit
  Serial.println();
}
double stabilize()
{
  return stabilization_coeff * pendulumPot.position();
  // return 0.8;
}
double boring_swing()
{
  return boring_swing_coeff * pendulumPot.position();
}
void update_eot()
{
  EOTPos.x = wheelTicks.position() + sin(pendulumPot.position()) * pendulumLength;
  EOTPos.y = railHeight - cos(pendulumPot.position()) * pendulumLength;
}
void update_state()
{
  if(digitalRead(BACKWARD_BTN_PIN)) {
    set_state(State::ShortCircuitBackward);
  }
  if(digitalRead(FORWARD_BTN_PIN)) {
    set_state(State::ShortCircuitForward);
  } 
  if(digitalRead(LEFT_BTN_PIN) || digitalRead(RIGHT_BTN_PIN)) {
    set_state(State::Error);
  }

  switch(state) {
  case State::Stabilize:
    if(abs(pendulumPot.speed()) < pendulumSpeed_stabilized && abs(pendulumPot.position()) < pendulumPos_stabilized) {
      set_state(State::ReturnHome);
    }
    break;
  case State::ReturnHome:
    if(abs(wheelTicks.position()-homePos) < 0.01) {
      set_state(State::Ready);
    }
    break;
  case State::TakingTree:
    if(millis() - state_start_ms > TAKE_DELAY) {
      set_state(State::Swinging);
    }
    break;
  case State::Swinging:
    if(/*EOTPos.y > (railHeight-pendulumLength+0.01) &&*/ pendulumPot.position() > 0.5) {
      set_state(State::JustGonnaSendIt);
    }
    break;
  case State::JustGonnaSendIt:
    if(dropBox.contains(EOTPos) || wheelTicks.position() > obstaclePos) {
      set_state(State::Drop);
    }
    break;
    
  case State::Drop:
    if(millis() - state_start_ms > DROP_DELAY) {
      set_state(State::Stabilize);
    }
    break;
    
  case State::ShortCircuitBackward:
    if(!digitalRead(BACKWARD_BTN_PIN)) {
      set_state(State::Ready);
    }
    break;
  case State::ShortCircuitForward:
    if(!digitalRead(FORWARD_BTN_PIN)) {
      set_state(State::Ready);
    }
    break;
  default:
    break;
  }
}

void set_state(State newState)
{
  Serial.print("Setstate ");
  Serial.println(static_cast<int>(newState));
  if(state == State::Error && newState != State::Ready) {
    return;
  }
  
  switch(newState) {
  case State::Swinging:
  case State::JustGonnaSendIt:
    pid_.enable();
    break;
  case State::TakingTree:
  case State::Stabilize:
  case State::Drop:
  case State::ReturnHome:
  case State::Error:
  case State::ShortCircuitForward:
  case State::ShortCircuitBackward:
  case State::Ready:
    pid_.disable();
    break;
  default:
    break;
  }
  state = newState;
  state_start_ms = millis();
}