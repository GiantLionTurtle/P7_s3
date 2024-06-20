
#include "Command.hpp"
#include "TicksWrapper.hpp"
#include "PotWrapper.hpp"
#include "BoundingBox.hpp"
#include "../../common_rpiarduino/Common.hpp"

#include <ArduinoJson.h> // librairie de syntaxe JSON
#include <SPI.h> // librairie Communication SPI
#include <LibS3GRO.h>

#define MSG_SEND_INTERVAL 50 // ms
#define DROP_DELAY 200 // ms
#define TAKE_DELAY 1000 // ms
#define PENDULUMSPEED_STABILIZED 0.02

#define PENDULUMPOT_PIN A7
#define CLAWSERVO_PIN 8
#define FORWARD_PIN 17
#define BACKWARD_PIN 16
#define MOTOR_PIN 1

// Modelisation

const double pendulumLength = 0.4; // m
const double railHeight = 1.0; // m
const double wheelRadius = 0.1; // m
const double ticksPerTurn = 50.0*3200;

const double obstaclePos = 0.5;

const double stabilization_coeff = -0.2;
const double pendulumSpeed_stabilized = 0.02; // rad/s

const double homePos = 0.0;

const double maxTorque = 2;

const int clawOpen_angle = 0;
const int clawClosed_angle = 180;

BoundingBox sendItBox(Position(obstaclePos-0.1, 0.2), Position(obstaclePos+0.2, 0.1));
BoundingBox dropBox(Position(obstaclePos+0.2, 0.2), Position(obstaclePos+0.5, 0.0));
BoundingBox homeBox(Position(homePos-0.05, 0.2), Position(homePos+0.05, 0.0));

// !Modelisation

ArduinoX AX_;                       // objet arduinoX
MegaServo servo_;                   // objet servomoteur
IMU9DOF imu_;                       // objet imu
PID pid_;                           // objet PID
MegaServo clawServo_;

Position EOTPos;

PotWrapper pendulumPot(-2.35619449, 2.35619449); // -135 to 135 deg
TicksWrapper wheelTicks(2*PI/(ticksPerTurn * 1000.0), obstaclePos);

unsigned int last_send_time_ms = 0;
unsigned int state_start_ms = 0; // Point in time when the current state was set

Command command;
State state { State::Ready };

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

// Updates the state machine given 
// the current positions of things and stuff
void update_state();
// Sets the state machine and 
// sets flags
void set_state(State st);

void setup()
{
  Serial.begin(BAUD_RATE);

  AX_.init();                       // initialisation de la carte ArduinoX 
  // imu_.init();                      // initialisation de la centrale inertielle
  // pinMode(PENDULUMPOT_PIN, INPUT);
  pinMode(FORWARD_PIN, INPUT);
  pinMode(BACKWARD_PIN, INPUT);
  clawServo_.attach(CLAWSERVO_PIN);

  // Initialisation du PID
  pid_.setGains(0.25,0.1 ,0);
  // Attache des fonctions de retour
  pid_.setEpsilon(0.001);
  pid_.setPeriod(200);

  pid_.setMeasurementFunc([]() -> double { return 0.0; });
  pid_.setCommandFunc([](double pwm){ AX_.setMotorPWM(MOTOR_PIN, pwm); });
}

void loop()
{
  if(millis()-last_send_time_ms > MSG_SEND_INTERVAL) {
    sendMsg();
    last_send_time_ms = millis();
  }
  update_state();

  switch(state) {
  case State::Swinging:
    pid_.setGoal(command.get_accel(millis()));
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
      clawServo_.write(clawOpen_angle);
    } else {
      clawServo_.write(clawClosed_angle);
    }
    break;
  case State::Drop:
    clawServo_.write(clawOpen_angle);
    break;
  case State::JustGonnaSendIt:
    pid_.setGoal(maxTorque);
  case State::ShortCircuitBackward:
    AX_.setMotorPWM(MOTOR_PIN, -0.2);
    break;
  case State::ShortCircuitForward:
    AX_.setMotorPWM(MOTOR_PIN, 0.2);
    break;
  case State::Ready:
    AX_.setMotorPWM(MOTOR_PIN, 0.0);
  }

  pendulumPot.update(analogRead(PENDULUMPOT_PIN));
  wheelTicks.update(AX_.readEncoder(MOTOR_PIN));

  // Mise a jour du pid
  pid_.run();
}

// Gets called at the end of each loop if there is
// data in the serial buffer
void serialEvent()
{
  // Lecture du message Json
  StaticJsonDocument<500> doc;
  JsonVariant parse_msg;

  // Lecture sur le port Seriel
  DeserializationError error = deserializeJson(doc, Serial);

  // Si erreur dans le message
  if (error) {
    Serial.print("deserialize() failed: ");
    Serial.println(error.c_str());
    return;
  }

  parse_msg = doc["PIDGains"];
  if(!parse_msg.isNull()) {
    pid_.disable();
    pid_.setGains(parse_msg[0], parse_msg[1], parse_msg[2]);
    pid_.enable();
  }

  parse_msg = doc["command"];
  if(!parse_msg.isNull()) {
    parse_msg = doc["command"]["startTime"];
    if(parse_msg.isNull())
      return;
    command.startTime_ms = parse_msg.as<unsigned int>();

    parse_msg = doc["command"]["accels"];
    if(parse_msg.isNull())
      return;

    for(int i = 0; i < N_ACCELS_SAMPLES; ++i) {
      command.Tm[i] = parse_msg[i];
    }
  }

  // Analyse des éléments du message message
  parse_msg = doc["state"];
  if(!parse_msg.isNull()){
     state = static_cast<State>(doc["state"].as<int>());
  }
}
void sendMsg()
{
  /* Envoit du message Json sur le port seriel */
  StaticJsonDocument<500> doc;
  // Elements du message

  doc["time"] = millis();
  doc["goal"] = pid_.getGoal();

  doc["wheel"] = wheelTicks.position();
  doc["dwheel"] = wheelTicks.speed();
  doc["ddwheel"] = wheelTicks.accel();
  doc["dlin"] = wheelTicks.speed() * 2 * PI * wheelRadius;

  doc["pendulumPot"] = pendulumPot.position();
  doc["dpendulumPot"] = pendulumPot.speed();

  doc["isGoal"] = pid_.isAtGoal();
  doc["actualTime"] = pid_.getActualDt();
  doc["state"] = static_cast<int>(state);
  doc["voltage"] = AX_.getVoltage();
  doc["current"] = AX_.getCurrent();

  // Serialisation
  serializeJson(doc, Serial);
  // Envoit
  Serial.println();
}
double stabilize()
{
  return stabilization_coeff * pendulumPot.speed();
}
void update_eot()
{
  EOTPos.x = wheelTicks.position() + sin(pendulumPot.position()) * pendulumLength;
  EOTPos.y = railHeight - cos(pendulumPot.position()) * pendulumLength;
}
void update_state()
{
  if(digitalRead(FORWARD_PIN)) {
    set_state(State::ShortCircuitForward);
  } else if(digitalRead(BACKWARD_PIN)) {
    set_state(State::ShortCircuitBackward);
  }
  switch(state) {
  case State::Stabilize:
    if(pendulumPot.speed() < PENDULUMSPEED_STABILIZED) {
      set_state(State::ReturnHome);
    }
    break;
  case State::ReturnHome:
    if(homeBox.contains(EOTPos)) {
      set_state(State::Ready);
    }
    break;
  case State::TakingTree:
    if(millis() - state_start_ms > TAKE_DELAY) {
      set_state(State::Swinging);
    }
    break;
  case State::Swinging:
    if(sendItBox.contains(EOTPos)) {
      set_state(State::JustGonnaSendIt);
    }
    break;
  case State::JustGonnaSendIt:
    if(dropBox.contains(EOTPos)) {
      set_state(State::Drop);
    }
    break;
  case State::Drop:
    if(millis() - state_start_ms > DROP_DELAY) {
      set_state(State::Stabilize);
    }
    break;
  case State::ShortCircuitBackward:
    if(!digitalRead(BACKWARD_PIN)) {
      set_state(State::Ready);
    }
  case State::ShortCircuitForward:
    if(!digitalRead(FORWARD_PIN)) {
      set_state(State::Ready);
    }
  default:
    break;
  }
}
void set_state(State newState)
{
  if(state == State::ReturnHome) {
    AX_.setMotorPWM(0, 0.0);
  }
  switch(newState) {
  case State::Ready:
  case State::Stabilize:
  case State::Swinging:
  case State::JustGonnaSendIt:
  case State::Drop:
    pid_.enable();
    break;
  case State::ReturnHome:
    pid_.disable();
    break;
  }
  state = newState;
  state_start_ms = millis();
}