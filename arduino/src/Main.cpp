
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

const double pendulumLength = 0.565; // m
const double axisHeight = 0.97;
const double ticksPerTurn = 3200;
const double axisRelPos = 0.095;
const double targetLift = 0.01; // 2cm
const double angleForTargetLift = acos((pendulumLength-targetLift) / pendulumLength);
const double angleWellOverTargetLift = acos((pendulumLength-targetLift*3) / pendulumLength);

const double homePosOffsetTreshold = 0.05; // Treshold after which we activate a bias on return swings
const double positionAccuracy = 0.01; // +/- 1 cm

const double obstaclePos = 0.6;
const double dropPos = 1.2;
const double homePos = obstaclePos - sqrt(2*targetLift-targetLift*targetLift)- axisRelPos;

const double stabilization_coeff = 0.05;
const double boring_swing_coeff = -0.1;
const double pendulumSpeed_stabilized = 0.005; // rad/s
const double pendulumPos_stabilized = 0.05;

const double freq_mult = 4.16;

const double sendItSpeed = 0.2;
const double deceleration = 0.0002;

// !Modelisation

const int openAngle = 135;
const int closeAngle = 0;

ArduinoX AX_;                       // objet arduinoX
MegaServo servo_;                   // objet servomoteur
PID pid_;                           // objet PID

double lift; // How high is the end of tool

PotWrapper pendulumPot(-2.35619449, 2.35619449); // -135 to 135 deg
TicksWrapper wheelTicks((2.0*PI*wheelRadius)/(ticksPerTurn), startPos);

unsigned long last_send_time_ms = 0;
unsigned long state_start_ms = 0; // Point in time when the current state was set

int sendItAtOscil = -1;
int oscilCount = 0;
int oscilSign = 0;

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
bool stable();
bool is_at(double pos);

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
  pid_.setGains(4, 2, 0.1);
  // Attache des fonctions de retour
  pid_.setEpsilon(0.0);
  pid_.setPeriod(UPDATE_RATE_MS);

  pid_.setMeasurementFunc([]() -> double { return wheelTicks.speed(); }); //acceleration lineaire
  pid_.setCommandFunc([](double pid_voltage){ AX_.setMotorPWM(MOTOR_PIN, pid_voltage); });

  // sendMsg();
}

void loop()
{

  if(millis()-last_send_time_ms < UPDATE_RATE_MS) {
    return;
  }

  pendulumPot.update(analogRead(PENDULUMPOT_PIN));
  wheelTicks.update(AX_.readEncoder(MOTOR_PIN));
  update_eot();

  update_state();

  switch(state) {
  case State::Swinging:
  case State::LastSwing:
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
    pid_.setGoal(stabilize());
    // AX_.setMotorPWM(MOTOR_PIN, stabilize());
    break;
  case State::TakingTree:
    AX_.setMotorPWM(MOTOR_PIN, 0.0);
    if(millis()-state_start_ms < TAKE_DELAY/2) {
      digitalWrite(MAGNET_PIN, LOW);
    } else {
      digitalWrite(MAGNET_PIN, HIGH);
    }
    break;
  case State::GetToDrop:
    AX_.setMotorPWM(MOTOR_PIN, wheelTicks.position() < dropPos ? 0.1 : -0.1);
    break;
  case State::Drop:
    digitalWrite(MAGNET_PIN, LOW);
    break;
  case State::JustGonnaSendIt:
    pid_.setGoal(sendItSpeed);
    break;
  case State::JustGonnaSmoothIt:
    pid_.setGoal(max(sendItSpeed-static_cast<double>((millis()-state_start_ms))*deceleration, 0));
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
    AX_.resetEncoder(MOTOR_PIN);
    break;
  }

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
    pid_.setKp(parse_msg.as<double>());
  }
  parse_msg = doc[JSON_PID_I];
  if(!parse_msg.isNull()) {
    pid_.setKi(parse_msg.as<double>());
  }
  parse_msg = doc[JSON_PID_D];
  if(!parse_msg.isNull()) {
    pid_.setKd(parse_msg.as<double>());
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
bool stable()
{
  return abs(pendulumPot.speed()) < pendulumSpeed_stabilized && abs(pendulumPot.position()) < pendulumPos_stabilized;
}
bool is_at(double pos)
{
  return abs(wheelTicks.position()-pos) < positionAccuracy;
}
double stabilize()
{
  return stabilization_coeff * pendulumPot.position();
  // return 0.8;
}
double boring_swing()
{
  double swing_time = static_cast<double>(millis() - state_start_ms) / 1000.0;
  double base = sin(freq_mult * swing_time) * swing_time / 100.0;

  int sign = base < 0.0 ? -1 : 1;
  if(sign != oscilSign) {
    oscilSign = sign;
    oscilCount++;
  }
  if(wheelTicks.position() > homePos+homePosOffsetTreshold && sign == -1) {
    base -= 0.03;
  }/* else if(wheelTicks.position() < homePos-0.05 && sign == 1) {
    base += 0.03;
  }*/
  return base;

  // return (sin(freq_mult*swing_time) + freq_mult*swing_time*cos(freq_mult*swing_time))/50.0;
  /*
    if(stable()) {
    return 0.05;    
  }
  int mult = pendulumPot.speed() < 0 ? -1 : 1;
  double add = 0.0;
  if(abs(wheelTicks.position() - homePos) > 0.05) {
    add = wheelTicks.position() > (homePos) ? -0.03 : 0.03;
  }
  return boring_swing_coeff * cos(pendulumPot.position()) * mult + add;

  */
}
void update_eot()
{
  // EOTPos.x = wheelTicks.position() + sin(pendulumPot.position()) * pendulumLength;
  // EOTPos.y = railHeight - cos(pendulumPot.position()) * pendulumLength;
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
    if(stable()) {
      set_state(State::GetToDrop);
    }
    break;
  case State::ReturnHome:
    if(is_at(homePos)) {
      set_state(State::Ready);
    }
    break;
  case State::TakingTree:
    if(millis() - state_start_ms > TAKE_DELAY) {
      set_state(State::Swinging);
      // set_state(State::Ready);
    }
    break;
  case State::Swinging:
    if(pendulumPot.position() > angleWellOverTargetLift && sendItAtOscil == -1) {
      sendItAtOscil = oscilCount+2; // Get back and forward
      set_state(State::LastSwing);
    }
    break;
  case State::LastSwing:
    if(oscilCount >= sendItAtOscil && abs(pendulumPot.position()) > 0.05) {
      set_state(State::JustGonnaSendIt);
    }
    break;
  case State::JustGonnaSendIt:
    if(/*dropBox.contains(EOTPos) || */wheelTicks.position() > obstaclePos) {
      set_state(State::JustGonnaSmoothIt);
    }
    break;
  case State::JustGonnaSmoothIt:
    if(pid_.getGoal() <= 0.00001 || wheelTicks.position() > dropPos) {
      set_state(State::Stabilize);
    }
    break;
  case State::GetToDrop:
    if(is_at(dropPos)) {
      set_state(State::Drop);
    }
    break;
  case State::Drop:
    if(millis() - state_start_ms > DROP_DELAY) {
      set_state(State::ReturnHome);
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
  if(state == State::Error && newState != State::Ready) {
    return;
  }
  
  switch(newState) {
  case State::Swinging:
    sendItAtOscil = -1;
    oscilCount = 0;
    oscilSign = 0;
  case State::LastSwing:
  case State::JustGonnaSendIt:
  case State::JustGonnaSmoothIt:
  case State::Stabilize:
    pid_.enable();
    break;
  case State::GetToDrop:
  case State::TakingTree:
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

  if(newState != State::LastSwing)
    state_start_ms = millis();
}
