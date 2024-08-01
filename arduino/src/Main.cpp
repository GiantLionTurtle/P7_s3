
#include "Command.hpp"
#include "TicksWrapper.hpp"
#include "PotWrapper.hpp"
#include "BoundingBox.hpp"
#include "Pince.hpp"
#include "../../common_rpiarduino/Common.hpp"

#include <ArduinoJson.h> // librairie de syntaxe JSON
#include <SPI.h> // librairie Communication SPI
#include <LibS3GRO.h>

#define DROP_DELAY 500 // ms
#define TAKE_DELAY 2000 // ms

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

const double pendulumLength = 0.58; // m
const double axisHeight = 0.97;
const double ticksPerTurn = 3200;
const double axisRelPos = 0.095;
const double targetLift = 0.04; // 2cm
const double angleForTargetLift = acos((pendulumLength-targetLift) / pendulumLength);
const double angleWellOverTargetLift = acos((pendulumLength-targetLift/5.0) / pendulumLength);

const double homePosOffsetTreshold = 0.02; // Treshold after which we activate a bias on return swings
const double positionAccuracy = 0.01; // +/- 1 cm

const double calibrateStartPos = 0.05;
const double obstaclePos = 0.6;
const double dropPos = 1.2;
const double homePos = obstaclePos - sqrt(2*targetLift-targetLift*targetLift)- axisRelPos-0.1;

const double stabilization_coeff = 1.8;
const double pendulumSpeed_stabilized = 0.5; // rad/s
const double pendulumPos_stabilized = 0.05;

const double freq_mult = 1.0/(2*PI*(sqrt(pendulumLength/9.81)));
const double Tt = 2.25/freq_mult;
const double Amp = acos((-targetLift/pendulumLength)+1)+0.25;

const double sendItSpeed = 5.0;
const double acceleration = 2.2;
const double deceleration = 1.6;

double initSpeed = 0.0;

// !Modelisation

double maxSendedItSpeed = 0.0;
double lastSwingSpeed = 0.0;

ArduinoX AX_;                       // objet arduinoX
Pince pince;
PID pidSpeed;                           // objet PID
PID pidPos; 

PotWrapper pendulumPot(-2.35619449, 2.35619449); // -135 to 135 deg
TicksWrapper wheelTicks((2.0*PI*wheelRadius)/(ticksPerTurn), startPos);

unsigned long last_send_time_ms = 0;
unsigned long state_start_ms = 0; // Point in time when the current state was set

int sendItAtOscil = 4;
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
  pince.Init(13);

  pinMode(PENDULUMPOT_PIN, INPUT);
  pinMode(FORWARD_BTN_PIN, INPUT);
  pinMode(BACKWARD_BTN_PIN, INPUT);
  pinMode(LEFT_BTN_PIN, INPUT);
  pinMode(RIGHT_BTN_PIN, INPUT);
  pinMode(MAGNET_PIN, OUTPUT);
  
  pendulumPot.calibrate(analogRead(PENDULUMPOT_PIN));

  // Initialisation du PID
  pidSpeed.setGains(0.6, 0.2, 0.015);
  // Attache des fonctions de retour
  pidSpeed.setEpsilon(0.0);
  pidSpeed.setPeriod(UPDATE_RATE_MS);

  pidSpeed.setMeasurementFunc([]() -> double { return wheelTicks.speed(); }); //acceleration lineaire
  pidSpeed.setCommandFunc([](double pidSpeedvoltage){ AX_.setMotorPWM(MOTOR_PIN, pidSpeedvoltage); });

  pidPos.setGains(1.2, 0.7, 0.009);
  pidPos.setEpsilon(0.0);
  pidPos.setPeriod(UPDATE_RATE_MS);
  pidPos.setMeasurementFunc([]() -> double { return wheelTicks.position(); });
  pidPos.setCommandFunc([](double pidSpeedvoltage) { AX_.setMotorPWM(MOTOR_PIN, pidSpeedvoltage); });
}

void loop()
{
  double dt = millis() - last_send_time_ms;
  if(dt < UPDATE_RATE_MS) {
    return;
  }
  dt /= 1000.0;
  pendulumPot.update(analogRead(PENDULUMPOT_PIN), dt);
  wheelTicks.update(AX_.readEncoder(MOTOR_PIN), dt);
  update_eot();

  update_state();

  switch(state) {
  case State::Swinging:
#ifdef BORING_SWING
    pidSpeed.setGoal(boring_swing());
    lastSwingSpeed = wheelTicks.speed();
#else
    pidSpeed.setGoal(command.get_accel(millis()));
#endif
    break;
  case State::GetToCalibratePos:
    pidPos.setGoal(calibrateStartPos);
    break;
  case State::Calibrate:
    AX_.setMotorPWM(MOTOR_PIN, -0.1);
    break;
  case State::ReturnHome:
    // AX_.setMotorPWM(MOTOR_PIN, wheelTicks.position() < homePos ? 0.1 : -0.1);
    pidPos.setGoal(homePos);
    break;
  case State::Stabilize:
    pidSpeed.setGoal(stabilize());
    // AX_.setMotorPWM(MOTOR_PIN, stabilize());
    break;
  case State::TakingTree:
    AX_.setMotorPWM(MOTOR_PIN, 0.0);
    if(millis()-state_start_ms < (TAKE_DELAY-100)) {
      pince.Open();
    } else {
      pince.close();
    }
    break;
  case State::BuildUp:
    AX_.setMotorPWM(MOTOR_PIN, wheelTicks.position() < (dropPos-axisRelPos) ? 0.1 : -0.1);
    // pidPos.setGoal(dropPos);
    break;
  case State::Drop:
    pince.Open();
    break;
  case State::JustGonnaSendIt:
    maxSendedItSpeed = max(maxSendedItSpeed, wheelTicks.speed());
    pidSpeed.setGoal(min(sendItSpeed, initSpeed + static_cast<double>(millis()-state_start_ms)/1000.0*acceleration));
    break;
  case State::JustGonnaSmoothIt:
    pidSpeed.setGoal(max(maxSendedItSpeed-static_cast<double>((millis()-state_start_ms))/1000.0*deceleration, 0));
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
  pidSpeed.run();
  pidPos.run();
  // AX_.setMotorPWM(MOTOR_PIN, 0.5);

  sendMsg();
  // {"s":4}
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
    pidSpeed.setKp(parse_msg.as<double>());
  }
  parse_msg = doc[JSON_PID_I];
  if(!parse_msg.isNull()) {
    pidSpeed.setKi(parse_msg.as<double>());
  }
  parse_msg = doc[JSON_PID_D];
  if(!parse_msg.isNull()) {
    pidSpeed.setKd(parse_msg.as<double>());
  }
  // 
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
  doc[JSON_PID_P] = pidSpeed.getKp();
  doc[JSON_PID_I] = pidSpeed.getKi();
  doc[JSON_PID_D] = pidSpeed.getKd();
    // sendArdState = false;
  // }
  
  doc[JSON_GOAL] = pidSpeed.getGoal();

  doc[JSON_WHEEL] = wheelTicks.position();
  doc[JSON_DWHEEL] = wheelTicks.speed();
  doc[JSON_DDWHEEL] = wheelTicks.accel();
  // doc["dlin"] = wheelTicks.speed() * 2 * PI * wheelRadius;

  doc[JSON_PENDULUM] = pendulumPot.position();
  doc[JSON_DPENDULUM] = pendulumPot.speed();

  doc[JSON_ATGOAL] = pidSpeed.isAtGoal();

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
  //double base = sin(freq_mult * swing_time)*swing_time/2;
  double base = Amp*(Tt*sin(14.14*swing_time/Tt)+14.14*swing_time*cos(14.14*swing_time/Tt))/pow(Tt, 2);

  int sign = base < 0.0 ? -1 : 1;
  if(sign != oscilSign) {
    oscilSign = sign;
    oscilCount++;
  }
  // if(wheelTicks.position() > (homePos+homePosOffsetTreshold) && sign == -1) {
  //   base -= 0.5;
  // }

  return base;
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
      set_state(State::BuildUp);
    }
    break;
  case State::GetToCalibratePos:
    if(is_at(calibrateStartPos)) {
      set_state(State::Calibrate);
    }
    break;
  case State::Calibrate:
    if(abs(wheelTicks.speed()) < 0.00000001 && millis() - state_start_ms > 200) {
      AX_.resetEncoder(MOTOR_PIN);
      set_state(State::ReturnHome);
    }
    break;
  case State::ReturnHome:
    if(is_at(homePos) && wheelTicks.speed() < 0.001) {
      // set_state(State::Ready);
      set_state(State::TakingTree);
    }
    break;
  case State::TakingTree:
    if(millis() - state_start_ms > TAKE_DELAY) {
      set_state(State::Swinging);
      // set_state(State::Ready);
    }
    break;
  case State::Swinging:
    // if(pendulumPot.position() > angleWellOverTargetLift && sendItAtOscil == -1) {
    //   sendItAtOscil = oscilCount+2; // Get back and forward
    //   set_state(State::LastSwing);
    // }
    // if(oscilCount >= sendItAtOscil && abs(pendulumPot.position()) > 0.01) {
    // if(oscilCount >= sendItAtOscil && pendulumPot.position() < 0.0 && pendulumPot.speed() < 0.0) { 
    // if(pendulumPot.position() >= 0.14)
    //   Serial.println("Position ok");
    // if(pendulumPot.speed() >= 1.3)
    //   Serial.println("Vel ok");
    //maxAngularVelReached = max(maxAngularVelReached, pendulumPot.speed());
    //maxAngleReached= max(maxAngleReached, abs(pendulumPot.position()));
    // if(pendulumPot.position() <= -0.3 && pendulumPot.speed() <= -4) {
    //if(pendulumPot.position() >= 0.14 && pendulumPot.speed() <= -1.3) {
    if((millis() - state_start_ms)/1000.0 > Tt-0.75)
    {
        set_state(State::JustGonnaSendIt);
        initSpeed = wheelTicks.speed();
        // Serial.println("End");
        // Serial.println(pendulumPot.position()*180/PI);
        // Serial.println(pendulumPot.speed()*180/PI);
    }

    break;
  case State::JustGonnaSendIt:
    if(/*dropBox.contains(EOTPos) || */wheelTicks.position() > obstaclePos + 0.25) {
        set_state(State::JustGonnaSmoothIt);
        initSpeed = 0.0;
    }
    break;
  case State::JustGonnaSmoothIt:
    if(wheelTicks.speed() <= 0.05/*|| wheelTicks.position() > dropPos+0.2*/) {
      set_state(State::Stabilize);
    }
    break;
  case State::BuildUp:
    if(is_at(dropPos-axisRelPos)) {
      set_state(State::Drop);
    }
    break;
  case State::Drop:
    if(millis() - state_start_ms > DROP_DELAY) {
      set_state(State::GetToCalibratePos);
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
  case State::GetToCalibratePos:
  case State::ReturnHome:
    pidSpeed.disable();
    pidPos.enable();
    break;
  case State::Swinging:
    oscilCount = 0;
    oscilSign = 0;
    maxSendedItSpeed = 0;
  case State::JustGonnaSendIt:
  case State::JustGonnaSmoothIt:
  case State::Stabilize:
    if(!pidSpeed.enabled()) {
      pidSpeed.enable();
      pidPos.disable();
    }
    break;
  case State::BuildUp:
  case State::TakingTree:
  case State::Drop:
  case State::Error:
  case State::ShortCircuitForward:
  case State::ShortCircuitBackward:
  case State::Ready:
  case State::Calibrate:
    pidSpeed.disable();
    pidPos.disable();
    break;
  default:
    break;
  }
  state = newState;

  state_start_ms = millis();
}
