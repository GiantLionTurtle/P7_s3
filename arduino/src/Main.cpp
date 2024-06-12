
#include "Command.hpp"

#define BAUD_RATE 115200

#define MSG_SEND_INTERVAL 50 // ms

ArduinoX AX_;                       // objet arduinoX
MegaServo servo_;                   // objet servomoteur
VexQuadEncoder vexEncoder_;         // objet encodeur vex
IMU9DOF imu_;                       // objet imu
PID pid_;                           // objet PID

unsigned int last_send_time_ms = 0;

Command command;

void serialEvent();
void sendMsg();

void setup()
{
  Serial.begin(BAUD_RATE);

  AX_.init();                       // initialisation de la carte ArduinoX 
  imu_.init();                      // initialisation de la centrale inertielle
  vexEncoder_.init(2,3);            // initialisation de l'encodeur VEX
  // attache de l'interruption pour encodeur vex
  attachInterrupt(vexEncoder_.getPinInt(), []{vexEncoder_.isr();}, FALLING);
  
  // Initialisation du PID
  pid_.setGains(0.25,0.1 ,0);
  // Attache des fonctions de retour
  pid_.setEpsilon(0.001);
  pid_.setPeriod(200);

  pid_.setMeasurementFunc([]() -> double { return 0.0; });
  pid_.setCommandFunc([](double pwm){ AX_.setMotorPWM(0, -pwm); });
}

void loop()
{
  if(millis()-last_send_time_ms > MSG_SEND_INTERVAL) {
    sendMsg();
    last_send_time_ms = millis();
  }

  pid_.setGoal(command.get_torque(millis()));

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

  parse_msg = doc["RunForward"];
  if(!parse_msg.isNull()) {
     RunForward_ = parse_msg;
  }

  parse_msg = doc["setPID"];
  if(!parse_msg.isNull()) {
    pid_.disable();
    pid_.setGains(parse_msg[0], parse_msg[1], parse_msg[2]);
    pid_.setEpsilon(parse_msg[3]);
    pid_.enable();
  }

  parse_msg = doc["command"];
  if(!parse_msg.isNull()) {
    parse_msg = doc["command"]["startTime"];
    if(parse_msg.isNull())
      return;
    command.startTime_ms = parse_msg.as<unsigned int>();

    parse_msg = doc["command"]["torques"];
    if(parse_msg.isNull())
      return;

    for(int i = 0; i < N_TORQUE_SAMPLES; ++i) {
      command.Tm[i] = parse_msg[i];
    }
  }
}
void sendMsg()
{
  /* Envoit du message Json sur le port seriel */
  StaticJsonDocument<500> doc;
  // Elements du message

  doc["time"] = millis();
  doc["encVex"] = vexEncoder_.getCount();
  doc["goal"] = pid_.getGoal();
  doc["voltage"] = AX_.getVoltage();
  doc["current"] = AX_.getCurrent(); 
  doc["accelX"] = imu_.getAccelX();
  doc["accelY"] = imu_.getAccelY();
  doc["accelZ"] = imu_.getAccelZ();
  doc["gyroX"] = imu_.getGyroX();
  doc["gyroY"] = imu_.getGyroY();
  doc["gyroZ"] = imu_.getGyroZ();
  doc["isGoal"] = pid_.isAtGoal();
  doc["actualTime"] = pid_.getActualDt();

  // Serialisation
  serializeJson(doc, Serial);
  // Envoit
  Serial.println();
}