
#include "mainwindow.hpp"
#include "ui_mainwindow.h"

#include "SimMatch.hpp"

#include <QDebug>
#include <QtWidgets>
#include <QJsonObject>
#include <QJsonDocument>

#include <iostream>

// How to scale slider values (int) to 
// actual PID gain
#define P_SLIDER_CONV 10.0f
#define I_SLIDER_CONV 10.0f
#define D_SLIDER_CONV 10.0f

MainWindow::MainWindow(QString portName, int updateRate, QWidget *parent)
  : QMainWindow(parent)
  , simulation(1.0)
{
  // Initialisation du UI
  ui = new Ui::MainWindow;
  ui->setupUi(this);


  // Fonctions de connections events/slots
  connectTimers(updateRate);
  connectPlotBoxe();  // activation du plot, mettre en commentaire si pas utilise
  connectComboBox();
  connectSliders();
  connectButtons();
  
  // Serial protocole
  serialCom = new SerialProtocol(portName, BAUD_RATE);
  connectSerialPortRead();
}

MainWindow::~MainWindow()
{
  updateTimer_.stop();
  delete serialCom;
  delete ui;
}

void MainWindow::closeEvent(QCloseEvent *event)
{
  sendMessage("{\"read\": \"false\"}"); //Arret de communication periodique de l'arduino
  event->accept();
}

void MainWindow::receiveFromSerial(QString msg) {
  // Fonction appelee lors de reception sur port serie
  // Accumulation des morceaux de message
  msgBuffer += msg;

  if(msgBuffer.endsWith('\n')) {
    is_readingArduino_ = true;

    QJsonDocument jsonResponse = QJsonDocument::fromJson(msgBuffer.toUtf8());
    if(~jsonResponse.isEmpty()) {
      QJsonObject jsonObj = jsonResponse.object();

      QString buff = jsonResponse.toJson(QJsonDocument::Indented);
      ui->textBrowser->setText(buff.mid(2,buff.length()-4));

      if(!jsonObj[JSON_STATE].isNull()) {
        int stateint = jsonObj[JSON_STATE].toInt();
        arduino_model.state = static_cast<State>(stateint);
        ui->statebox->setCurrentIndex(stateint);
      }
      if(!jsonObj[JSON_PID_P].isNull()) {
        ui->PID_p->setValue(jsonObj[JSON_PID_P].toDouble() * P_SLIDER_CONV);
      }
      if(!jsonObj[JSON_PID_I].isNull()) {
        ui->PID_i->setValue(jsonObj[JSON_PID_I].toDouble() * I_SLIDER_CONV);
      }
      if(!jsonObj[JSON_PID_D].isNull()) {
        ui->PID_d->setValue(jsonObj[JSON_PID_D].toDouble() * D_SLIDER_CONV);
      }

      if(!jsonObj[JSON_TIME].isNull()) {
        arduino_model.time_ms = jsonObj[JSON_TIME].toInt();
      }
      // if(!jsonObj["dlin"].isNull()) {
        // arduino_model.linSpeed = jsonObj["dlin"].toDouble();
      // }
      if(!jsonObj[JSON_DWHEEL].isNull()) {
        arduino_model.wheelAngSpeed = jsonObj[JSON_DWHEEL].toDouble();
      }
      if(!jsonObj[JSON_WHEEL].isNull()) {
        arduino_model.wheel_pos = jsonObj[JSON_WHEEL].toDouble();
      }
      double current = 0;
      if(!jsonObj[JSON_CURRENT].isNull()) {
        current = jsonObj[JSON_CURRENT].toDouble();
      }
      double voltage = 0;
      if(!jsonObj[JSON_VOLTAGE].isNull()) {
        voltage = jsonObj[JSON_VOLTAGE].toDouble();
      }
      dist_tot += std::abs(last_dist - arduino_model.wheel_pos);
      ui->Dist_label->setText(QString::number(dist_tot) + " m");
      last_dist = arduino_model.wheel_pos;

      double delta_s = static_cast<double>(arduino_model.time_ms-lastUpdMillis) / 1000.0;
      double freq = 1.0/delta_s;
      ui->ArduinoUpdFrq_label->setText(QString::number(freq) + " Hz");
      lastUpdMillis = arduino_model.time_ms;

      if(delta_s < 1000) {
        qDebug()<<energy<<": "<<voltage<<",  "<<current<<",  "<<delta_s<<"\n";
        energy += voltage*current*delta_s;
        ui->Energy_label->setText(QString::number(energy) + " J");
      }
      // Plot data
      scene.clear();
      currentPot.addData(jsonObj[JSON_PENDULUM].toDouble());
      currentPot.draw(&scene);
      // Ajouter donnee au chart

      graphPosition(jsonObj);

      msgBuffer = "";
      is_readingArduino_ = false;
    }

  }
}
void MainWindow::onPeriodicUpdate()
{
  if(arduino_model.state != State::Swinging) {
    return;
  }
  if(pid_tune_mode) {
    size_t start_time = arduino_model.time_ms - tune_start;
    std::vector<double> samples(N_ACCELS_SAMPLES);
    for(size_t i = 0; i < samples.size(); ++i) {
      samples[i] = pidTune_fn(start_time + (COMMAND_DURATION_MS / N_ACCELS_SAMPLES) * i);
    }
    sendCommand(samples);
  } else {
    unsigned long int est_simulation_time = last_simulation_time + (last_arduino_time-arduino_model.time_ms);
    double timeHint = static_cast<double>(est_simulation_time) / 1000.0;

    bool match_success;
    double simTime = simMatch(arduino_model.pendulum_angle, arduino_model.pendulum_dangle,
                              timeHint, 20.0, match_success);
    if(!match_success) {
      qDebug()<<"Simulation match failed T_T "<<arduino_model.pendulum_angle<<",  "<<
              arduino_model.pendulum_dangle<<",  "<<timeHint<<"\n";
      sendState(State::Stabilize);
      return;
    }

    // std::vector<double> accels = simulation.get_accels(simTime, );
    double duration_s = static_cast<double>(COMMAND_DURATION_MS) / 1000.0;
    sendCommand(simulation.RunSimulation(simTime, duration_s, N_ACCELS_SAMPLES, arduino_model.wheelAngSpeed, arduino_model.linSpeed));
  }
}
double MainWindow::pidTune_fn(unsigned int time) const
{
  time = time % 10000;
  if(time > 6000) {
    return 0.0;
  }
  double d_time = static_cast<double>(time) / 1000;
  return d_time * std::sin(d_time * 10) / 100;
}

void MainWindow::connectTimers(int updateRate) 
{
  // Fonction de connection de timers
  connect(&updateTimer_, &QTimer::timeout, this, [this] {
    onPeriodicUpdate(); // call overriden virtual function
  });
  updateTimer_.start(updateRate);
}

void MainWindow::connectSerialPortRead() 
{
  // Fonction de connection au message de la classe (serialProtocol)
  connect(serialCom, SIGNAL(newMessage(QString)), this, SLOT(receiveFromSerial(QString)));
}

void MainWindow::connectPlotBoxe() 
{
  
  ui->Pot_view->setScene(&scene);
  ui->Position_view->setScene(&scenePosition);
  // Plot data
  currentPot.setDataLen(300);
  currentPot.setColor(255,0,0);
  currentPot.setGain(25);

  currentPos.setDataLen(300);
  currentPos.setColor(255,0,0);
  currentPos.setGain(40);

  currentSpeed.setDataLen(300);
  currentSpeed.setColor(0,255,0);
  currentSpeed.setGain(5000);

  currentAccel.setDataLen(300);
  currentAccel.setColor(0,0,255);
  currentAccel.setGain(100000);

  pidTarget.setDataLen(300);
  pidTarget.setColor(0,120,120);
  pidTarget.setGain(currentAccel.getGain());
}
void MainWindow::connectComboBox()
{
  connect(ui->statebox, SIGNAL(currentIndexChanged(int)), this, SLOT(sendState(int)));
  // connect(ui->Position_selector, SIGNAL(currentIndexChanged(int)), &currentPos, SLOT(clear()));
}
void MainWindow::connectSliders()
{
  // Send P gain when slider is moved
  connect(ui->PID_p, SIGNAL(valueChanged(int)), this, SLOT(set_P(int)));

  // Send I gain when slider is moved
  connect(ui->PID_i, SIGNAL(valueChanged(int)), this, SLOT(set_I(int)));
  
  // Send D gain when slider is moved
  connect(ui->PID_d, SIGNAL(valueChanged(int)), this, SLOT(set_D(int)));
}
void MainWindow::connectButtons()
{
  connect(ui->PIDtune_btn, SIGNAL(clicked()), this, SLOT(toggle_PIDTune()));
  connect(ui->Stop_btn, SIGNAL(clicked()), this, SLOT(eStop()));
}

void MainWindow::sendCommand(std::vector<double> accels)
{ 
  QString startTime_str = QString("{\"") + JSON_COMMAND_START + "\":" + QString::number(arduino_model.time_ms) + "}";
  QString command_str = QString("{\"") + JSON_COMMAND_ACCELS + "\":[";
  for(size_t i = 0; i < accels.size(); ++i) {
    command_str += QString::number(accels[i]);
    if(i < accels.size()-1) {
      command_str += ",";
    } else {
      command_str += "]";
    }
  }
  command_str += "}";

  sendMessage(command_str);
}

void MainWindow::sendMessage(QString msg) 
{
  // Fonction d'ecriture sur le port serie
  serialCom->sendMessage(msg);
  qDebug() << msg;
}

void MainWindow::setUpdateRate(int rateMs) 
{
  // Fonction d'initialisation du Timer
  updateTimer_.start(rateMs);
}
void MainWindow::sendState(int state)
{
  if(!is_readingArduino_) {
    sendMessage(QString("{\"") + JSON_STATE + "\":" + QString::number(state) + "}");
  }
}
void MainWindow::set_P(int slider)
{
  auto val_str = QString::number(static_cast<double>(slider)/P_SLIDER_CONV);
  ui->P_label->setText(val_str);

  if(is_readingArduino_) 
    return; // Protect from potential feedback loops
  serialCom->sendMessage(QString("{\"") + JSON_PID_P + "\":" + val_str + "}");
}
void MainWindow::set_I(int slider)
{
  auto val_str = QString::number(static_cast<double>(slider)/I_SLIDER_CONV);
  ui->I_label->setText(val_str);

  if(is_readingArduino_) 
    return; // Protect from potential feedback loops
  serialCom->sendMessage(QString("{\"") + JSON_PID_I + "\":" + val_str + "}");
}
void MainWindow::set_D(int slider)
{
  auto val_str = QString::number(static_cast<double>(slider)/D_SLIDER_CONV);
  ui->D_label->setText(val_str);

  if(is_readingArduino_) 
    return; // Protect from potential feedback loops
  serialCom->sendMessage(QString("{\"") + JSON_PID_D + "\":" + val_str + "}");
}
void MainWindow::toggle_PIDTune()
{
  pid_tune_mode = !pid_tune_mode;
  if(pid_tune_mode) {
    sendState(State::Swinging);
    tune_start = arduino_model.time_ms;      
  } else {
    sendState(State::Ready);
  }
  // qDebug()<<"PIDTune: "<<pid_tune_mode<<"\n";
}
void MainWindow::eStop()
{
  sendState(State::Error);
}
void MainWindow::graphPosition(QJsonObject JsonObj)
{
  enum Kinds { Position, Speed, Acceleration };

  currentPos.addData(JsonObj[JSON_WHEEL].toDouble());
  currentSpeed.addData(JsonObj[JSON_DWHEEL].toDouble());
  currentAccel.addData(JsonObj[JSON_DDWHEEL].toDouble());
  pidTarget.addData(JsonObj[JSON_GOAL].toDouble());
  scenePosition.clear();

  switch(ui->Position_selector->currentIndex()) {
    case Position:
      currentPos.draw(&scenePosition);
      break;
    case Speed:
      currentSpeed.draw(&scenePosition);
      break;
    case Acceleration:
      currentAccel.draw(&scenePosition);
      pidTarget.draw(&scenePosition);
      break;
  }
}