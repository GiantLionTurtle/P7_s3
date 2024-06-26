
#include "mainwindow.hpp"
#include "ui_mainwindow.h"

#include "SimMatch.hpp"

#include <QDebug>
#include <QtWidgets>
#include <QJsonObject>
#include <QJsonDocument>

#include <iostream>

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

      if(!jsonObj["state"].isNull()) {
        int stateint = jsonObj["state"].toInt();
        arduino_model.state = static_cast<State>(stateint);
        ui->statebox->setCurrentIndex(stateint);
      }
      if(!jsonObj["time"].isNull()) {
        arduino_model.time_ms = jsonObj["time"].toInt();
      }
      if(!jsonObj["dlin"].isNull()) {
        arduino_model.linSpeed = jsonObj["dlin"].toDouble();
      }
      if(!jsonObj["dwheel"].isNull()) {
        arduino_model.wheelAngSpeed = jsonObj["dwheel"].toDouble();
      }

      // Plot data
      scene.clear();
      currentPot.addData(jsonObj["pendulumPot"].toDouble());
      currentPot.draw(&scene);


      msgBuffer = "";
    }
    is_readingArduino_ = false;
  }
}
void MainWindow::onPeriodicUpdate()
{
  if(arduino_model.state == State::Swinging) {
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
  ui->graphicsView->setScene(&scene);
  // Plot data
  currentPot.setDataLen(300);
  currentPot.setColor(255,0,0);
  currentPot.setGain(4);
}
void MainWindow::connectComboBox()
{
  connect(ui->statebox, SIGNAL(currentIndexChanged(int)), this, SLOT(sendState(int)));
}
void MainWindow::connectSliders()
{
  connect(ui->PID_p, SIGNAL(valueChanged()), this, SLOT(setPID()));
  connect(ui->PID_i, SIGNAL(valueChanged()), this, SLOT(setPID()));
  connect(ui->PID_d, SIGNAL(valueChanged()), this, SLOT(setPID())); 
}

void MainWindow::sendCommand(std::vector<double> accels)
{ 
  QString command_str = "\"command\": {\"startTime\":" + QString::number(arduino_model.time_ms) + ",\"accels\":[";
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
void MainWindow::sendState(State state)
{
  if(!is_readingArduino_) {
    sendMessage("{\"state\": " + QString::number(static_cast<int>(state)) + "}");
  }
}
void MainWindow::setPID()
{
  serialCom->sendMessage("{\"PIDGains\":[" + 
                      QString::number(static_cast<double>(ui->PID_p->value())/10.0) + ", " + 
                      QString::number(static_cast<double>(ui->PID_i->value())/10.0) + ", " + 
                      QString::number(static_cast<double>(ui->PID_d->value())/10.0) + "]}");
}