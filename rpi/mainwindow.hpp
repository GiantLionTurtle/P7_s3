#ifndef MAINWINDOW_HPP_
#define MAINWINDOW_HPP_

#include "ArduinoModel.hpp"

#include <QMainWindow>
#include <QTimer>
#include "serialprotocol.h"
#include "plot.h"
#include <QCloseEvent>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsItem>
#include <QtWidgets>

#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
    Q_OBJECT
    
private:
  unsigned long int last_arduino_time { 0 };    // ms, the instant into the arduino code
                                                // last time we checked
  ArduinoModel arduino_model;

  bool pid_tune_mode { false };
  size_t tune_start { 0 };

public:
  int TIMEOUT_MS = 100; // ms

  explicit MainWindow(QString portName, int updateRate, QWidget *parent = 0);
  virtual ~MainWindow();
  void closeEvent(QCloseEvent *event) override;

  /* --- FUNCTIONS FOR DERIVED CLASS (TO OVERRIDE OR CALL) --- */

  // can be used in derived class
  void sendMessage(QString msg);
  void setUpdateRate(int rateMs);

  void onPeriodicUpdate();

  void graphPosition(QJsonObject JsonObj);

private slots:
  void receiveFromSerial(QString);

  void sendCommand(std::vector<double> accels);
  void sendState(int state);
  void sendState(State state) { sendState(static_cast<int>(state)); }
  void set_P(int slider);
  void set_I(int slider);
  void set_D(int slider);
  void toggle_PIDTune();
  void eStop();

private:
  void connectTimers(int updateRate);
  void connectSerialPortRead();
  void connectPlotBoxe();
  void connectComboBox();
  void connectSliders();
  void connectButtons();

  double pidTune_fn(unsigned int time) const;

  QTimer updateTimer_;
  QString msgReceived_{""};
  QString msgBuffer{""};
  bool is_readingArduino_ {false }; // Reading flag to avoid callback loops
  SerialProtocol* serialCom;

  QLineSeries seriesPot_;
  QChart chartPot_;

  QGraphicsScene scene;
  QGraphicsScene scenePosition;
  //Plot currentPot;
  Plot currentPos;
  Plot currentSpeed;
  Plot currentAccel;
  Plot pidTarget;
  size_t lastUpdMillis { 0 };

  double dist_tot { 0 };
  double last_dist { startPos };
  double energy { 0 };
  bool notInitated { true };

protected:
  Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
