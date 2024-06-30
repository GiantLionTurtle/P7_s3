#ifndef MAINWINDOW_HPP_
#define MAINWINDOW_HPP_

#include "ArduinoModel.hpp"
#include "Simulation_V2.hpp"

#include <QMainWindow>
#include <QTimer>
#include "serialprotocol.h"
#include "plot.h"
#include <QCloseEvent>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsItem>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
    Q_OBJECT
    
private:
  Simulation_V2 simulation;
  unsigned long int last_simulation_time { 0 }; // ms, the instant into the
                                                // simulation last time we checked
  unsigned long int last_arduino_time { 0 };    // ms, the instant into the arduino code
                                                // last time we checked
  ArduinoModel arduino_model;

public:
    int TIMEOUT_MS = 100; // ms
    int DEFAULT_UPDATE_RATE = 100; // ms

    explicit MainWindow(QString portName, int updateRate, QWidget *parent = 0);
    virtual ~MainWindow();
    void closeEvent(QCloseEvent *event) override;

    /* --- FUNCTIONS FOR DERIVED CLASS (TO OVERRIDE OR CALL) --- */

    // can be used in derived class
    void sendMessage(QString msg);
    void setUpdateRate(int rateMs);

    void onPeriodicUpdate();

private slots:
    void receiveFromSerial(QString);

    void sendCommand(std::vector<double> accels);
    void sendState(State state);

private:
    void connectTimers(int updateRate);
    void connectSerialPortRead();
    void connectPlotBoxe();
    void connectComboBox();
    void connectSliders();
    void setPID();

    QTimer updateTimer_;
    QString msgReceived_{""};
    QString msgBuffer{""};
    bool is_readingArduino_ {false }; // Reading flag to avoid callback loops
    SerialProtocol* serialCom;


    QGraphicsScene scene;
    Plot currentPot;

protected:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
