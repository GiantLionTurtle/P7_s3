/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.15.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets/QWidget>
#include "qchartview.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QGridLayout *gridLayout;
    QTextBrowser *textBrowser;
    QLabel *D_label;
    QPushButton *Go_btn;
    QSlider *PID_d;
    QLabel *P_label;
    QFrame *line;
    QLabel *label_6;
    QLabel *ArduinoUpdFrq_label;
    QLabel *label;
    QLabel *label_3;
    QChartView *Pot_view;
    QLabel *Position_label;
    QLabel *label_2;
    QComboBox *statebox;
    QLabel *label_10;
    QLabel *label_5;
    QSlider *PID_i;
    QSlider *PID_p;
    QLabel *label_4;
    QChartView *Position_view;
    QLabel *I_label;
    QComboBox *Position_selector;
    QLabel *label_7;
    QPushButton *PIDtune_btn;
    QLabel *Energy_label;
    QLabel *Dist_label;
    QLabel *label_9;
    QPushButton *Stop_btn;
    QPushButton *Setup_btn;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(631, 787);
        MainWindow->setAcceptDrops(false);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        gridLayout = new QGridLayout(centralWidget);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        textBrowser = new QTextBrowser(centralWidget);
        textBrowser->setObjectName(QString::fromUtf8("textBrowser"));
        QFont font;
        font.setPointSize(9);
        textBrowser->setFont(font);

        gridLayout->addWidget(textBrowser, 14, 0, 1, 4);

        D_label = new QLabel(centralWidget);
        D_label->setObjectName(QString::fromUtf8("D_label"));

        gridLayout->addWidget(D_label, 4, 2, 1, 1);

        Go_btn = new QPushButton(centralWidget);
        Go_btn->setObjectName(QString::fromUtf8("Go_btn"));
        Go_btn->setStyleSheet(QString::fromUtf8("background: rgb(84, 255, 152);"));

        gridLayout->addWidget(Go_btn, 4, 3, 1, 1);

        PID_d = new QSlider(centralWidget);
        PID_d->setObjectName(QString::fromUtf8("PID_d"));
        PID_d->setMaximum(1000);
        PID_d->setOrientation(Qt::Horizontal);

        gridLayout->addWidget(PID_d, 4, 1, 1, 1);

        P_label = new QLabel(centralWidget);
        P_label->setObjectName(QString::fromUtf8("P_label"));

        gridLayout->addWidget(P_label, 1, 2, 1, 1);

        line = new QFrame(centralWidget);
        line->setObjectName(QString::fromUtf8("line"));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);

        gridLayout->addWidget(line, 5, 0, 1, 5);

        label_6 = new QLabel(centralWidget);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        gridLayout->addWidget(label_6, 6, 3, 1, 1);

        ArduinoUpdFrq_label = new QLabel(centralWidget);
        ArduinoUpdFrq_label->setObjectName(QString::fromUtf8("ArduinoUpdFrq_label"));

        gridLayout->addWidget(ArduinoUpdFrq_label, 13, 1, 1, 1);

        label = new QLabel(centralWidget);
        label->setObjectName(QString::fromUtf8("label"));

        gridLayout->addWidget(label, 1, 3, 1, 1);

        label_3 = new QLabel(centralWidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        gridLayout->addWidget(label_3, 4, 0, 1, 1);

        Pot_view = new QChartView(centralWidget);
        Pot_view->setObjectName(QString::fromUtf8("Pot_view"));
        Pot_view->setStyleSheet(QString::fromUtf8("color: rgb(255, 0, 0);"));

        gridLayout->addWidget(Pot_view, 12, 0, 1, 2);

        Position_label = new QLabel(centralWidget);
        Position_label->setObjectName(QString::fromUtf8("Position_label"));

        gridLayout->addWidget(Position_label, 7, 4, 1, 1);

        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        gridLayout->addWidget(label_2, 7, 3, 1, 1);

        statebox = new QComboBox(centralWidget);
        statebox->addItem(QString());
        statebox->addItem(QString());
        statebox->addItem(QString());
        statebox->addItem(QString());
        statebox->addItem(QString());
        statebox->addItem(QString());
        statebox->addItem(QString());
        statebox->addItem(QString());
        statebox->addItem(QString());
        statebox->addItem(QString());
        statebox->addItem(QString());
        statebox->addItem(QString());
        statebox->addItem(QString());
        statebox->addItem(QString());
        statebox->setObjectName(QString::fromUtf8("statebox"));
        QSizePolicy sizePolicy(QSizePolicy::Maximum, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(statebox->sizePolicy().hasHeightForWidth());
        statebox->setSizePolicy(sizePolicy);

        gridLayout->addWidget(statebox, 1, 4, 1, 1);

        label_10 = new QLabel(centralWidget);
        label_10->setObjectName(QString::fromUtf8("label_10"));

        gridLayout->addWidget(label_10, 13, 0, 1, 1);

        label_5 = new QLabel(centralWidget);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        gridLayout->addWidget(label_5, 1, 0, 1, 1);

        PID_i = new QSlider(centralWidget);
        PID_i->setObjectName(QString::fromUtf8("PID_i"));
        PID_i->setMaximum(1000);
        PID_i->setOrientation(Qt::Horizontal);

        gridLayout->addWidget(PID_i, 3, 1, 1, 1);

        PID_p = new QSlider(centralWidget);
        PID_p->setObjectName(QString::fromUtf8("PID_p"));
        PID_p->setMaximum(1000);
        PID_p->setSingleStep(1);
        PID_p->setOrientation(Qt::Horizontal);

        gridLayout->addWidget(PID_p, 1, 1, 1, 1);

        label_4 = new QLabel(centralWidget);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        gridLayout->addWidget(label_4, 3, 0, 1, 1);

        Position_view = new QChartView(centralWidget);
        Position_view->setObjectName(QString::fromUtf8("Position_view"));

        gridLayout->addWidget(Position_view, 10, 0, 1, 2);

        I_label = new QLabel(centralWidget);
        I_label->setObjectName(QString::fromUtf8("I_label"));

        gridLayout->addWidget(I_label, 3, 2, 1, 1);

        Position_selector = new QComboBox(centralWidget);
        Position_selector->addItem(QString());
        Position_selector->addItem(QString());
        Position_selector->addItem(QString());
        Position_selector->setObjectName(QString::fromUtf8("Position_selector"));

        gridLayout->addWidget(Position_selector, 6, 0, 1, 1);

        label_7 = new QLabel(centralWidget);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        gridLayout->addWidget(label_7, 12, 3, 1, 1);

        PIDtune_btn = new QPushButton(centralWidget);
        PIDtune_btn->setObjectName(QString::fromUtf8("PIDtune_btn"));

        gridLayout->addWidget(PIDtune_btn, 3, 3, 1, 1);

        Energy_label = new QLabel(centralWidget);
        Energy_label->setObjectName(QString::fromUtf8("Energy_label"));

        gridLayout->addWidget(Energy_label, 12, 4, 1, 1);

        Dist_label = new QLabel(centralWidget);
        Dist_label->setObjectName(QString::fromUtf8("Dist_label"));

        gridLayout->addWidget(Dist_label, 6, 4, 1, 1);

        label_9 = new QLabel(centralWidget);
        label_9->setObjectName(QString::fromUtf8("label_9"));

        gridLayout->addWidget(label_9, 11, 0, 1, 1);

        Stop_btn = new QPushButton(centralWidget);
        Stop_btn->setObjectName(QString::fromUtf8("Stop_btn"));
        Stop_btn->setStyleSheet(QString::fromUtf8("background: rgb(255, 79, 105);\n"
""));

        gridLayout->addWidget(Stop_btn, 4, 4, 1, 1);

        Setup_btn = new QPushButton(centralWidget);
        Setup_btn->setObjectName(QString::fromUtf8("Setup_btn"));

        gridLayout->addWidget(Setup_btn, 3, 4, 1, 1);

        MainWindow->setCentralWidget(centralWidget);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "Centre de controle", nullptr));
        D_label->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        Go_btn->setText(QCoreApplication::translate("MainWindow", "Goooo", nullptr));
        P_label->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        label_6->setText(QCoreApplication::translate("MainWindow", "Distance parcourrue", nullptr));
        ArduinoUpdFrq_label->setText(QCoreApplication::translate("MainWindow", "??? Hz", nullptr));
        label->setText(QCoreApplication::translate("MainWindow", "Etat arduino", nullptr));
        label_3->setText(QCoreApplication::translate("MainWindow", "De-1", nullptr));
        Position_label->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        label_2->setText(QCoreApplication::translate("MainWindow", "Position", nullptr));
        statebox->setItemText(0, QCoreApplication::translate("MainWindow", "Ready", nullptr));
        statebox->setItemText(1, QCoreApplication::translate("MainWindow", "Stabilize", nullptr));
        statebox->setItemText(2, QCoreApplication::translate("MainWindow", "ReturnHome", nullptr));
        statebox->setItemText(3, QCoreApplication::translate("MainWindow", "GetToCalibratePos", nullptr));
        statebox->setItemText(4, QCoreApplication::translate("MainWindow", "Calibrate", nullptr));
        statebox->setItemText(5, QCoreApplication::translate("MainWindow", "TakingTree", nullptr));
        statebox->setItemText(6, QCoreApplication::translate("MainWindow", "Swinging", nullptr));
        statebox->setItemText(7, QCoreApplication::translate("MainWindow", "JustGonnaSendIt", nullptr));
        statebox->setItemText(8, QCoreApplication::translate("MainWindow", "JustGonnaSmoothIt", nullptr));
        statebox->setItemText(9, QCoreApplication::translate("MainWindow", "BuildUp", nullptr));
        statebox->setItemText(10, QCoreApplication::translate("MainWindow", "Drop", nullptr));
        statebox->setItemText(11, QCoreApplication::translate("MainWindow", "ShortCircuitForward", nullptr));
        statebox->setItemText(12, QCoreApplication::translate("MainWindow", "ShortCircuitBackward", nullptr));
        statebox->setItemText(13, QCoreApplication::translate("MainWindow", "Error", nullptr));

        label_10->setText(QCoreApplication::translate("MainWindow", "Arduino received", nullptr));
        label_5->setText(QCoreApplication::translate("MainWindow", "Pe-1", nullptr));
        label_4->setText(QCoreApplication::translate("MainWindow", "Ie-1", nullptr));
        I_label->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        Position_selector->setItemText(0, QCoreApplication::translate("MainWindow", "Position", nullptr));
        Position_selector->setItemText(1, QCoreApplication::translate("MainWindow", "Vitesse", nullptr));
        Position_selector->setItemText(2, QCoreApplication::translate("MainWindow", "Acceleration", nullptr));

        label_7->setText(QCoreApplication::translate("MainWindow", "Energie consommee", nullptr));
        PIDtune_btn->setText(QCoreApplication::translate("MainWindow", "PidTune", nullptr));
        Energy_label->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        Dist_label->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        label_9->setText(QCoreApplication::translate("MainWindow", "Angle pendule", nullptr));
        Stop_btn->setText(QCoreApplication::translate("MainWindow", "Stop", nullptr));
        Setup_btn->setText(QCoreApplication::translate("MainWindow", "Setup", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
