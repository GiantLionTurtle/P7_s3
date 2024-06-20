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
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QSlider>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QGridLayout *gridLayout;
    QFrame *line;
    QSlider *PID_i;
    QLabel *label_4;
    QLabel *label_5;
    QSlider *PID_p;
    QLabel *label;
    QTextBrowser *textBrowser;
    QComboBox *statebox;
    QLabel *label_3;
    QSlider *PID_d;
    QGraphicsView *graphicsView;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(738, 649);
        MainWindow->setAcceptDrops(false);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        gridLayout = new QGridLayout(centralWidget);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        line = new QFrame(centralWidget);
        line->setObjectName(QString::fromUtf8("line"));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);

        gridLayout->addWidget(line, 4, 0, 1, 4);

        PID_i = new QSlider(centralWidget);
        PID_i->setObjectName(QString::fromUtf8("PID_i"));
        PID_i->setMaximum(100);
        PID_i->setOrientation(Qt::Horizontal);

        gridLayout->addWidget(PID_i, 2, 1, 1, 1);

        label_4 = new QLabel(centralWidget);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        gridLayout->addWidget(label_4, 2, 0, 1, 1);

        label_5 = new QLabel(centralWidget);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        gridLayout->addWidget(label_5, 1, 0, 1, 1);

        PID_p = new QSlider(centralWidget);
        PID_p->setObjectName(QString::fromUtf8("PID_p"));
        PID_p->setMaximum(200);
        PID_p->setSingleStep(1);
        PID_p->setOrientation(Qt::Horizontal);

        gridLayout->addWidget(PID_p, 1, 1, 1, 1);

        label = new QLabel(centralWidget);
        label->setObjectName(QString::fromUtf8("label"));

        gridLayout->addWidget(label, 1, 2, 1, 1);

        textBrowser = new QTextBrowser(centralWidget);
        textBrowser->setObjectName(QString::fromUtf8("textBrowser"));
        QFont font;
        font.setPointSize(9);
        textBrowser->setFont(font);

        gridLayout->addWidget(textBrowser, 10, 0, 1, 4);

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
        statebox->setObjectName(QString::fromUtf8("statebox"));
        QSizePolicy sizePolicy(QSizePolicy::Maximum, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(statebox->sizePolicy().hasHeightForWidth());
        statebox->setSizePolicy(sizePolicy);

        gridLayout->addWidget(statebox, 1, 3, 1, 1);

        label_3 = new QLabel(centralWidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        gridLayout->addWidget(label_3, 3, 0, 1, 1);

        PID_d = new QSlider(centralWidget);
        PID_d->setObjectName(QString::fromUtf8("PID_d"));
        PID_d->setMaximum(100);
        PID_d->setOrientation(Qt::Horizontal);

        gridLayout->addWidget(PID_d, 3, 1, 1, 1);

        graphicsView = new QGraphicsView(centralWidget);
        graphicsView->setObjectName(QString::fromUtf8("graphicsView"));
        graphicsView->setEnabled(true);

        gridLayout->addWidget(graphicsView, 9, 0, 1, 4);

        MainWindow->setCentralWidget(centralWidget);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "Centre de controle", nullptr));
        label_4->setText(QCoreApplication::translate("MainWindow", "Ie-1", nullptr));
        label_5->setText(QCoreApplication::translate("MainWindow", "Pe-1", nullptr));
        label->setText(QCoreApplication::translate("MainWindow", "Etat arduino", nullptr));
        statebox->setItemText(0, QCoreApplication::translate("MainWindow", "Ready", nullptr));
        statebox->setItemText(1, QCoreApplication::translate("MainWindow", "Stabilize", nullptr));
        statebox->setItemText(2, QCoreApplication::translate("MainWindow", "ReturnHome", nullptr));
        statebox->setItemText(3, QCoreApplication::translate("MainWindow", "TakingTree", nullptr));
        statebox->setItemText(4, QCoreApplication::translate("MainWindow", "Swinging", nullptr));
        statebox->setItemText(5, QCoreApplication::translate("MainWindow", "JustGonnaSendIt", nullptr));
        statebox->setItemText(6, QCoreApplication::translate("MainWindow", "Drop", nullptr));
        statebox->setItemText(7, QCoreApplication::translate("MainWindow", "ShortCircuitForward", nullptr));
        statebox->setItemText(8, QCoreApplication::translate("MainWindow", "ShorCircuitBackward", nullptr));

        label_3->setText(QCoreApplication::translate("MainWindow", "De-1", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
