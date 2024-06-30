#include "mainwindow.hpp"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w("/dev/ttyACM0", 100);
    w.show();
    return a.exec();

}
