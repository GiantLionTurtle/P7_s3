
#include "mainwindow.hpp"
#include <QApplication>
#include "../../common_rpiarduino/Common.hpp"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w("/dev/ttyACM0", UPDATE_RATE_MS);
    w.show();
    return a.exec();

}
