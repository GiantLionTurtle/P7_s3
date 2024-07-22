#include "plot.h"

// Functions

Plot::Plot(double gradX_, double gradY_){
    gradX = gradX_;
    gradY = gradY_;
}

void Plot::setColor(int r, int g, int b){
    QColor color;
    color.setAlpha(255);
    color.setRed(r);
    color.setGreen(g);
    color.setBlue(b);
    pen.setColor(color);
}

void Plot::setDataLen(int dataLen){
    dataBufferLen = dataLen;
}

void Plot::addData(double newData){

    if (data.length()>dataBufferLen){
        data.pop_front();
    }
    data.append(newData);
}
void Plot::clear()
{
  data.clear();
}
void Plot::draw(QGraphicsScene* scene){
    if(data.length() == 0)
        return;

    double min = data[0];
    double max = data[0];

    for(int i = 1; i < data.length(); i++) {
        if(data[i] > max) {
            max = data[i];
        }
        if(data[i] < min) {
            min = data[i];
        }
    }
    min *= -gain;
    max *= -gain;

    QPen(Qt::red);

    for (int y = 0; y >= max; y -= gradY*gain) {
        scene->addLine(0, y, 500, y, QPen(Qt::black));
    }
    for (int y = min; y >= 0; y -= gradY*gain) {
        scene->addLine(0, y, 500, y, QPen(Qt::black));
    }

    QPainterPath curve;
    curve.moveTo(0,-gain*data[0]);
    for (int i = 0; i < data.length(); ++i){
        curve.lineTo(i,-gain*data[i]);
    }
    scene->addPath(curve,pen);
}

void Plot::setGain(double gain_){
    gain = gain_;
}
