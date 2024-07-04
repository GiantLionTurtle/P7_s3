#ifndef POTENTIOMETRE_HPP_
#define POTENTIOMETRE_HPP_

#include <LibS3GRO.h>

class Potentiometre
{
    public:
        POTENTIOMETRE(uint8_t pin_ch1, uint8_t pin_ch2){potentiometre.init(pin_ch1, pin_ch2);};
        double getAngle(){return rotationRange*(potentiometre.analogRead()-zeroValue)/1023;};
        void calibrate(){zeroValue = potentiometre.analogRead()};
    private:
        double zeroValue;
        const rotationRange = 265; //ou 250
        VexQuadEncoder potentiometre;
};

#endif