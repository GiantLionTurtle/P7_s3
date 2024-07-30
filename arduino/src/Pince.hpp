#define PINCE_HPP
#define OUVERTE 90
#define FERMEE 0
#include <Arduino.h>
#include <LibS3GRO.h>

class Pince
{
    private :
        bool open = true;
        MegaServo megaservo_;
    public:
        void Init(int pin){megaservo_.attach(pin)};
        void Open(){megaservo_.write(FERMEE); open = true};
        void close(){megaservo_.write(OUVERTE);open = false;};
        bool isOpen(){return open};
};

