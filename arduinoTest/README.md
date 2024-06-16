## What's this?
This is a bunch of code to test the inputs/outputs associated with the motors / sensors.

## How do I get stuff to work?
### Motor
> AX.setMotorPWM(motorID, value);

where motorID is probably 0, and value \[-1, 1\]

### Motor encoder
> int value = AX.readResetEncoder(motorID);

where motorID is probably 0

Note: there are 
- 3200 CPR on the Polulu 50:1
- 1200 CPR on the Polulu 19:1
-> (CPR (encoder count per revolution))

### ServoMotor
> servo.write(angle);

where angle is \[0, 180\], in degrees

Note: You need to attach a pin to your servo object with servo.attach(SERVOPIN)

### Potentiometer
> double value = analogRead(POTPIN);

where value will be \[0, 1024\]

To get a value in degrees:
> double value = 180* (analogRead(POTPIN)/1024) - zeroOffsetValue;

### IMU
Doesn't work, cry harder.

### Libraries
You'll need those:
- Arduino.h
- LibS3GRO.h
- Adafruit BusIO (by adafruit, from PlatIO library index)
- MPU6050 (by electroniccats, from PlatIO library index)
- SoftwareSerial (by featherfly, from PlatIO library index)


## How do I plug stuff in?
### Servo
- Black -> GND
- Red -> 5V
- Yellow -> pin 2 (Arduino pins), in the PWM section

### Potentiometer
- Black -> GND
- Red -> 5V
- White -> pin A7 (Arduino pins), in the Analog In section

Note: The Vcc pin on the analog input Grove connectors has weird voltage, don't use them.

### The rest
Why did you unplug it

## I just want to copy and paste the requirements...
### Top of code
> \# define POTPIN 7
> 
> \# define SERVOPIN 2

### Objects (below your \# define stuff)
> MegaServo servo;
> 
> ArduinoX AX;

### In setup() 
> AX.init();
> 
> servo.attach(SERVOPIN);

