# include <Arduino.h>
# include <string.h>
# include <LibS3GRO.h>

# define POTPIN 7

// Objects
MegaServo servo;
ArduinoX AX;
// IMU9DOF imu;


double accelx;

void setup() 
{
  Serial.begin(115200);
  // init ArduinoX
  AX.init();

  // Define Servo pin
  servo.attach(2);

  // imu.init();
  
}

void loop() 
{ 
  
  for(int i = 0; i<180; i++)
  {
    double potval = analogRead(POTPIN);
    servo.write((potval/1024)*180);
    AX.setMotorPWM(0,(potval-512)/512);

    delay(25);

    // Various prints
    Serial.print(270*potval/1024);
    Serial.print("    ");
    Serial.print(AX.readResetEncoder(0));
    
    /* accelx = imu.getAccelX();
    Serial.print("    ");
    Serial.print(imu.isConnected());
    Serial.print("    ");
    Serial.println(accelx); */
  }
  
}

