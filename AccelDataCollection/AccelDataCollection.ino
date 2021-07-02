
#include <Arduino.h>
#include <Arduino_LSM9DS1.h>




float kTargetHz = 31.25;


float x[75];
float y[75];
float z[75];

float y_pred;
int arrayIndex = 0;



int skipCounter = 1;
boolean dataFull = false;
int sampleEveryN;





boolean fell;


void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);











  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU");
  }



  IMU.setAccelFS(3);
  IMU.setAccelODR(3);
  IMU.setContinuousMode();
  sampleEveryN = static_cast<int>(roundf(IMU.getAccelODR() / kTargetHz));


}

// the loop routine runs over and over again forever:
void loop() {




  while (IMU.accelerationAvailable()) {
    float aX, aY, aZ;

    IMU.readAcceleration(aZ, aY, aX);


    if (skipCounter != sampleEveryN) {
      skipCounter++;
      continue;
    }


    x[arrayIndex] = aX;
    y[arrayIndex] = aY;
    z[arrayIndex] = aZ;



    skipCounter = 1;
    arrayIndex++;
    if (arrayIndex > 74) {
      dataFull = true;

      break;
    }

  }







  if (dataFull) {

    for (int i = 0; i < 75; i++) {

      Serial.print(String(x[i]) + ",");
      Serial.print(String(y[i]) + ",");
      Serial.print(String(z[i]) + ",");
    }

    arrayIndex = 0;
    dataFull = false;


    Serial.println("");




  }





}
