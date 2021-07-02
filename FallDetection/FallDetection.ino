
#include "TensorFlowLite.h"

#include "fallmodel.h"

#include "tensorflow/lite/micro/kernels/micro_ops.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/testing/micro_test.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"


#include <Arduino.h>
#include <Arduino_LSM9DS1.h>
#include <ArduinoBLE.h>


BLEService fallService("5321");
BLEBoolCharacteristic fallDetectionBool("0001", BLERead | BLENotify);


tflite::MicroErrorReporter tflErrorReporter;

const tflite::Model* tflModel = nullptr;
tflite::MicroInterpreter* tflInterpreter = nullptr;
TfLiteTensor* tflInputTensor = nullptr;
TfLiteTensor* tflOutputTensor = nullptr;

const int tensorArenaSize = 32 * 1024;
uint8_t tensorArena[tensorArenaSize];


float y_pred;
int arrayIndex = 0;
float kTargetHz = 31.25;
int skipCounter = 1;
int sampleEveryN;
boolean dataFull = false;


const int buttonPin = 12;
const int notificationPin = 8;

boolean buttonState;
boolean currentButtonState;
boolean lastButtonState;
boolean buttonPressed;

unsigned long lastDebounceTime;
unsigned long lastFallTime;

boolean fell;


void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  delay(7000);

  tflModel = tflite::GetModel(model);
  if (tflModel->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("Model schema mismatch!");
    while (1);
  }

  static tflite::MicroMutableOpResolver<5> resolver;  // NOLINT
  resolver.AddMaxPool2D();
  resolver.AddConv2D();
  resolver.AddFullyConnected();
  resolver.AddSoftmax();
  resolver.AddReshape();




  tflInterpreter = new tflite::MicroInterpreter(tflModel, resolver, tensorArena, tensorArenaSize, &tflErrorReporter);

  // Allocate memory for the model's input and output tensors
  TfLiteStatus allocate_status = tflInterpreter->AllocateTensors();

  if (allocate_status != kTfLiteOk) {
    Serial.println("AllocateTensors() failed");
    return;
  }

  // Get pointers for the model's input and output tensors
  tflInputTensor = tflInterpreter->input(0);
  tflOutputTensor = tflInterpreter->output(0);


  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU");
  }

  if (!BLE.begin())
  {
    Serial.println("starting BLE failed!");
    while (1);
  }

  //set up accelerometer range and frequency
  IMU.setAccelFS(3);
  IMU.setAccelODR(3);
  IMU.setContinuousMode();

  sampleEveryN = static_cast<int>(roundf(IMU.getAccelODR() / kTargetHz));


  //setting up BLE
  String BLEAddress = BLE.address();
  Serial.println ("Address: " + BLEAddress);

  BLE.setDeviceName("Fall Detector");

  BLE.setAppearance(1088);

  BLE.setLocalName("BLE Fall Detection");

  BLE.setAdvertisedService(fallService);

  fallService.addCharacteristic(fallDetectionBool);
  BLE.addService(fallService);
  fallDetectionBool.writeValue(false);

  BLE.advertise();

  Serial.println("Looking for connections");

  BLE.setEventHandler(BLEConnected, bleConnectHandler);

  BLE.setEventHandler(BLEDisconnected, bleDisconnectHandler);


  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(notificationPin, OUTPUT);

  Serial.println("Initialized");
}


void loop() {

  BLEDevice central = BLE.central();


  while (IMU.accelerationAvailable()) {
    float aX, aY, aZ;

    IMU.readAcceleration(aZ, aY, aX);

    //skipping to match the frequency of the data used to train the model
    if (skipCounter != sampleEveryN) {
      skipCounter++;
      continue;
    }
    
    tflInputTensor->data.f[3 * arrayIndex] = aX/10;
    tflInputTensor->data.f[3 * arrayIndex + 1] = aY/10;
    tflInputTensor->data.f[3 * arrayIndex + 2] = aZ/10;
    //x[arrayIndex] = aX;
    //y[arrayIndex] = aY;
    //z[arrayIndex] = aZ;

    skipCounter = 1;
    arrayIndex++;
    if (arrayIndex > 74) {
      dataFull = true;
      break;
    }

  }

  currentButtonState = !digitalRead(buttonPin); //read state of button

  if (currentButtonState != lastButtonState) {
    lastDebounceTime = millis();
  }

  if (millis() - lastDebounceTime > 50) {
    if (currentButtonState != buttonState) {
      buttonState = currentButtonState;
      if (buttonState == LOW) {
        buttonPressed = true;
        Serial.println("button pressed");
      }
    }
  }
  lastButtonState = currentButtonState;

  if (dataFull) {


    // Serial.println(tflInputTensor->data.f[0]);

    TfLiteStatus invokeStatus = tflInterpreter->Invoke(); //run the model

    arrayIndex = 0;
    dataFull = false;

    if (invokeStatus != kTfLiteOk) {
      Serial.println("invoke failed");
    }

    Serial.println("");


    y_pred = tflOutputTensor->data.f[1]; //model's prediction on accel data
    Serial.println(y_pred);

    if (y_pred > 0.7) {

      //fall has been detected by the model

      Serial.println("Fall");

      lastFallTime = millis();
      fell = true;
      digitalWrite(notificationPin, HIGH);
    }
  }

  if (fell) {

    //options after a fall is dected
    if (millis() - lastFallTime > 30 * 1000) {
      //alerting phone app as no button press for 30 seconds after fall detected
      if (central) {
        fallDetectionBool.writeValue(true);
      }

      digitalWrite(notificationPin, LOW);
      fell = false;
    }
    else if (buttonPressed) {
      //button is pressed during initial time period and fall alert is deactivated


      digitalWrite(notificationPin, LOW);
      if (central) {
        fallDetectionBool.writeValue(false);
      }
      fell = false;
    }
    else {
      //initial 30 second time period when buzzer is on
      digitalWrite(notificationPin, HIGH);

    }
  }
  else if (buttonPressed) {
    //if the button is pressed when a fall is not detected activate the fall timer
    lastFallTime = millis();
    fell = true;
    digitalWrite(notificationPin, HIGH);

  }



  buttonPressed = false;
}


void bleConnectHandler(BLEDevice central) {
  Serial.println("connect");
}

void bleDisconnectHandler(BLEDevice central) {
  Serial.println("disconnect");
}
