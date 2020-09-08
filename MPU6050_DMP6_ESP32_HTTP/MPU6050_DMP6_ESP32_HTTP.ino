#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#include <WiFi.h>
#include <WiFiClient.h>
#include <ArduinoJson.h> //Json 사용을 위한 라이브러리 
#include <HTTPClient.h>  //HTTP 전송을 위한 라이브러리

char ssid[] = "KT_GiGA_2G_Wave2_1123";          // your network SSID (name)
char pass[] = "ke77ff4984";                     // your network password

MPU6050 mpu;
WiFiClient client;
HTTPClient http;    

// 센서데이터에서 읽어오는 종류를 정할 수 있다
// 현재 목적은 YPR 값이기때문에 OUTPUT_READBLE_YAWPITCHROLL 주석 해제해서 사용하는데 원하는 데이터에 맞게 주석을 풀어서 사용 

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT

//#define OUTPUT_TEAPOT_OSC << OSC 

#define INTERRUPT_PIN 15  // MPU6050 - ESP32 INTERRUPT_PIN 번호 

// MPU control/status vars  //센서 제어 및 상태들을 저장한 변수
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[1024]; // FIFO storage buffer , 기존 코드는 64 인데 FIFO overflow 방지를 위해 늘림

//orientation/motion vars  //ㅊ
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// ===               인터럽트 발생유무 확인용                ===

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ===               setup 시작                ===+

void setup() {
  
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
       //I2C 셋팅 및 시작
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);  
    
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize(); //MPU센서 초기화
    pinMode(INTERRUPT_PIN, INPUT); //인터럽트핀(15) 입력으로 설정

    // 연결확인
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    
  /////////////////////////////////wifi 연결부분/////////////////////////////////////
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, pass);
  Serial.print(F("WiFi connected! IP address: "));
  Serial.println(WiFi.localIP());
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  ///////////////////////////////////////////////////////////////////////////////////
  
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // 초기 감도 셋팅
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true); //DMP 활성화

        // 인터럽트 핀과 함수를 연결, 인터럽트 PIN이 LOW -> HIGH로 올라갈 때 dmpDataReady 함수 호출
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));//INTERRUPT_PIN
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = 초기화에러 
        // 2 = DMP 업데이트 에러
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}
// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

    if (!dmpReady) return; //초기화 실패 시 함수 종료 
    // wait for MPU interrupt or extra packet(s) available
    
    //mpuInterrupt 변수 기다리다가 인터럽트가 발생하면 다음으로 넘어감
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
          // try to get out of the infinite loop 
          fifoCount = mpu.getFIFOCount();
        }  
    }
    // reset interrupt flag and get INT_STATUS byte
    //인터럽트 변수 초기화
    mpuInterrupt = false;
    //mpu6050 상태 읽기
    mpuIntStatus = mpu.getIntStatus();

    // FIFO 버퍼 개수 얻기
   fifoCount = mpu.getFIFOCount();

    //=========== 대기 -> 초기화 -> 데이터 받아오기 ==========
    if ((mpuIntStatus & (0x01 << MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 16384) {
        mpu.resetFIFO(); //버퍼초기화
        fifoCount = mpu.getFIFOCount();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & (0x01 << MPU6050_INTERRUPT_DMP_INT_BIT)) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // FIFO 에서 데이터 받아오기
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
  
    #ifdef OUTPUT_READABLE_YAWPITCHROLL       //YPR 값을 얻어서 시리얼에 출력하는 기능
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
    #endif
   }
         if(WiFi.status() == WL_CONNECTED){   //값을 JSON 형식으로 가공하는 부분
                   //값을 JSON 형식으로 변환 
             StaticJsonDocument<100> doc;  
             JsonObject root = doc.to<JsonObject>();
             root["Type"] = "MPU6050";
             JsonObject value = root.createNestedObject("value");      
             value["Yaw"] = ypr[0] * 180/M_PI ;
             value["Pitch"] = ypr[1] * 180/M_PI;
             value["Roll"] = ypr[2] * 180/M_PI;

             // JSON 로 가공한 데이터를 HTTP 로 전송 
             String requestBody;
             serializeJson(root, requestBody);
             Serial.println(requestBody);
             postDataToServer(requestBody);   //가공한 데이터를 POST형식으로 서버에 전송하는 함수 호출
           }
 
}
//        #ifdef OUTPUT_READABLE_QUATERNION
//            // display quaternion values in easy matrix form: w x y z
//            mpu.dmpGetQuaternion(&q, fifoBuffer);
//            Serial.print("quat\t");
//            Serial.print(q.w);
//            Serial.print("\t");
//            Serial.print(q.x);
//            Serial.print("\t");
//            Serial.print(q.y);
//            Serial.print("\t");
//            Serial.println(q.z);
//        #endif
//
//        #ifdef OUTPUT_READABLE_EULER
//            // display Euler angles in degrees
//            mpu.dmpGetQuaternion(&q, fifoBuffer);
//            mpu.dmpGetEuler(euler, &q);
//            Serial.print("euler\t");
//            Serial.print(euler[0] * 180/M_PI);
//            Serial.print("\t");
//            Serial.print(euler[1] * 180/M_PI);
//            Serial.print("\t");
//            Serial.println(euler[2] * 180/M_PI);
//        #endif

//        #ifdef OUTPUT_READABLE_REALACCEL
//            // display real acceleration, adjusted to remove gravity
//            mpu.dmpGetQuaternion(&q, fifoBuffer);
//            mpu.dmpGetAccel(&aa, fifoBuffer);
//            mpu.dmpGetGravity(&gravity, &q);
//            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//            Serial.print("areal\t");
//            Serial.print(aaReal.x);
//            Serial.print("\t");
//            Serial.print(aaReal.y);
//            Serial.print("\t");
//            Serial.println(aaReal.z);
//        #endif
//
//        #ifdef OUTPUT_READABLE_WORLDACCEL
//            // display initial world-frame acceleration, adjusted to remove gravity
//            // and rotated based on known orientation from quaternion
//            mpu.dmpGetQuaternion(&q, fifoBuffer);
//            mpu.dmpGetAccel(&aa, fifoBuffer);
//            mpu.dmpGetGravity(&gravity, &q);
//            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
//            Serial.print("aworld\t");
//            Serial.print(aaWorld.x);
//            Serial.print("\t");
//            Serial.print(aaWorld.y);
//            Serial.print("\t");
//            Serial.println(aaWorld.z);
//        #endif
    

//데이터 post 형식 전송 함수 
     void postDataToServer(String requestBody){
            Serial.println("Posting JSON data to server...");
            http.begin("http://172.30.1.47:3000/senser_data");  // 수정 -> 서버 주소:포트/(경로)
            //http.begin("http://ec2-18-237-47-239.us-west-2.compute.amazonaws.com:9090/senser_data");  // 사용X 
            http.addHeader("Content-Type", "application/json; charset = utf-8");
            int httpResponseCode = http.POST(requestBody);  //POST형식으로 
 
      if(httpResponseCode>0){ 
      String response = http.getString();                    
      Serial.println(response);
    }
    else { 
      Serial.printf("Error");     
    }
   }
