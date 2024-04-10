#include <REG.h>
#include <wit_c_sdk.h>
#include <PS2X_lib.h>
#include <Thread.h>

#define ACC_UPDATE 0x01
#define GYRO_UPDATE 0x02
#define ANGLE_UPDATE 0x04
#define MAG_UPDATE 0x08
#define READ_UPDATE 0x80

static volatile char s_cDataUpdate = 0, s_cCmd = 0xff;
static Thread thread1, thread2;

static void CmdProcess(void);
static void AutoScanSensor(void);
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);
const uint32_t c_uiBaud[8] = { 0, 4800, 9600, 19200, 38400, 57600, 115200, 230400 };

int i;
float fAcc[3], fGyro[3], fAngle[3];

float init_x = 0;
float init_y = 0;
float xAngle = 0;
float yAngle = 0;
#define MAX_TILT 60

#define s1 9  // Motor 1 speed
#define s2 6  // Motor 2 speed
#define d1 8  // Motor 1 direction
#define d2 7  // Motor 2 direction

#define E_STOP 2

PS2X ps2x;

int error = 0;
byte type = 0;
byte vibrate = 0;

int speedVal = 40;

unsigned long lastButtonPressTime = 0;  // Variable to store the time of the last button press
unsigned long debounceDelay = 50;       // Adjust this delay as needed

bool e_stop = false;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(d1, OUTPUT);
  pinMode(d2, OUTPUT);
  analogWrite(s1, 0);
  analogWrite(s2, 0);
  digitalWrite(d1, 0);
  digitalWrite(d2, 0);

  pinMode(E_STOP, OUTPUT);
  digitalWrite(E_STOP, HIGH);


  WitInit(WIT_PROTOCOL_NORMAL, 0x50);
  WitSerialWriteRegister(SensorUartSend);
  WitRegisterCallBack(SensorDataUpdata);
  WitDelayMsRegister(Delayms);
  AutoScanSensor();

  error = ps2x.config_gamepad(13, 11, 10, 12, false, false);  //(clock, command, attention, data)


  if (error == 0) {
    Serial.println("Found Controller, configured successful");
  } else if (error == 1) {
    Serial.println("No controller found, check wiring or reset the Arduino");
  } else if (error == 2) {
    Serial.println("Controller found but not accepting commands");
  } else if (error == 3) {
    Serial.println("Controller refusing to enter Pressures mode, may not support it.");
  }

  type = ps2x.readType();
  switch (type) {
    case 0:
      break;
    case 1:
      break;
    case 2:
      break;
  }

  if (Serial1.available()) {
    WitSerialDataIn(Serial1.read());
  }
  if (Serial.available()) {
    CopeCmdData(Serial.read());
  }
  CmdProcess();
  if (s_cDataUpdate) {
    for (i = 0; i < 3; i++) {
      fAcc[i] = sReg[AX + i] / 32768.0f * 16.0f;
      fGyro[i] = sReg[GX + i] / 32768.0f * 2000.0f;
      fAngle[i] = sReg[Roll + i] / 32768.0f * 180.0f;
    }

    if (s_cDataUpdate & ANGLE_UPDATE) {

      init_x = fAngle[0];
      init_y = fAngle[1];

      s_cDataUpdate &= ~ANGLE_UPDATE;
    }

    s_cDataUpdate = 0;
  }

  thread1.onRun([]() {
    while (Serial1.available()) {
      WitSerialDataIn(Serial1.read());
    }
    while (Serial.available()) {
      CopeCmdData(Serial.read());
    }
    CmdProcess();
    if (s_cDataUpdate) {
      for (i = 0; i < 2; i++) {
        fAngle[i] = sReg[Roll + i] / 32768.0f * 180.0f;
      }

      xAngle = fAngle[0] - init_x;
      yAngle = fAngle[1] - init_y;

      if (abs(xAngle) > MAX_TILT || abs(xAngle) > MAX_TILT) {
        e_stop = true;
        Serial.print("[WARNING] Tilting over MAX_TILT ");
        digitalWrite(E_STOP, LOW);
        delay(100);
      } else {
        e_stop = false;
        digitalWrite(E_STOP, HIGH);
      }

      if (s_cDataUpdate & ANGLE_UPDATE) {
        Serial.print("X:");
        Serial.print(xAngle, 2);
        Serial.print(" Y: ");
        Serial.print(yAngle, 2);
        Serial.print("\r\n");
        s_cDataUpdate &= ~ANGLE_UPDATE;
      }

      s_cDataUpdate = 0;
    }
  });

  thread2.onRun([]() {
    ps2x.read_gamepad(false, vibrate);

    int rightstickry = ps2x.Analog(PSS_RY);
    int rightstickrx = ps2x.Analog(PSS_RX);

    unsigned long currentMillis = millis();

    if (!e_stop) {

      vibrate = 0;
      if (currentMillis - lastButtonPressTime >= debounceDelay) {
        if (ps2x.Button(PSB_PAD_UP)) {
          Serial.println("FORWARD");
          digitalWrite(d1, 1);
          digitalWrite(d2, 1);
          analogWrite(s1, speedVal);
          analogWrite(s2, speedVal);
          lastButtonPressTime = currentMillis;
        } else if (ps2x.Button(PSB_PAD_RIGHT)) {
          Serial.println("RIGHT");
          digitalWrite(d1, 1);
          digitalWrite(d2, 1);
          analogWrite(s1, speedVal);
          analogWrite(s2, 20);
          lastButtonPressTime = currentMillis;
        } else if (ps2x.Button(PSB_PAD_LEFT)) {
          Serial.println("LEFT");
          digitalWrite(d1, 1);
          digitalWrite(d2, 1);
          analogWrite(s1, 20);
          analogWrite(s2, speedVal);
          lastButtonPressTime = currentMillis;
        } else if (ps2x.Button(PSB_PAD_DOWN)) {
          Serial.println("BACK");
          digitalWrite(d1, 0);
          digitalWrite(d2, 0);
          analogWrite(s1, speedVal);
          analogWrite(s2, speedVal);
          lastButtonPressTime = currentMillis;
        }
      }
    }


    delay(50);
  });



  thread1.setInterval(50);
  thread2.setInterval(50);
  thread1.enabled = true;
  thread2.enabled = true;
}

void loop() {
  thread1.run();
  thread2.run();
}

void CopeCmdData(unsigned char ucData) {
  static unsigned char s_ucData[50], s_ucRxCnt = 0;

  s_ucData[s_ucRxCnt++] = ucData;
  if (s_ucRxCnt < 3)
    return;  //Less than three data returned
  if (s_ucRxCnt >= 50)
    s_ucRxCnt = 0;
  if (s_ucRxCnt >= 3) {
    if ((s_ucData[1] == '\r') && (s_ucData[2] == '\n')) {
      s_cCmd = s_ucData[0];
      memset(s_ucData, 0, 50);
      s_ucRxCnt = 0;
    } else {
      s_ucData[0] = s_ucData[1];
      s_ucData[1] = s_ucData[2];
      s_ucRxCnt = 2;
    }
  }
}

static void ShowHelp(void) {
  Serial.print("\r\n************************   WIT_SDK_DEMO ************************");
  Serial.print("\r\n************************          HELP           ************************\r\n");
  Serial.print("UART SEND:a\\r\\n   Acceleration calibration.\r\n");
  Serial.print("UART SEND:m\\r\\n   Magnetic field calibration,After calibration send:   e\\r\\n   to indicate the end\r\n");
  Serial.print("UART SEND:U\\r\\n   Bandwidth increase.\r\n");
  Serial.print("UART SEND:u\\r\\n   Bandwidth reduction.\r\n");
  Serial.print("UART SEND:B\\r\\n   Baud rate increased to 115200.\r\n");
  Serial.print("UART SEND:b\\r\\n   Baud rate reduction to 9600.\r\n");
  Serial.print("UART SEND:R\\r\\n   The return rate increases to 10Hz.\r\n");
  Serial.print("UART SEND:r\\r\\n   The return rate reduction to 1Hz.\r\n");
  Serial.print("UART SEND:C\\r\\n   Basic return content: acceleration, angular velocity, angle, magnetic field.\r\n");
  Serial.print("UART SEND:c\\r\\n   Return content: acceleration.\r\n");
  Serial.print("UART SEND:h\\r\\n   help.\r\n");
  Serial.print("******************************************************************************\r\n");
}

static void CmdProcess(void) {
  switch (s_cCmd) {
    case 'a':
      if (WitStartAccCali() != WIT_HAL_OK)
        Serial.print("\r\nSet AccCali Error\r\n");
      break;
    case 'm':
      if (WitStartMagCali() != WIT_HAL_OK)
        Serial.print("\r\nSet MagCali Error\r\n");
      break;
    case 'e':
      if (WitStopMagCali() != WIT_HAL_OK)
        Serial.print("\r\nSet MagCali Error\r\n");
      break;
    case 'u':
      if (WitSetBandwidth(BANDWIDTH_5HZ) != WIT_HAL_OK)
        Serial.print("\r\nSet Bandwidth Error\r\n");
      break;
    case 'U':
      if (WitSetBandwidth(BANDWIDTH_256HZ) != WIT_HAL_OK)
        Serial.print("\r\nSet Bandwidth Error\r\n");
      break;
    case 'B':
      if (WitSetUartBaud(WIT_BAUD_115200) != WIT_HAL_OK)
        Serial.print("\r\nSet Baud Error\r\n");
      else {
        Serial1.begin(c_uiBaud[WIT_BAUD_115200]);
        Serial.print(" 115200 Baud rate modified successfully\r\n");
      }
      break;
    case 'b':
      if (WitSetUartBaud(WIT_BAUD_9600) != WIT_HAL_OK)
        Serial.print("\r\nSet Baud Error\r\n");
      else {
        Serial1.begin(c_uiBaud[WIT_BAUD_9600]);
        Serial.print(" 9600 Baud rate modified successfully\r\n");
      }
      break;
    case 'r':
      if (WitSetOutputRate(RRATE_1HZ) != WIT_HAL_OK)
        Serial.print("\r\nSet Baud Error\r\n");
      else
        Serial.print("\r\nSet Baud Success\r\n");
      break;
    case 'R':
      if (WitSetOutputRate(RRATE_10HZ) != WIT_HAL_OK)
        Serial.print("\r\nSet Baud Error\r\n");
      else
        Serial.print("\r\nSet Baud Success\r\n");
      break;
    case 'C':
      if (WitSetContent(RSW_ACC | RSW_GYRO | RSW_ANGLE | RSW_MAG) != WIT_HAL_OK)
        Serial.print("\r\nSet RSW Error\r\n");
      break;
    case 'c':
      if (WitSetContent(RSW_ACC) != WIT_HAL_OK)
        Serial.print("\r\nSet RSW Error\r\n");
      break;
    case 'h':
      ShowHelp();
      break;
    default:
      break;
  }
  s_cCmd = 0xff;
}

static void SensorUartSend(uint8_t *p_data, uint32_t uiSize) {
  Serial1.write(p_data, uiSize);
  Serial1.flush();
}

static void Delayms(uint16_t ucMs) {
  delay(ucMs);
}

static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum) {
  int i;
  for (i = 0; i < uiRegNum; i++) {
    switch (uiReg) {
      case AZ:
        s_cDataUpdate |= ACC_UPDATE;
        break;
      case GZ:
        s_cDataUpdate |= GYRO_UPDATE;
        break;
      case HZ:
        s_cDataUpdate |= MAG_UPDATE;
        break;
      case Yaw:
        s_cDataUpdate |= ANGLE_UPDATE;
        break;
      default:
        s_cDataUpdate |= READ_UPDATE;
        break;
    }
    uiReg++;
  }
}

static void AutoScanSensor(void) {
  int i, iRetry;

  for (i = 0; i < sizeof(c_uiBaud) / sizeof(c_uiBaud[0]); i++) {
    Serial1.begin(c_uiBaud[i]);
    Serial1.flush();
    iRetry = 2;
    s_cDataUpdate = 0;
    do {
      WitReadReg(AX, 3);
      delay(200);
      while (Serial1.available()) {
        WitSerialDataIn(Serial1.read());
      }
      if (s_cDataUpdate != 0) {
        Serial.print(c_uiBaud[i]);
        Serial.print(" baud find sensor\r\n\r\n");
        ShowHelp();
        return;
      }
      iRetry--;
    } while (iRetry);
  }
  Serial.print("can not find sensor\r\n");
  Serial.print("please check your connection\r\n");
}
