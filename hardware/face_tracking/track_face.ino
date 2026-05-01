#include <AccelStepper.h>

const int ROLL_DIRPIN   = 26;
const int ROLL_STEPPIN  = 25;
const int PITCH_DIRPIN  = 33;
const int PITCH_STEPPIN = 32;
const int YAW_DIRPIN    = 18;
const int YAW_STEPPIN   = 19;

const int ROLL_MS1      = 4;
const int ROLL_MS2      = 2;
const int ROLL_MS3      = 15;
const int PITCH_MS1     = 5;
const int PITCH_MS2     = 17;
const int PITCH_MS3     = 16;
const int YAW_MS1       = 23;
const int YAW_MS2       = 1;
const int YAW_MS3       = 3;

const int   MICROSTEPS    = 16;
const float STEPS_PER_DEG = (200.0f * MICROSTEPS) / 360.0f;  // 8.889 steps/deg

AccelStepper nemaRoll (AccelStepper::DRIVER, ROLL_STEPPIN,  ROLL_DIRPIN);
AccelStepper nemaPitch(AccelStepper::DRIVER, PITCH_STEPPIN, PITCH_DIRPIN);
AccelStepper nemaYaw  (AccelStepper::DRIVER, YAW_STEPPIN,   YAW_DIRPIN);

SemaphoreHandle_t motorMutex;
volatile long rollTarget  = 0;
volatile long pitchTarget = 0;
volatile long yawTarget   = 0;

void setMicrostepping(int ms1, int ms2, int ms3, int microsteps) {
  switch (microsteps) {
    case 1:  digitalWrite(ms1, LOW);  digitalWrite(ms2, LOW);  digitalWrite(ms3, LOW);  break;
    case 2:  digitalWrite(ms1, HIGH); digitalWrite(ms2, LOW);  digitalWrite(ms3, LOW);  break;
    case 4:  digitalWrite(ms1, LOW);  digitalWrite(ms2, HIGH); digitalWrite(ms3, LOW);  break;
    case 8:  digitalWrite(ms1, HIGH); digitalWrite(ms2, HIGH); digitalWrite(ms3, LOW);  break;
    case 16: digitalWrite(ms1, HIGH); digitalWrite(ms2, HIGH); digitalWrite(ms3, HIGH); break;
    default: digitalWrite(ms1, LOW);  digitalWrite(ms2, LOW);  digitalWrite(ms3, LOW);  break;
  }
}

// Parses "Y:<yaw>,P:<pitch>\n" sent by track_face.py at 10 Hz
void serialTask(void *params) {
  while (true) {
    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();

      int yIdx = cmd.indexOf("Y:");
      int pIdx = cmd.indexOf(",P:");
      if (yIdx != -1 && pIdx != -1) {
        float yaw   = cmd.substring(yIdx + 2, pIdx).toFloat();
        float pitch = cmd.substring(pIdx + 3).toFloat();

        long yawSteps   = (long)(yaw   * STEPS_PER_DEG);
        long pitchSteps = (long)(pitch * STEPS_PER_DEG);

        xSemaphoreTake(motorMutex, portMAX_DELAY);
        yawTarget   = yawSteps;
        pitchTarget = pitchSteps;
        xSemaphoreGive(motorMutex);

        Serial.printf("ACK Y:%.2f(%ld steps) P:%.2f(%ld steps)\n",
                      yaw, yawSteps, pitch, pitchSteps);
      }
    }
    vTaskDelay(1);
  }
}

void motorTask(void *params) {
  while (true) {
    xSemaphoreTake(motorMutex, portMAX_DELAY);
    nemaRoll.moveTo(rollTarget);
    nemaPitch.moveTo(pitchTarget);
    nemaYaw.moveTo(yawTarget);
    xSemaphoreGive(motorMutex);

    for (int i = 0; i < 20; i++) {
      nemaRoll.run();
      nemaPitch.run();
      nemaYaw.run();
    }

    taskYIELD();
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(ROLL_MS1,  OUTPUT); pinMode(ROLL_MS2,  OUTPUT); pinMode(ROLL_MS3,  OUTPUT);
  pinMode(ROLL_DIRPIN,  OUTPUT); pinMode(ROLL_STEPPIN,  OUTPUT);
  setMicrostepping(ROLL_MS1, ROLL_MS2, ROLL_MS3, MICROSTEPS);

  pinMode(PITCH_MS1, OUTPUT); pinMode(PITCH_MS2, OUTPUT); pinMode(PITCH_MS3, OUTPUT);
  pinMode(PITCH_DIRPIN, OUTPUT); pinMode(PITCH_STEPPIN, OUTPUT);
  setMicrostepping(PITCH_MS1, PITCH_MS2, PITCH_MS3, MICROSTEPS);

  pinMode(YAW_MS1,  OUTPUT); pinMode(YAW_MS2,  OUTPUT); pinMode(YAW_MS3,  OUTPUT);
  pinMode(YAW_DIRPIN,  OUTPUT); pinMode(YAW_STEPPIN,  OUTPUT);
  setMicrostepping(YAW_MS1, YAW_MS2, YAW_MS3, MICROSTEPS);

  nemaRoll.setMaxSpeed(3000);  nemaRoll.setAcceleration(2000);
  nemaPitch.setMaxSpeed(3000); nemaPitch.setAcceleration(2000);
  nemaYaw.setMaxSpeed(3000);   nemaYaw.setAcceleration(2000);

  motorMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(serialTask, "Serial", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(motorTask,  "Motor",  4096, NULL, 2, NULL, 1);
}

void loop() {}
