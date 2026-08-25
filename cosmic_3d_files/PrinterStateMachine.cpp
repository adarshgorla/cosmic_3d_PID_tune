#include "PrinterStateMachine.h"

volatile SystemState sysState = STATE_IDLE;
bool r_home = false, z_home = false, theta_home = false;
long setpointHome[4] = {0, 1000, 0, 0};
unsigned long dwellStart = 0;
SystemState pausedPrintState = STATE_IDLE;

CartesianSegment segmentBuffer[MAX_INTERPOLATION_SEGMENTS];
int currentSegmentIndex = 0;
int totalSegmentsCount = 0;

static bool mk9CoreReady = false;

void cosmicPolarSetup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n=== Cosmic Polar 400 Modular Controller Starting ===");

  pinMode(motorPinA1, OUTPUT);
  pinMode(motorPinA2, OUTPUT);
  pinMode(motorPinB1, OUTPUT);
  pinMode(motorPinB2, OUTPUT);
  pinMode(motorPinc1, OUTPUT);
  pinMode(motorPinc2, OUTPUT);
  pinMode(motorPind1, OUTPUT);
  pinMode(motorPind2, OUTPUT);
  stopAllMotors();

  pinMode(THETA_HOME_SENSOR_PIN, INPUT_PULLUP);
  pinMode(Z_LIMIT_PIN, INPUT_PULLUP);

  pinMode(HEATER_PIN, OUTPUT);
  digitalWrite(HEATER_PIN, LOW);
  pinMode(BED_HEATER_PIN, OUTPUT);
  digitalWrite(BED_HEATER_PIN, LOW);

#ifdef ARDUINO
  // Initialize MG945 Servo for Theta Axis
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servoTheta.setPeriodHertz(50);
  servoTheta.attach(THETA_SERVO_PIN, 500, 2400);
  setThetaServoAngle(SERVO_MIN_ANGLE_DEG);
  Serial.printf("[SERVO] MG945 Theta Servo attached to GPIO %d (Min: %d deg)\n",
                THETA_SERVO_PIN, SERVO_MIN_ANGLE_DEG);

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("[SD] SD Card Mount Failed.");
  } else {
    Serial.println("[SD] SD Card initialized.");
  }

  prefs.begin("mk9_state", false);
  loadState();

  initAS5600();

  mqttClient.setUsernamePassword(MQTT_USER, MQTT_PASSWORD);
  if (WiFi.status() == WL_CONNECTED) {
    if (mqttClient.connect(MQTT_BROKER, 1883)) {
      Serial.println("[MQTT] Initial connection successful ✅");
      mqttClient.subscribe(TOPIC_START_file_start_stop);
      mqttClient.subscribe(TOPIC_STOP);
      mqttClient.subscribe(TOPIC_xyz_move);
      mqttClient.subscribe(MOTOR_START);
      mqttClient.subscribe(MOTOR_STOP);
    }
  }
#endif

  Serial.println("[SETUP] Cosmic Polar 400 Core Ready.");
  mk9CoreReady = true;
}

void cosmicPolarLoop() {
  if (!mk9CoreReady) {
    delay(10);
    return;
  }

#ifdef ARDUINO
  // --- MQTT Maintenance ---
  if (!mqttClient.connected()) {
    if (WiFi.status() == WL_CONNECTED && !isActivePrintState(sysState)) {
      unsigned long now = millis();
      if (now - lastAttemptTime > reconnectDelay) {
        lastAttemptTime = now;
        mqttReconnect();
      }
    }
  } else {
    mqttClient.poll();
  }
#endif

  // --- Heater Control ---
  static unsigned long lastHeaterPoll = 0;
  if (millis() - lastHeaterPoll >= HEATER_POLL_MS) {
    lastHeaterPoll = millis();
    runHeaterControl();
  }

  // --- Encoder Diagnostics Poll ---
  static unsigned long lastDiagnostic = 0;
  if (millis() - lastDiagnostic > 30000) {
    lastDiagnostic = millis();
    checkAllEncoders();
  }

  // --- Process Interactive Serial Commands (for Axis Testing) ---
  if (Serial.available() > 0) {
    String serCmd = Serial.readStringUntil('\n');
    serCmd.trim();
    serCmd.toUpperCase();

    if (serCmd == "HOME") {
      Serial.println("[CMD] Homing Cosmic Polar 400...");
      r_home = false;
      z_home = false;
      theta_home = false;
      sysState = STATE_HOMING_SEEK;
    } else if (serCmd.startsWith("R ")) {
      float bedDeg = serCmd.substring(2).toFloat();
      long rCounts = bedAngleToMotorCounts(bedDeg);
      setpoint[0] = rCounts;
      Serial.printf("[TEST] R Bed Target: %.2f deg → %ld counts\n", bedDeg, rCounts);
      sysState = STATE_MOTION_PID;
    } else if (serCmd.startsWith("THETA ")) {
      float radialMm = serCmd.substring(6).toFloat();
      int sAngle = radialMmToServoAngle(radialMm);
      setThetaServoAngle(sAngle);
      Serial.printf("[TEST] Theta Target: %.2f mm → Servo %d deg\n", radialMm, sAngle);
    } else if (serCmd.startsWith("Z ")) {
      float zMm = serCmd.substring(2).toFloat();
      long zCounts = zMmToEncoderCounts(zMm);
      setpoint[1] = zCounts;
      Serial.printf("[TEST] Z Target: %.2f mm → %ld counts\n", zMm, zCounts);
      sysState = STATE_MOTION_PID;
    } else if (serCmd == "POS?") {
      checkAllEncoders();
    }
  }

  // --- Process MQTT Messages ---
  if (messageReceived) {
    messageReceived = false;
    Serial.printf("[LOOP] Topic: %s\n", topicBuffer.c_str());

    if (topicBuffer == MOTOR_STOP) {
      if (payloadBuffer == "STOP") {
        Serial.println("[MOTION] Emergency Stop.");
        stopAllMotors();
#ifdef ARDUINO
        if (file)
          file.close();
#endif
        sysState = STATE_IDLE;
        pausedPrintState = STATE_IDLE;
        manualMovePending = false;
      } else if (payloadBuffer == "PAUSE") {
        if (isActivePrintState(sysState)) {
          Serial.println("[MOTION] Pausing print safely...");
          pausedPrintState = sysState;
          sysState = STATE_MOTION_PAUSED;
        }
      }
    } else if (topicBuffer == TOPIC_START_file_start_stop) {
      if (payloadBuffer.indexOf('|') != -1) {
        int pipe1 = payloadBuffer.indexOf('|');
        int pipe2 = payloadBuffer.indexOf('|', pipe1 + 1);
        int pipe3 = payloadBuffer.indexOf('|', pipe2 + 1);

        if (pipe1 != -1 && pipe2 != -1) {
          if (pipe1 == 0 && pipe3 != -1) {
            fileName = payloadBuffer.substring(pipe1 + 1, pipe2);
            totalChunks = payloadBuffer.substring(pipe2 + 1, pipe3).toInt();
            expectedChecksum = payloadBuffer.substring(pipe3 + 1);
          } else {
            fileName = payloadBuffer.substring(0, pipe1);
            totalChunks = payloadBuffer.substring(pipe1 + 1, pipe2).toInt();
            expectedChecksum = payloadBuffer.substring(pipe2 + 1);
          }
          fileName.trim();
          expectedChecksum.trim();
          if (!fileName.startsWith("/")) {
            fileName = "/" + fileName;
          }

          lastReceivedChunk = -1;
          fileReceiving = true;
#ifdef ARDUINO
          if (SD.exists(fileName.c_str()))
            SD.remove(fileName.c_str());
#endif
          saveState();
          Serial.printf("[XFER] Start: file=%s chunks=%d checksum=%s\n",
                        fileName.c_str(), totalChunks, expectedChecksum.c_str());
#ifdef ARDUINO
          mqttClient.beginMessage(TOPIC_ACK);
          mqttClient.print("READY");
          mqttClient.endMessage();
#endif
        }
      } else if (payloadBuffer.indexOf('/') != -1) {
        int sep = payloadBuffer.indexOf('/');
        String command = payloadBuffer.substring(0, sep);
        int value = payloadBuffer.substring(sep + 1).toInt();

        if (command == "hotendtemp") {
          hotendSetpoint = (float)value;
          Serial.printf("[HEAT] Hotend setpoint → %.1f°C\n", hotendSetpoint);
        } else if (command == "bedtemp") {
          bedSetpoint = (float)value;
          Serial.printf("[HEAT] Bed setpoint → %.1f°C\n", bedSetpoint);
        }
      } else {
        String command = payloadBuffer;
        command.trim();

        if (command == "home" && sysState == STATE_IDLE) {
          Serial.println("[HOME] Polar Homing all axes...");
          r_home = false;
          z_home = false;
          theta_home = false;
          sysState = STATE_HOMING_SEEK;
        } else if (command == "stop") {
          Serial.println("[CMD] Stop.");
          stopAllMotors();
#ifdef ARDUINO
          if (file)
            file.close();
#endif
          sysState = STATE_IDLE;
        }
      }
    } else if (topicBuffer == TOPIC_STOP && fileReceiving) {
      int pipe1 = payloadBuffer.indexOf('|');
      int pipe2 = payloadBuffer.indexOf('|', pipe1 + 1);

      if (pipe1 != -1 && pipe2 != -1) {
        int chunkID = payloadBuffer.substring(pipe1 + 1, pipe2).toInt();
        if (chunkID <= lastReceivedChunk) {
          Serial.printf("[XFER] Duplicate chunk %d, ignored.\n", chunkID);
        } else if (chunkID != lastReceivedChunk + 1) {
#ifdef ARDUINO
          mqttClient.beginMessage(TOPIC_ACK);
          mqttClient.print("RESEND|");
          mqttClient.print(lastReceivedChunk + 1);
          mqttClient.endMessage();
#endif
        } else {
#ifdef ARDUINO
          appendFile(SD, fileName.c_str(), payloadBuffer.substring(pipe2 + 1).c_str());
#endif
          lastReceivedChunk = chunkID;
          saveState();
#ifdef ARDUINO
          mqttClient.beginMessage(TOPIC_ACK);
          mqttClient.print("ACK|");
          mqttClient.print(chunkID);
          mqttClient.endMessage();
#endif
        }
      }
    } else if (topicBuffer == TOPIC_xyz_move) {
      String cmd = payloadBuffer;
      cmd.trim();
      cmd.toUpperCase();

      char firstChar = cmd.length() > 0 ? cmd.charAt(0) : '\0';
      float dx = 0.0f, dy = 0.0f, dz = 0.0f;
      bool valid = false;

      if (firstChar == 'X' || firstChar == 'Y' || firstChar == 'Z') {
        if (cmd.length() >= 2) {
          char axis = firstChar;
          char sign = cmd.charAt(1);
          if (sign == '+' || sign == '-') {
            float amount = cmd.substring(2).toFloat();
            if (amount == 0.0f)
              amount = 1.0f;
            if (sign == '-')
              amount = -amount;

            if (axis == 'X')
              dx = amount;
            else if (axis == 'Y')
              dy = amount;
            else if (axis == 'Z')
              dz = amount;
            valid = true;
          }
        }
      } else {
        if (cmd == "X+") {
          dx = MANUAL_JOG_MM;
          valid = true;
        } else if (cmd == "X-") {
          dx = -MANUAL_JOG_MM;
          valid = true;
        } else if (cmd == "Y+") {
          dy = MANUAL_JOG_MM;
          valid = true;
        } else if (cmd == "Y-") {
          dy = -MANUAL_JOG_MM;
          valid = true;
        } else if (cmd == "Z+") {
          dz = MANUAL_JOG_MM;
          valid = true;
        } else if (cmd == "Z-") {
          dz = -MANUAL_JOG_MM;
          valid = true;
        }
      }

      if (valid) {
        if (isActivePrintState(sysState)) {
          queueManualMove(dx, dy, dz);
          pausedPrintState = sysState;
          sysState = STATE_MOTION_PAUSED;
        } else if (sysState == STATE_MOTION_PAUSED ||
                   sysState == STATE_MANUAL_XYZ || sysState == STATE_IDLE) {
          if (sysState != STATE_MANUAL_XYZ) {
            resetManualXYZTarget();
            sysState = STATE_MANUAL_XYZ;
          }
          queueManualMove(dx, dy, dz);
        }
      }
    } else if (topicBuffer == MOTOR_START) {
      if (payloadBuffer == "RESUME" &&
          (sysState == STATE_MOTION_PAUSED || sysState == STATE_MANUAL_XYZ)) {
        if (pausedPrintState != STATE_IDLE) {
          sysState = pausedPrintState;
          pausedPrintState = STATE_IDLE;
        } else {
          stopAllMotors();
          sysState = STATE_IDLE;
        }
      } else if (sysState == STATE_IDLE) {
        for (int i = 0; i < 4; i++) {
          updateEncoderFromAS5600(i);
          totalRotations[i] = 0;
        }
        sysState = STATE_MOTION_OPEN;
      }
    }
  }

  delay(1);

  // ===========================================================================
  // STATE MACHINE — NON-BLOCKING COOPERATIVE EXECUTION
  // ===========================================================================
  switch (sysState) {

  case STATE_IDLE:
    break;

  case STATE_MOTION_PAUSED: {
    stopAllMotors();
    if (manualMovePending) {
      resetManualXYZTarget();
      applyPendingManualMove();
      sysState = STATE_MANUAL_XYZ;
    }
    break;
  }

  case STATE_MANUAL_XYZ: {
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTimePID) / 1000.0f;
    if (dt <= 0.0f)
      dt = 0.001f;
    lastTimePID = currentTime;

    updateEncoderFromAS5600(0); // R DC Motor
    updateEncoderFromAS5600(1); // Z DC Motor
    updateEncoderFromAS5600(3); // E Motor

    if (manualMovePending)
      applyPendingManualMove();

    setThetaServoAngle(manualServoTarget);

    bool rReached =
        abs(manualTarget[0] - encoderCount[0]) <= MOTION_TOLERANCE_COUNTS;
    bool zReached =
        abs(manualTarget[1] - encoderCount[1]) <= MOTION_TOLERANCE_COUNTS;

    if (rReached && zReached) {
      stopAllMotors();
    } else {
      driveMotor(0, calculatePID(0, manualTarget[0], dt));
      driveMotor(1, calculatePID(1, manualTarget[1], dt));
    }
    break;
  }

  // --- POLAR HOMING ARCHITECTURE ---
  case STATE_HOMING_SEEK: {
    // R Bed Rotation Homing
    if (!r_home) {
      analogWrite(motorPinA1, 60);
      analogWrite(motorPinA2, 0);
      if (digitalRead(THETA_HOME_SENSOR_PIN) == LOW) {
        analogWrite(motorPinA1, 0);
        r_home = true;
        encoderCount[0] = 0;
        rawAccumulator[0] = 0;
        lastBedAngleDeg = 0.0f;
        accumulatedBedAngleDeg = 0.0f;
        Serial.println("[HOME] R Bed reference sensor reached.");
      }
    }
    // Z Height Homing
    if (!z_home) {
      analogWrite(motorPinB1, 0);
      analogWrite(motorPinB2, 100);
      if (digitalRead(Z_LIMIT_PIN) == LOW) {
        analogWrite(motorPinB2, 0);
        z_home = true;
        encoderCount[1] = 0;
        rawAccumulator[1] = 0;
        Serial.println("[HOME] Z Limit switch reached.");
      }
    }
    // Theta Servo Homing
    if (!theta_home) {
      setThetaServoAngle(SERVO_MIN_ANGLE_DEG);
      theta_home = true;
      Serial.println("[HOME] Theta MG945 Servo homed to 0 deg.");
    }

    if (r_home && z_home && theta_home) {
      sysState = STATE_HOMING_ZERO;
    }
    break;
  }

  case STATE_HOMING_ZERO: {
    for (int i = 0; i < 4; i++) {
      if (i == 2)
        continue;
      if (as5600Ready && selectI2CChannel(motorToChannel[i])) {
        delay(5);
        lastAngle[i] = readRawAngle();
      } else {
        lastAngle[i] = 0;
      }
      encoderCount[i] = 0;
      rawAccumulator[i] = 0;
      totalRotations[i] = 0;
      delay(1);
    }
    disableAllI2CChannels();

    setpointHome[0] = 0;
    setpointHome[1] = zMmToEncoderCounts(10.0f); // 10 mm standoff
    setpointHome[2] = SERVO_MIN_ANGLE_DEG;

    lastTimePID = millis();
    for (int i = 0; i < 4; i++) {
      integral[i] = 0.0f;
      prevError[i] = 0;
    }
    Serial.println("[HOME] Moving Z to standoff (10 mm)...");
    sysState = STATE_HOMING_STANDOFF;
    break;
  }

  case STATE_HOMING_STANDOFF: {
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTimePID) / 1000.0f;
    if (dt <= 0.0f)
      dt = 0.001f;
    lastTimePID = currentTime;

    updateEncoderFromAS5600(0);
    updateEncoderFromAS5600(1);

    long zError = setpointHome[1] - encoderCount[1];
    if (abs(zError) <= MOTION_TOLERANCE_COUNTS) {
      stopAllMotors();
      Serial.println("[HOME] Polar homing sequence complete ✅");
      currentX = HOME_X;
      currentY = HOME_Y;
      currentZ = 10.0f;
      sysState = STATE_IDLE;
    } else {
      driveMotor(1, calculatePID(1, setpointHome[1], dt));
    }
    break;
  }

  case STATE_MOTION_OPEN: {
#ifdef ARDUINO
    listDir(SD, "/", 0);
    file = SD.open(fileName.c_str());
    if (!file) {
      Serial.printf("[MOTION] Failed to open file: %s\n", fileName.c_str());
      sysState = STATE_IDLE;
      break;
    }
    totalLines = countLinesInFile(SD, fileName.c_str());
#endif
    myconut = 0;
    dwellStart = millis();
    sysState = STATE_MOTION_DWELL;
    break;
  }

  case STATE_MOTION_READ_LINE: {
    String motionLine = readNextLine();
    motionLine.trim();

    float targetX = 0.0f, targetY = 0.0f, targetZ = 0.0f, targetE = 0.0f;
    if (!parseCartesianLine(motionLine, targetX, targetY, targetZ, targetE)) {
      myconut++;
      dwellStart = millis();
      sysState = STATE_MOTION_DWELL;
      break;
    }

    // Interpolate line into Cartesian segments
    totalSegmentsCount = generateCartesianSegments(
        currentX, currentY, currentZ, 0.0f, targetX, targetY, targetZ, targetE,
        segmentBuffer, MAX_INTERPOLATION_SEGMENTS);
    currentSegmentIndex = 0;
    myconut++;

    dwellStart = millis();
    sysState = STATE_MOTION_DWELL;
    break;
  }

  case STATE_MOTION_DWELL: {
    if (millis() - dwellStart >= 50) {
      if (currentSegmentIndex < totalSegmentsCount) {
        // Execute next segment of current line
        CartesianSegment seg = segmentBuffer[currentSegmentIndex++];
        long targetR = 0, targetZ = 0;
        int targetTheta = SERVO_MIN_ANGLE_DEG;

        if (!calculatePolarIK(seg.x, seg.y, seg.z, targetR, targetTheta,
                              targetZ)) {
          Serial.println("[MOTION] Segment Polar IK failed. Stopping.");
          stopAllMotors();
#ifdef ARDUINO
          if (file)
            file.close();
#endif
          sysState = STATE_IDLE;
          break;
        }

        setpoint[0] = targetR;
        setpoint[1] = targetZ;
        setpoint[3] = (long)lroundf(seg.e * STEPS_PER_MM_E);
        setThetaServoAngle(targetTheta);

        currentX = seg.x;
        currentY = seg.y;
        currentZ = seg.z;

        lastTimePID = millis();
        for (int i = 0; i < 4; i++) {
          integral[i] = 0.0f;
          prevError[i] = 0;
        }
        sysState = STATE_MOTION_PID;
      } else {
        if (myconut >= totalLines) {
          sysState = STATE_MOTION_DONE;
        } else {
          sysState = STATE_MOTION_READ_LINE;
        }
      }
    }
    break;
  }

  case STATE_MOTION_PID: {
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTimePID) / 1000.0f;
    if (dt <= 0.0f)
      dt = 0.001f;
    lastTimePID = currentTime;

    updateEncoderFromAS5600(0); // R Motor
    updateEncoderFromAS5600(1); // Z Motor
    updateEncoderFromAS5600(3); // E Motor

    bool rReached =
        abs(setpoint[0] - encoderCount[0]) <= MOTION_TOLERANCE_COUNTS;
    bool zReached =
        abs(setpoint[1] - encoderCount[1]) <= MOTION_TOLERANCE_COUNTS;
    bool eReached =
        abs(setpoint[3] - encoderCount[3]) <= MOTION_TOLERANCE_COUNTS;

    if (rReached && zReached && eReached) {
      dwellStart = millis();
      sysState = STATE_MOTION_DWELL;
    } else {
      driveMotor(0, calculatePID(0, setpoint[0], dt));
      driveMotor(1, calculatePID(1, setpoint[1], dt));
      driveMotor(3, calculatePID(3, setpoint[3], dt));
    }
    break;
  }

  case STATE_MOTION_DONE: {
    stopAllMotors();
#ifdef ARDUINO
    if (file)
      file.close();
#endif
    Serial.println("[MOTION] Print complete.");
    checkAllEncoders();
    sysState = STATE_IDLE;
    break;
  }
  }
}
