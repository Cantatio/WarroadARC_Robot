#include <Servo.h>

const int NUM_JOINTS = 6;

Servo servos[NUM_JOINTS];
const int servoPins[NUM_JOINTS] = {3, 5, 6, 9, 10, 11};

int currentPos[NUM_JOINTS] = {90, 90, 90, 90, 90, 60};
int targetPos[NUM_JOINTS]  = {90, 90, 90, 90, 90, 60};

int minPos[NUM_JOINTS] = {10, 20, 15, 0, 0, 30};
int maxPos[NUM_JOINTS] = {170, 160, 165, 180, 180, 120};

int offsets[NUM_JOINTS] = {0, 0, 0, 0, 0, 0};
bool reversed[NUM_JOINTS] = {false, true, false, false, false, true};

unsigned long lastStep = 0;
int stepIntervalMs = 20;

int clampJoint(int joint, int value) {
  if (value < minPos[joint]) return minPos[joint];
  if (value > maxPos[joint]) return maxPos[joint];
  return value;
}

int applyCalibration(int joint, int value) {
  if (reversed[joint]) {
    value = maxPos[joint] - (value - minPos[joint]);
  }
  value += offsets[joint];
  return clampJoint(joint, value);
}

void moveStepTowardTarget() {
  for (int i = 0; i < NUM_JOINTS; i++) {
    if (currentPos[i] < targetPos[i]) currentPos[i]++;
    else if (currentPos[i] > targetPos[i]) currentPos[i]--;
    int calibrated = applyCalibration(i, currentPos[i]);
    servos[i].write(calibrated);
  }
}

void printStatus() {
  Serial.print("POS ");
  for (int i = 0; i < NUM_JOINTS; i++) {
    Serial.print(currentPos[i]);
    if (i < NUM_JOINTS - 1) Serial.print(' ');
  }
  Serial.println();
}

void parseMoveCommand(char* args) {
  int values[NUM_JOINTS];
  int count = 0;
  char* token = strtok(args, " ");
  while (token != NULL && count < NUM_JOINTS) {
    values[count++] = atoi(token);
    token = strtok(NULL, " ");
  }
  if (count != NUM_JOINTS) {
    Serial.println("ERR BAD_ARGS");
    return;
  }
  for (int i = 0; i < NUM_JOINTS; i++) {
    targetPos[i] = clampJoint(i, values[i]);
  }
  Serial.println("OK MOVE");
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < NUM_JOINTS; i++) {
    servos[i].attach(servoPins[i]);
    servos[i].write(applyCalibration(i, currentPos[i]));
  }
  Serial.println("ARM READY (CALIBRATED)");
}

void parseCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line == "PING") {
    Serial.println("PONG");
    return;
  }

  if (line == "STATUS") {
    printStatus();
    return;
  }

  if (line == "HOME") {
    int home[NUM_JOINTS] = {90, 90, 90, 90, 90, 60};
    for (int i = 0; i < NUM_JOINTS; i++) targetPos[i] = home[i];
    Serial.println("OK HOME");
    return;
  }

  if (line == "STOP") {
    for (int i = 0; i < NUM_JOINTS; i++) targetPos[i] = currentPos[i];
    Serial.println("OK STOP");
    return;
  }

  char buf[96];
  line.toCharArray(buf, sizeof(buf));

  if (strncmp(buf, "M ", 2) == 0) {
    parseMoveCommand(buf + 2);
    return;
  }

  if (strncmp(buf, "OFFSET ", 7) == 0) {
    int j, val;
    sscanf(buf + 7, "%d %d", &j, &val);
    if (j >= 0 && j < NUM_JOINTS) {
      offsets[j] = val;
      Serial.println("OK OFFSET");
      return;
    }
  }

  if (strncmp(buf, "REV ", 4) == 0) {
    int j, val;
    sscanf(buf + 4, "%d %d", &j, &val);
    if (j >= 0 && j < NUM_JOINTS) {
      reversed[j] = (val != 0);
      Serial.println("OK REV");
      return;
    }
  }

  if (strncmp(buf, "SPEED ", 6) == 0) {
    int newInterval = atoi(buf + 6);
    if (newInterval >= 5 && newInterval <= 100) {
      stepIntervalMs = newInterval;
      Serial.println("OK SPEED");
      return;
    }
    Serial.println("ERR BAD_SPEED");
    return;
  }

  Serial.println("ERR UNKNOWN");
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    parseCommand(line);
  }

  if (millis() - lastStep >= (unsigned long)stepIntervalMs) {
    lastStep = millis();
    moveStepTowardTarget();
  }
}
