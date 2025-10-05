const int SENS[6] = { A5, A4, A3, A2, A1, A0 };

#define AIN1 10         // motor trái PWM
#define AIN2 9          // motor trái PWM
#define BIN1 3          // motor phải PWM
#define BIN2 11         // motor phải PWM
#define BUZZER 4        // chân còi
#define BUTTON_START 7  // nút nhấn 

const unsigned long LOST_ALARM_AFTER_MS = 1000;
const unsigned long LOST_BEEP_INTERVAL = 250;
const unsigned long LOST_BEEP_DURATION = 120;

unsigned long lastBeepMs = 0;

const bool BLACK_IS_LOW = true;
int THRESHOLD = 200;
int TH[6] = { 0, 0, 0, 0, 0, 0 };

// PID và tốc độ
int baseSpeed = 200;
int trim = 0;
float Kp = 44.0;
float Ki = 0.1;
float Kd = 50.0;

int recoverSpeed = 120;
int recoverTimeout = 400;
float Kp_center_scale = 0.6f;

const int STATE_CENTER_LINE = 0; 
const int STATE_FIRST_TURN = 1;  
const int STATE_LOOP = 2;        
const int STATE_END = 3;       


const int TURN_SPEED = 200;       
const int TURN_DURATION_MS = 200; 

// Biến toàn cục
uint8_t lastMap = 0;
int lastError = 0;
long sumError = 0;
unsigned long lastSeenMillis = 0;
int missionState = 0;  
int loopIntersectionCount = 0; 

void setWheel(int in1, int in2, int dPWM) {
    dPWM = constrain(dPWM, -255, 255);
    if (dPWM >= 0) {
        analogWrite(in1, dPWM);
        analogWrite(in2, 0);
    } else {
        analogWrite(in1, 0);
        analogWrite(in2, -dPWM);
    }
}

void drive(int left, int right) {
    left = constrain(left + trim, -255, 255);
    right = constrain(right - trim, -255, 255);
    setWheel(AIN1, AIN2, left);
    setWheel(BIN1, BIN2, right);
}

void printBin6(uint8_t m) {
    for (int b = 5; b >= 0; --b) Serial.print((m & (1 << b)) ? '1' : '0');
}

// đọc cảm biến
uint8_t readSensors() {
    uint8_t Map = 0x00;

    Serial.print("RAW: ");
    for (int i = 0; i < 6; i++) {
        int val = analogRead(SENS[i]);
        int th = (TH[i] > 0 ? TH[i] : THRESHOLD);

        bool isBlack = BLACK_IS_LOW ? (val < th) : (val > th);
        if (isBlack) Map |= (1 << (5 - i));

        Serial.print("A"); Serial.print(5 - i); Serial.print("="); Serial.print(val);
        Serial.print(isBlack ? "(B) " : "(W) ");
    }
    Serial.print(" | Map:");
    printBin6(Map);
    Serial.println();

    if (Map != 0) {
        lastMap = Map;
        lastSeenMillis = millis();
    }
    return (Map != 0 ? Map : lastMap);
}

// Tính lỗi PID
int computeErrorFromMap(uint8_t Map) {
    const int W[6] = { -5, -3, -1, +1, +3, +5 };
    int sumW = 0, cnt = 0;

    for (int i = 0; i < 6; i++) {
        if (Map & (1 << (5 - i))) {
            sumW += W[i];
            cnt++;
        }
    }
    if (cnt == 0) return lastError;
    return (sumW * 100) / cnt;
}

// xác định tâm của line
bool isCenterBand(uint8_t Map) {
    bool midLeft = Map & (1 << 3);
    bool midRight = Map & (1 << 2);
    return midLeft && midRight;
}

bool isCurrentlyLost() {
    return (millis() - lastSeenMillis) > (unsigned long)recoverTimeout;
}

void recoverSpin() {
    if (lastError > 0) {
        drive(+recoverSpeed, -recoverSpeed);  // quay phải
    } else {
        drive(-recoverSpeed, +recoverSpeed);  // quay trái
    }
    Serial.println(lastError > 0 ? "Recover: spin RIGHT" : "Recover: spin LEFT");
}

// ngã tư
void turnLeftFixed() {
    drive(-TURN_SPEED, TURN_SPEED);
    delay(TURN_DURATION_MS);
}

void turnRightFixed() {
    drive(TURN_SPEED, -TURN_SPEED);
    delay(TURN_DURATION_MS);
}

void goStraightFixed() {
    drive(baseSpeed + 20, baseSpeed + 20);
    delay(30); 
}

bool isIntersection(uint8_t Map) {
    int count_on = 0;
    for (int i = 0; i < 6; i++) {
        if (Map & (1 << (5 - i))) {
            count_on++;
        }
    }
    return (count_on >= 4); 
}

void setup() {
    Serial.begin(115200);

    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(BUZZER, OUTPUT);
    pinMode(BUTTON_START, INPUT_PULLUP);
    drive(0, 0);

    delay(300);
    for (int i = 2000; i < 3500; i += 500) {
        tone(BUZZER, i, 100);
        delay(200);
    }
    noTone(BUZZER);

    Serial.println(F("== Line 6 mắt + PID (Sẵn sàng) =="));
}


void loop() {
    static bool isStarted = false;

    if (!isStarted) {
        if (digitalRead(BUTTON_START) == LOW) {
            isStarted = true;
            tone(BUZZER, 3000, 200);
            Serial.println(F("ĐÃ NHẤN NÚT -> BẮT ĐẦU CHẠY"));
            delay(300);
        } else {
            drive(0, 0);
            delay(100);
            return;
        }
    }

    uint8_t mapNow = readSensors();
    unsigned long now = millis();
    bool seenRecently = (now - lastSeenMillis) <= LOST_ALARM_AFTER_MS;

    if (!seenRecently) {
        if (now - lastBeepMs >= LOST_BEEP_INTERVAL) {
            tone(BUZZER, 2000, LOST_BEEP_DURATION);
            lastBeepMs = now;
        }
    } else {
        noTone(BUZZER);
    }

    if (isCurrentlyLost()) {
        recoverSpin();
        delay(10);
        return;
    }

    if (missionState != STATE_END) { 
      
      if (isIntersection(mapNow)) { 
          
        if (missionState == STATE_CENTER_LINE) {
          goStraightFixed(); 
          Serial.println("STATE 0 -> 1: Ngã Tư Dưới, Đi THẲNG");
          missionState = STATE_FIRST_TURN; 
          return; 
        } 
        
        else if (missionState == STATE_FIRST_TURN) {
          turnRightFixed();
          Serial.println("STATE 1 -> 2: Ngã Tư Trên Lần 1, RẼ PHẢI. Start Loop.");
          missionState = STATE_LOOP; 
          loopIntersectionCount = 0; 
          return; 
        }
        
        else if (missionState == STATE_LOOP) {
          loopIntersectionCount++;
          
          if (loopIntersectionCount == 1) {
            goStraightFixed();
            Serial.println("STATE 2 (Loop 1): Ngã Tư Dưới, Đi THẲNG qua để hoàn thành vòng");
            return;
          } else if (loopIntersectionCount == 2) {
            turnLeftFixed();
            Serial.println("STATE 2 -> 3 (Loop 2): Ngã Tư Trên, RẼ TRÁI (Nhiệm vụ hoàn thành)");
            missionState = STATE_END; 
          }
        }
      }
    } 

    if (missionState == STATE_END) {
        static bool missionReported = false;
        if (!missionReported) {
            tone(BUZZER, 3500, 500);
            Serial.println(F("== NHIỆM VỤ HOÀN THÀNH - TIẾP TỤC DÒ LINE =="));
            missionReported = true;
        }

    }

    int error = computeErrorFromMap(mapNow);

    int P = error;
    sumError += error;
    sumError = constrain(sumError, -5000L, 5000L);
    int D = error - lastError;

    float Kp_use = isCenterBand(mapNow) ? (Kp * Kp_center_scale) : Kp;

    float correctionF = Kp_use * P + Ki * sumError + Kd * D;
    int correction = (int)(correctionF / 100.0f);
    lastError = error;

    // Tính tốc độ 2 bánh
    int left = baseSpeed - correction;
    int right = baseSpeed + correction;

    // Chạy motor
    drive(left, right);

    // Debug
    Serial.print("Err:"); Serial.print(error);
    Serial.print(" Corr:"); Serial.print(correction);
    Serial.print(" L:"); Serial.print(left);
    Serial.print(" R:"); Serial.print(right);
    Serial.print(" CenterBand:"); Serial.println(isCenterBand(mapNow) ? "YES" : "NO");

    delay(10);
}
