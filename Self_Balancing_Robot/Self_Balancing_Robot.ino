#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <EEPROM.h>
#include <ESP32Servo.h>
#include <Bluepad32.h> 

#define EEPROM_SIZE 128

// ==========================
// 1. PINOUT (TB6612FNG + QUADRATURE ENCODER)
// ==========================
#define STBY 33      // Standby pin TB6612FNG
#define AIN1 26      // Motor KIRI
#define AIN2 25
#define PWMA 27
#define BIN1 18      // Motor KANAN (wiring terbalik)
#define BIN2 19
#define PWMB 17

// === QUADRATURE ENCODER (4 PIN) ===
#define ENC_L_A 35   // Encoder Left Channel A
#define ENC_L_B 34   // Encoder Left Channel B
#define ENC_R_A 32   // Encoder Right Channel A  
#define ENC_R_B 39   // Encoder Right Channel B

// === SERVO PINS ===
#define SERVO_HITAM 12   // Lift naik/turun
#define SERVO_BIRU 13    // Grip buka/tutup

// ==========================
// 2. SERVO SETTINGS
// ==========================
#define LIFT_DOWN 140    // H_TURUN  
#define LIFT_UP   110    // H_NAIK     
#define GRIP_START 100   // B_BUKA    
#define GRIP_END   140 // B_TUTUP

Servo servoLift;
Servo servoGrip;
int servoSequenceState = 0;
unsigned long servoTimer = 0;

// ==========================
// 3. VARIABLES
// ==========================
Adafruit_MPU6050 mpu;
ControllerPtr myController = nullptr;

// MPU Variables
float rawAngle = 0.0;
float angleOffset = 0.0;
float currentAngle = 0.0;
float gyroRate = 0.0;
float accAngle = 0.0;

// === TUNING PARAMETERS ===
float Kp = 15.0;
float Ki = 0.80;     // Integral untuk menahan beban
float Kd = 0.30;
float min_PWM = 15.0;
float angleSetpoint = -1.5;  // MINUS! (Titik seimbang tanpa beban)

// === LOAD COMPENSATION SETTINGS ===
float loadCompensation = 0.0; // Variabel penyimpan status beban
float BEBAN_OFFSET = -15.0;    // Robot mundur 15.5 derajat saat bawa barang

// === MOTION SETTINGS ===
float MAX_TILT_FORWARD = 6.0;
float MAX_TILT_BACKWARD = 5.0;
float MAX_TURN_SPEED = 30.0;
const int AXIS_DEADZONE = 25;

// PID Variables
float angleError = 0, angleIntegral = 0;
float motorOutput = 0;
float previousError = 0;

// === QUADRATURE ENCODER VARIABLES ===
volatile long encoderLeftCount = 0;
volatile long encoderRightCount = 0;
long lastEncoderLeft = 0;
long lastEncoderRight = 0;
float leftSpeed = 0, rightSpeed = 0;

// Timing
unsigned long lastLoopTime = 0;
unsigned long lastSpeedCalcTime = 0;
const int LOOP_TIME_MS = 20;  // 50Hz

// ==========================
// 4. BLUETOOTH CALLBACKS
// ==========================
void onConnectedController(ControllerPtr ctl) {
    if (myController == nullptr) {
        myController = ctl;
        Serial.println("🎮 Controller CONNECTED");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    if (myController == ctl) {
        myController = nullptr;
        Serial.println("🎮 Controller DISCONNECTED");
    }
}

// ==========================
// 5. QUADRATURE ENCODER INTERRUPTS
// ==========================
void IRAM_ATTR readEncLeft() {
    static uint8_t oldA = 0, oldB = 0;
    uint8_t newA = digitalRead(ENC_L_A);
    uint8_t newB = digitalRead(ENC_L_B);
    
    if (oldA == 0 && newA == 1) {  // Rising edge on A
        if (newB == 0) encoderLeftCount++;   // B low = forward
        else encoderLeftCount--;             // B high = reverse
    }
    else if (oldA == 1 && newA == 0) {  // Falling edge on A
        if (newB == 1) encoderLeftCount++;   // B high = forward
        else encoderLeftCount--;             // B low = reverse
    }
    oldA = newA;
    oldB = newB;
}

void IRAM_ATTR readEncRight() {
    static uint8_t oldA = 0, oldB = 0;
    uint8_t newA = digitalRead(ENC_R_A);
    uint8_t newB = digitalRead(ENC_R_B);
    
    if (oldA == 0 && newA == 1) {  // Rising edge on A
        if (newB == 0) encoderRightCount++;   // B low = forward
        else encoderRightCount--;             // B high = reverse
    }
    else if (oldA == 1 && newA == 0) {  // Falling edge on A
        if (newB == 1) encoderRightCount++;   // B high = forward
        else encoderRightCount--;             // B low = reverse
    }
    oldA = newA;
    oldB = newB;
}

// Calculate wheel speeds
void calculateWheelSpeeds() {
    unsigned long now = millis();
    float dt = (now - lastSpeedCalcTime) / 1000.0;  // in seconds
    
    if (dt > 0.05) {  // Update every 50ms minimum
        noInterrupts();
        long currentLeft = encoderLeftCount;
        long currentRight = encoderRightCount;
        interrupts();
        
        // Calculate pulses per second
        leftSpeed = (currentLeft - lastEncoderLeft) / dt;
        rightSpeed = (currentRight - lastEncoderRight) / dt;
        
        lastEncoderLeft = currentLeft;
        lastEncoderRight = currentRight;
        lastSpeedCalcTime = now;
    }
}

// ==========================
// 6. MOTOR CONTROL FOR TB6612FNG
// ==========================
void setMotorTB6612(float left, float right) {
    // Constrain values
    left = constrain(left, -255, 255);
    right = constrain(right, -255, 255);
    
    // Motor KIRI - TB6612FNG Normal
    if (left >= 0) {  // MAJU
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
    } else {  // MUNDUR
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
        left = -left;
    }
    analogWrite(PWMA, left);
    
    // Motor KANAN - TB6612FNG (Wiring Terbalik)
    if (right >= 0) {  // MAJU
        digitalWrite(BIN1, HIGH);  // DIBALIK
        digitalWrite(BIN2, LOW);   // DIBALIK
    } else {  // MUNDUR
        digitalWrite(BIN1, LOW);   // DIBALIK
        digitalWrite(BIN2, HIGH);  // DIBALIK
        right = -right;
    }
    analogWrite(PWMB, right);
}

void stopMotors() {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
}

// ==========================
// 7. MPU6050 READING
// ==========================
void readMPU6050() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    float dt = 0.02;  // 20ms loop time
    
    // RUMUS dari program TB6612FNG Anda
    gyroRate = g.gyro.y * 57.296;  // rad/s to deg/s
    accAngle = -atan2(a.acceleration.x, a.acceleration.z) * 57.296;  // MINUS!
    
    // Complementary filter (98:2 ratio)
    rawAngle = 0.98 * (rawAngle + gyroRate * dt) + 0.02 * accAngle;
    
    // Apply calibration offset
    currentAngle = rawAngle - angleOffset;
}

// ==========================
// 8. SERVO FUNCTIONS (LOGIKA BARU - BERTAHAP)
// ==========================
void handleServoSequence() {
    if (servoSequenceState == 0) return;
    
    unsigned long now = millis();

    // --- SEQUENCE AMBIL BARANG (PICKUP) ---
    // State 1: Menunggu Grip menutup sempurna (500ms) -> Lalu CONDONG
    if (servoSequenceState == 1) {
        if (now - servoTimer >= 200) { 
            loadCompensation = BEBAN_OFFSET; // AKTIFKAN OFFSET DISINI (Robot mundur)
            Serial.println(">> 2. Robot Condong (Offset Aktif)... Menunggu Lift");
            
            servoTimer = millis();   // Reset timer
            servoSequenceState = 3;  // Pindah ke State 3
        }
    }
    // State 3: Menunggu robot stabil condong (500ms) -> Lalu ANGKAT LIFT
    else if (servoSequenceState == 3) {
        if (now - servoTimer >= 200) { 
            servoLift.write(LIFT_UP);  // ANGKAT LIFT DISINI
            Serial.println(">> 3. Lift NAIK (Selesai)");
            servoSequenceState = 0;    // Sequence selesai
        }
    }

    // --- SEQUENCE TARUH BARANG (DROP) ---
    // State 2: Menunggu Lift turun (500ms) -> Lalu BUKA GRIP
    else if (servoSequenceState == 2) {
        if (now - servoTimer >= 350) {
            servoGrip.write(GRIP_START); // BUKA GRIP DISINI
            Serial.println(">> Grip BUKA (Barang dilepas)");
            servoSequenceState = 0; // Sequence selesai
        }
    }
}

// ==========================
// 9. CONTROLLER INPUT
// ==========================
void processControllerInput() {
    if (!myController || !myController->isConnected()) {
        return;
    }
    
    int axisY = myController->axisY();
    int axisX = myController->axisRX();
    
    // Deadzone
    if (abs(axisY) < AXIS_DEADZONE) axisY = 0;
    if (abs(axisX) < AXIS_DEADZONE) axisX = 0;
    
    // Convert to -1.0 to 1.0
    float throttle = -axisY / 512.0;
    float steering = -axisX / 512.0;
    
    // Apply tilt limits
    float tiltLimit = (throttle > 0) ? MAX_TILT_FORWARD : MAX_TILT_BACKWARD;
    float manualThrottle = throttle * tiltLimit;
    
    // Steering
    float manualSteering = steering * MAX_TURN_SPEED;
    
    // === UPDATE SETPOINT ===
    // loadCompensation sekarang diatur otomatis oleh handleServoSequence
    angleSetpoint = 1.5 + manualThrottle + loadCompensation;
    
    // Store for PID
    motorOutput = manualSteering;
    
    // --- TOMBOL Y: AMBIL BARANG (BERTAHAP) ---
    if (myController->y() && servoSequenceState == 0) {
        // TAHAP 1: Grip Menutup Dulu
        servoGrip.write(GRIP_END); 
        
        // Timer mulai, loadCompensation BELUM diubah
        servoTimer = millis();
        servoSequenceState = 1; 
        
        Serial.println(">> 1. Grip TUTUP... Memulai Sequence Ambil");
    }
    
    // --- TOMBOL X: TARUH BARANG ---
    if (myController->x() && servoSequenceState == 0) {
        // TAHAP 1: Lift Turun Dulu
        servoLift.write(LIFT_DOWN);
        
        // MATIKAN offset beban segera (asumsi barang sudah di lantai)
        loadCompensation = 0.0; 
        
        servoTimer = millis();
        servoSequenceState = 2; 
        
        Serial.println(">> Lift TURUN -> Offset MATI -> Menunggu Grip Buka");
    }
}

// ==========================
// 10. EEPROM FUNCTIONS
// ==========================
void saveToEEPROM() {
    EEPROM.begin(EEPROM_SIZE);
    EEPROM.put(0, Kp);
    EEPROM.put(4, Ki);
    EEPROM.put(8, Kd);
    EEPROM.put(12, angleSetpoint); // Simpan default (-1.5)
    EEPROM.put(16, min_PWM);
    EEPROM.put(20, angleOffset);
    EEPROM.commit();
    Serial.println(">> Parameters SAVED");
}

void loadFromEEPROM() {
    EEPROM.begin(EEPROM_SIZE);
    float test;
    EEPROM.get(0, test);
    
    if (isnan(test)) {
        Serial.println(">> EEPROM kosong, pakai default");
        return;
    }
    
    EEPROM.get(0, Kp);
    EEPROM.get(4, Ki);
    EEPROM.get(8, Kd);
    EEPROM.get(12, angleSetpoint);
    EEPROM.get(16, min_PWM);
    EEPROM.get(20, angleOffset);
    
    // Default jika invalid
    if(isnan(Kp) || Kp <= 0) Kp = 15.0;
    if(isnan(Ki)) Ki = 0.8; 
    if(isnan(Kd)) Kd = 0.3;
    if(isnan(angleSetpoint)) angleSetpoint = -1.5;
    if(isnan(min_PWM) || min_PWM < 10) min_PWM = 15.0;
    
    Serial.println(">> Parameters LOADED");
}

// ==========================
// 11. SERIAL COMMANDS
// ==========================
void handleSerialCommands() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim(); cmd.toUpperCase();
        
        if (cmd.startsWith("KP")) {
            Kp = cmd.substring(2).toFloat();
            Serial.printf("Kp: %.2f\n", Kp);
            saveToEEPROM();
        }
        else if (cmd.startsWith("KI")) {
            Ki = cmd.substring(2).toFloat();
            Serial.printf("Ki: %.4f\n", Ki);
            saveToEEPROM();
        }
        else if (cmd.startsWith("KD")) {
            Kd = cmd.substring(2).toFloat();
            Serial.printf("Kd: %.2f\n", Kd);
            saveToEEPROM();
        }
        else if (cmd.startsWith("SET")) {
            angleSetpoint = cmd.substring(3).toFloat();
            Serial.printf("Setpoint Base: %.2f°\n", angleSetpoint);
            saveToEEPROM();
        }
        else if (cmd.startsWith("OFF")) { 
            BEBAN_OFFSET = cmd.substring(3).toFloat();
            Serial.printf("Beban Offset set to: %.2f°\n", BEBAN_OFFSET);
        }
        else if (cmd == "TEST") {
            Serial.println("Motor Test: Left/Right");
            setMotorTB6612(100, 100); delay(500);
            stopMotors();
        }
        else if (cmd == "CAL") {
            Serial.println("Calibrating... Keep robot UPRIGHT");
            delay(3000);
            float sumAngle = 0;
            for(int i = 0; i < 100; i++) {
                readMPU6050();
                sumAngle += currentAngle;
                delay(20);
            }
            loadCompensation = 0;
            angleOffset = sumAngle / 100.0 - angleSetpoint;
            Serial.printf("Offset: %.2f°\n", angleOffset);
            saveToEEPROM();
        }
        else if (cmd == "INFO") {
            Serial.println("\n=== ROBOT INFO ===");
            Serial.printf("Angle: %.2f° | BaseSet: %.2f° | LoadComp: %.2f°\n", currentAngle, -1.5, loadCompensation);
            Serial.printf("Total Setpoint: %.2f°\n", angleSetpoint);
            Serial.printf("PID: Kp=%.2f, Ki=%.4f, Kd=%.2f\n", Kp, Ki, Kd);
            Serial.println("==================\n");
        }
    }
}

// ==========================
// 12. SETUP FUNCTION
// ==========================
void setup() {
    Serial.begin(115200);
    Serial.println("\n=== TB6612FNG BALANCING ROBOT (DELAYED LIFT LOGIC) ===");
    
    // Initialize I2C
    Wire.begin(21, 22);
    Wire.setClock(400000);
    
    // Load parameters
    loadFromEEPROM();
    
    // Initialize Bluepad32
    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();
    
    // Initialize servos
    servoLift.attach(SERVO_HITAM);
    servoGrip.attach(SERVO_BIRU);
    servoLift.write(LIFT_DOWN);
    servoGrip.write(GRIP_START);
    
    // Initialize motor pins
    pinMode(STBY, OUTPUT);
    digitalWrite(STBY, HIGH);
    
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);
    
    stopMotors();
    
    // Initialize QUADRATURE encoder pins
    pinMode(ENC_L_A, INPUT_PULLUP);
    pinMode(ENC_L_B, INPUT_PULLUP);
    pinMode(ENC_R_A, INPUT_PULLUP);
    pinMode(ENC_R_B, INPUT_PULLUP);
    
    // Attach interrupts
    attachInterrupt(digitalPinToInterrupt(ENC_L_A), readEncLeft, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_R_A), readEncRight, CHANGE);
    
    // Initialize MPU6050
    if (!mpu.begin()) {
        Serial.println("ERROR: MPU6050 not found!");
        while(1);
    }
    
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    
    // Initial calibration
    Serial.println("Calibrating MPU... Keep robot UPRIGHT");
    delay(3000);
    readMPU6050();
    
    Serial.println("\n>> System READY!");
}

// ==========================
// 13. MAIN LOOP
// ==========================
void loop() {
    BP32.update();
    
    unsigned long now = millis();
    
    // Handle inputs
    handleServoSequence();
    processControllerInput();
    handleSerialCommands();
    
    // Calculate wheel speeds
    calculateWheelSpeeds();
    
    // Main control loop (20ms)
    if (now - lastLoopTime >= LOOP_TIME_MS) {
        float dt = 0.02;
        lastLoopTime = now;
        
        // Read MPU
        readMPU6050();
        
        // Safety check
        if (abs(currentAngle - angleSetpoint) > 40) {
            stopMotors();
            angleIntegral = 0;
            return;
        }
        
        // PID Calculation
        angleError = currentAngle - angleSetpoint;
        
        // Integral with anti-windup
        angleIntegral += angleError * dt;
        angleIntegral = constrain(angleIntegral, -100, 100);
        
        // Derivative
        float derivative = (angleError - previousError) / dt;
        previousError = angleError;
        
        // PID Output
        float pidOutput = Kp * angleError + Ki * angleIntegral + Kd * derivative;
        
        // Apply steering
        float leftOutput = pidOutput - motorOutput;
        float rightOutput = pidOutput + motorOutput;
        
        // Add minimum PWM
        if (leftOutput > 0) leftOutput += min_PWM;
        else if (leftOutput < 0) leftOutput -= min_PWM;
        
        if (rightOutput > 0) rightOutput += min_PWM;
        else if (rightOutput < 0) rightOutput -= min_PWM;
        
        // Constrain and set motors
        setMotorTB6612(leftOutput, rightOutput);
    }
    
    delay(1);
}