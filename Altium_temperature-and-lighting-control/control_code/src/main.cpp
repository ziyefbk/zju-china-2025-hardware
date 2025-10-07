#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>

// I2C mode pin definitions (Enable I2C mode when CS is pulled high or floating)
#define BMP_SCL 19   // SCL pin - ESP32-S3 standard SCL pin
#define BMP_SDA 20   // SDA pin - ESP32-S3 standard SDA pin
#define BMP_CS 18    // CS pin - Pull high to enable I2C mode

// OLED display parameters
#define SCREEN_WIDTH 128 // OLED width in pixels
#define SCREEN_HEIGHT 64 // OLED height in pixels
#define OLED_RESET     -1 // Reset pin (-1 means sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C // OLED I2C address (commonly 0x3C or 0x3D)

// Relay control pins
#define RELAY_HEAT 12    // Heating relay pin
#define RELAY_COOL 11    // Cooling relay pin

// Rotary encoder pins
#define ENCODER_RED_CLK 15    // Red control encoder CLK pin
#define ENCODER_RED_DT 16     // Red control encoder DT pin
#define ENCODER_GREEN_CLK 6   // Green control encoder CLK pin
#define ENCODER_GREEN_DT 7    // Green control encoder DT pin
#define ENCODER_BLUE_CLK 17   // Blue control encoder CLK pin
#define ENCODER_BLUE_DT 18    // Blue control encoder DT pin
#define ENCODER_TEMP_CLK 9    // Temperature control encoder CLK pin
#define ENCODER_TEMP_DT 10    // Temperature control encoder DT pin
#define ENCODER_WHITE_CLK 4   // White light control encoder CLK pin
#define ENCODER_WHITE_DT 5    // White light control encoder DT pin

// NeoPixel LED pins
#define NEOPIXEL_PIN_1 35    // NeoPixel data pin 1
#define NEOPIXEL_PIN_2 36    // NeoPixel data pin 2
#define NEOPIXEL_PIN_3 37    // NeoPixel data pin 3
#define NEOPIXEL_PIN_4 38    // NeoPixel data pin 4
#define NEOPIXEL_PIN_5 39    // NeoPixel data pin 5
#define NEOPIXEL_COUNT 20    // Number of LEDs per pin

// PID parameters
#define TARGET_TEMP 25.0     // Target temperature (Celsius)
#define KP 2.0               // Proportional gain
#define KI 0.1               // Integral gain
#define KD 0.5               // Derivative gain
#define PID_MIN -100.0       // Minimum PID output
#define PID_MAX 100.0        // Maximum PID output

// Standard sea level pressure
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_NeoPixel rgb_display_1 = Adafruit_NeoPixel(NEOPIXEL_COUNT, NEOPIXEL_PIN_1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel rgb_display_2 = Adafruit_NeoPixel(NEOPIXEL_COUNT, NEOPIXEL_PIN_2, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel rgb_display_3 = Adafruit_NeoPixel(NEOPIXEL_COUNT, NEOPIXEL_PIN_3, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel rgb_display_4 = Adafruit_NeoPixel(NEOPIXEL_COUNT, NEOPIXEL_PIN_4, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel rgb_display_5 = Adafruit_NeoPixel(NEOPIXEL_COUNT, NEOPIXEL_PIN_5, NEO_GRB + NEO_KHZ800);

// PID variables
float previousError = 0.0;
float integral = 0.0;
unsigned long lastTime = 0;

// Rotary encoder variables
volatile int redEncoderPosition = 0;
volatile int greenEncoderPosition = 0;
volatile int blueEncoderPosition = 0;
volatile int tempEncoderPosition = 0;
volatile int whiteEncoderPosition = 0;

int lastRedEncoderPosition = 0;
int lastGreenEncoderPosition = 0;
int lastBlueEncoderPosition = 0;
int lastTempEncoderPosition = 0;
int lastWhiteEncoderPosition = 0;

int redValue = 0;     // 红色值 (0-255)
int greenValue = 0;   // 绿色值 (0-255)
int blueValue = 0;    // 蓝色值 (0-255)
int whiteValue = 0;   // 白光亮度值 (0-255)
float targetTemp = TARGET_TEMP;  // 可调节的目标温度

// Encoder state variables
bool redClkState, lastRedClkState;
bool greenClkState, lastGreenClkState;
bool blueClkState, lastBlueClkState;
bool tempClkState, lastTempClkState;
bool whiteClkState, lastWhiteClkState;
bool neoPixelEnabled = true;  // NeoPixel开关状态

// Red encoder interrupt handler
void IRAM_ATTR redEncoderISR() {
  redClkState = digitalRead(ENCODER_RED_CLK);
  
  if (redClkState != lastRedClkState && redClkState == LOW) {
    if (digitalRead(ENCODER_RED_DT) == HIGH) {
      redEncoderPosition++;  // Clockwise rotation
    } else {
      redEncoderPosition--;  // Counterclockwise rotation
    }
  }
  lastRedClkState = redClkState;
}

// Green encoder interrupt handler
void IRAM_ATTR greenEncoderISR() {
  greenClkState = digitalRead(ENCODER_GREEN_CLK);
  
  if (greenClkState != lastGreenClkState && greenClkState == LOW) {
    if (digitalRead(ENCODER_GREEN_DT) == HIGH) {
      greenEncoderPosition++;  // Clockwise rotation
    } else {
      greenEncoderPosition--;  // Counterclockwise rotation
    }
  }
  lastGreenClkState = greenClkState;
}

// Blue encoder interrupt handler (Encoder 4 controls blue)
void IRAM_ATTR blueEncoderISR() {
  blueClkState = digitalRead(ENCODER_BLUE_CLK);
  
  if (blueClkState != lastBlueClkState && blueClkState == LOW) {
    if (digitalRead(ENCODER_BLUE_DT) == HIGH) {
      blueEncoderPosition++;  // Clockwise rotation
    } else {
      blueEncoderPosition--;  // Counterclockwise rotation
    }
  }
  lastBlueClkState = blueClkState;
}

// Temperature encoder interrupt handler (restores temperature control)
void IRAM_ATTR tempEncoderISR() {
  tempClkState = digitalRead(ENCODER_TEMP_CLK);
  
  if (tempClkState != lastTempClkState && tempClkState == LOW) {
    if (digitalRead(ENCODER_TEMP_DT) == HIGH) {
      tempEncoderPosition++;  // Clockwise rotation
    } else {
      tempEncoderPosition--;  // Counterclockwise rotation
    }
  }
  lastTempClkState = tempClkState;
}

// White encoder interrupt handler
void IRAM_ATTR whiteEncoderISR() {
  whiteClkState = digitalRead(ENCODER_WHITE_CLK);
  
  if (whiteClkState != lastWhiteClkState && whiteClkState == LOW) {
    if (digitalRead(ENCODER_WHITE_DT) == HIGH) {
      whiteEncoderPosition++;  // Clockwise rotation
    } else {
      whiteEncoderPosition--;  // Counterclockwise rotation
    }
  }
  lastWhiteClkState = whiteClkState;
}

// Read encoder positions and update RGB and temperature values
void updateEncoderValues() {
  // 更新红色值
  if (redEncoderPosition != lastRedEncoderPosition) {
    int delta = redEncoderPosition - lastRedEncoderPosition;
  redValue += delta * 5;  // Each step increases/decreases by 5 units
    if (redValue > 255) redValue = 255;
    if (redValue < 0) redValue = 0;
    lastRedEncoderPosition = redEncoderPosition;
    
    Serial.print("红色值: ");
    Serial.println(redValue);
  }
  
  // 更新绿色值
  if (greenEncoderPosition != lastGreenEncoderPosition) {
    int delta = greenEncoderPosition - lastGreenEncoderPosition;
  greenValue += delta * 5;  // Each step increases/decreases by 5 units
    if (greenValue > 255) greenValue = 255;
    if (greenValue < 0) greenValue = 0;
    lastGreenEncoderPosition = greenEncoderPosition;
    
    Serial.print("绿色值: ");
    Serial.println(greenValue);
  }
  
  // 更新蓝色值 (4号编码器控制蓝色)
  if (blueEncoderPosition != lastBlueEncoderPosition) {
    int delta = blueEncoderPosition - lastBlueEncoderPosition;
  blueValue += delta * 5;  // Each step increases/decreases by 5 units
    if (blueValue > 255) blueValue = 255;
    if (blueValue < 0) blueValue = 0;
    lastBlueEncoderPosition = blueEncoderPosition;
    
    Serial.print("蓝色值: ");
    Serial.println(blueValue);
  }
  
  // 更新温度值 (恢复温度控制)
  if (tempEncoderPosition != lastTempEncoderPosition) {
    int delta = tempEncoderPosition - lastTempEncoderPosition;
  targetTemp += delta * 0.5;  // Each step increases/decreases by 0.5°C
  if (targetTemp > 50.0) targetTemp = 50.0;  // Max 50°C
  if (targetTemp < 0.0) targetTemp = 0.0;    // Min 0°C
    lastTempEncoderPosition = tempEncoderPosition;
    
    Serial.print("目标温度: ");
    Serial.print(targetTemp);
    Serial.println("°C");
  }
  
  // 更新白光亮度 (新的白光编码器)
  if (whiteEncoderPosition != lastWhiteEncoderPosition) {
    int delta = whiteEncoderPosition - lastWhiteEncoderPosition;
  whiteValue += delta * 5;  // Each step increases/decreases by 5 units
    if (whiteValue > 255) whiteValue = 255;
    if (whiteValue < 0) whiteValue = 0;
    lastWhiteEncoderPosition = whiteEncoderPosition;
    
    Serial.print("白光亮度: ");
    Serial.println(whiteValue);
  }
}

// PID control function
float calculatePID(float currentTemp, float targetTemp) {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0; // Convert to seconds
  
  if (deltaTime <= 0) return 0; // Avoid division by zero
  
  // Calculate error
  float error = targetTemp - currentTemp;
  
  // Proportional term
  float proportional = KP * error;
  
  // Integral term (prevent integral windup)
  integral += error * deltaTime;
  if (integral > PID_MAX / KI) integral = PID_MAX / KI;
  if (integral < PID_MIN / KI) integral = PID_MIN / KI;
  float integralTerm = KI * integral;
  
  // Derivative term
  float derivative = (error - previousError) / deltaTime;
  float derivativeTerm = KD * derivative;
  
  // PID output
  float output = proportional + integralTerm + derivativeTerm;
  
  // Limit output range
  if (output > PID_MAX) output = PID_MAX;
  if (output < PID_MIN) output = PID_MIN;
  
  // Update variables
  previousError = error;
  lastTime = currentTime;
  
  return output;
}

// Relay control function
void controlRelays(float pidOutput) {
  // Turn off all relays
  digitalWrite(RELAY_HEAT, LOW);
  digitalWrite(RELAY_COOL, LOW);
  
  if (pidOutput > 10.0) {
    // Need heating
    digitalWrite(RELAY_HEAT, HIGH);
    Serial.print("Heating ON, PID output: ");
    Serial.println(pidOutput);
  } else if (pidOutput < -10.0) {
    // Need cooling
    digitalWrite(RELAY_COOL, HIGH);
    Serial.print("Cooling ON, PID output: ");
    Serial.println(pidOutput);
  } else {
    // Temperature close to target, keep off
    Serial.print("Temperature stable, PID output: ");
    Serial.println(pidOutput);
  }
}

// NeoPixel control function
void updateNeoPixels() {
  // If NeoPixel is disabled, clear all LEDs and return
  if (!neoPixelEnabled) {
    rgb_display_1.clear();
    rgb_display_2.clear();
    rgb_display_3.clear();
    rgb_display_4.clear();
    rgb_display_5.clear();
    rgb_display_1.show();
    rgb_display_2.show();
    rgb_display_3.show();
    rgb_display_4.show();
    rgb_display_5.show();
    return;
  }
  
  // RGB color (for LED strips 1, 3, 5)
  uint32_t rgbColor = rgb_display_1.Color(redValue, greenValue, blueValue);
  
  // White color (for LED strips 2, 4)
  uint32_t whiteColor = rgb_display_2.Color(whiteValue, whiteValue, whiteValue);
  
  // Set LED strip 1 to RGB color
  for (int i = 0; i < NEOPIXEL_COUNT; i++) {
    rgb_display_1.setPixelColor(i, rgbColor);
  }
  
  // Set LED strip 2 to white
  for (int i = 0; i < NEOPIXEL_COUNT; i++) {
    rgb_display_2.setPixelColor(i, whiteColor);
  }
  
  // Set LED strip 3 to RGB color
  for (int i = 0; i < NEOPIXEL_COUNT; i++) {
    rgb_display_3.setPixelColor(i, rgbColor);
  }
  
  // Set LED strip 4 to white
  for (int i = 0; i < NEOPIXEL_COUNT; i++) {
    rgb_display_4.setPixelColor(i, whiteColor);
  }
  
  // Set LED strip 5 to RGB color
  for (int i = 0; i < NEOPIXEL_COUNT; i++) {
    rgb_display_5.setPixelColor(i, rgbColor);
  }
  
  // Show all LEDs on all pins
  rgb_display_1.show();
  rgb_display_2.show();
  rgb_display_3.show();
  rgb_display_4.show();
  rgb_display_5.show();
}

// OLED display function
void updateDisplay(float currentTemp, float currentPressure, float pidOutput) {
  display.clearDisplay();
  
  // Set text properties
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  // Line 1: Title (simplified)
  display.setCursor(0, 0);
  display.println("PID Temp Control");
  
  // Line 2: Current and target temperature
  display.setCursor(0, 10);
  display.print("Cur:");
  display.print(currentTemp, 1);
  display.print("C Tar:");
  display.print(targetTemp, 1);
  display.print("C");
  
  // Line 3: Error and PID output
  display.setCursor(0, 20);
  display.print("Err:");
  display.print(targetTemp - currentTemp, 1);
  display.print(" PID:");
  display.print(pidOutput, 1);
  
  // Line 4: RGB values and white value
  display.setCursor(0, 30);
  display.print("RGB:");
  display.print(redValue);
  display.print(",");
  display.print(greenValue);
  display.print(",");
  display.print(blueValue);
  
  // Line 5: White brightness and relay status
  display.setCursor(0, 40);
  display.print("White:");
  display.print(whiteValue);
  display.print(" ");
  if (digitalRead(RELAY_HEAT)) {
    display.print("HEAT");
  } else if (digitalRead(RELAY_COOL)) {
    display.print("COOL");
  } else {
    display.print("STBL");
  }
  
  // Line 6: NeoPixel status
  display.setCursor(0, 50);
  display.print("LED:");
  display.print(neoPixelEnabled ? "ON" : "OFF");
  
  display.display();
}

void setup() {
  Serial.begin(115200);
  delay(2000);  // 等待2秒让串口稳定
  Serial.println("=== ESP32-S3 PID温度控制系统启动 ===");
  
  // 初始化继电器引脚
  pinMode(RELAY_HEAT, OUTPUT);
  pinMode(RELAY_COOL, OUTPUT);
  digitalWrite(RELAY_HEAT, LOW);  // 初始关闭制热
  digitalWrite(RELAY_COOL, LOW);  // 初始关闭制冷
  
  // 初始化旋转编码器引脚
  pinMode(ENCODER_RED_CLK, INPUT_PULLUP);
  pinMode(ENCODER_RED_DT, INPUT_PULLUP);
  pinMode(ENCODER_GREEN_CLK, INPUT_PULLUP);
  pinMode(ENCODER_GREEN_DT, INPUT_PULLUP);
  pinMode(ENCODER_BLUE_CLK, INPUT_PULLUP);
  pinMode(ENCODER_BLUE_DT, INPUT_PULLUP);
  pinMode(ENCODER_TEMP_CLK, INPUT_PULLUP);
  pinMode(ENCODER_TEMP_DT, INPUT_PULLUP);
  pinMode(ENCODER_WHITE_CLK, INPUT_PULLUP);
  pinMode(ENCODER_WHITE_DT, INPUT_PULLUP);
  
  // 读取初始状态
  lastRedClkState = digitalRead(ENCODER_RED_CLK);
  lastGreenClkState = digitalRead(ENCODER_GREEN_CLK);
  lastBlueClkState = digitalRead(ENCODER_BLUE_CLK);
  lastTempClkState = digitalRead(ENCODER_TEMP_CLK);
  lastWhiteClkState = digitalRead(ENCODER_WHITE_CLK);
  
  // 设置中断
  attachInterrupt(digitalPinToInterrupt(ENCODER_RED_CLK), redEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_GREEN_CLK), greenEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_BLUE_CLK), blueEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_TEMP_CLK), tempEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_WHITE_CLK), whiteEncoderISR, CHANGE);
  
  // 初始化所有NeoPixel
  rgb_display_1.begin();
  rgb_display_2.begin();
  rgb_display_3.begin();
  rgb_display_4.begin();
  rgb_display_5.begin();
  rgb_display_1.clear();
  rgb_display_2.clear();
  rgb_display_3.clear();
  rgb_display_4.clear();
  rgb_display_5.clear();
  rgb_display_1.show();
  rgb_display_2.show();
  rgb_display_3.show();
  rgb_display_4.show();
  rgb_display_5.show();
  redValue = 50;    // 初始红色值
  greenValue = 50;  // 初始绿色值
  blueValue = 50;   // 初始蓝色值
  whiteValue = 100; // 初始白光亮度值
  
  Serial.println("继电器和NeoPixel初始化完成");
  Serial.print("初始RGB值: "); 
  Serial.print(redValue); Serial.print(",");
  Serial.print(greenValue); Serial.print(",");
  Serial.println(blueValue);
  Serial.print("初始白光亮度: "); Serial.println(whiteValue);
  Serial.print("制热继电器: GPIO"); Serial.println(RELAY_HEAT);
  Serial.print("制冷继电器: GPIO"); Serial.println(RELAY_COOL);
  Serial.print("NeoPixel引脚: GPIO"); Serial.print(NEOPIXEL_PIN_1); Serial.print(","); Serial.print(NEOPIXEL_PIN_2); Serial.print(","); Serial.print(NEOPIXEL_PIN_3); Serial.print(","); Serial.print(NEOPIXEL_PIN_4); Serial.print(","); Serial.println(NEOPIXEL_PIN_5);
  Serial.print("红色编码器: GPIO"); Serial.print(ENCODER_RED_CLK); Serial.print(","); Serial.println(ENCODER_RED_DT);
  Serial.print("绿色编码器: GPIO"); Serial.print(ENCODER_GREEN_CLK); Serial.print(","); Serial.println(ENCODER_GREEN_DT);
  Serial.print("蓝色编码器4: GPIO"); Serial.print(ENCODER_BLUE_CLK); Serial.print(","); Serial.println(ENCODER_BLUE_DT);
  Serial.print("温度编码器: GPIO"); Serial.print(ENCODER_TEMP_CLK); Serial.print(","); Serial.println(ENCODER_TEMP_DT);
  Serial.print("白光编码器: GPIO"); Serial.print(ENCODER_WHITE_CLK); Serial.print(","); Serial.println(ENCODER_WHITE_DT);
  Serial.println("编码器D引脚: 已接地");
  Serial.print("目标温度: "); Serial.print(TARGET_TEMP); Serial.println("°C");
  
  // 将CS引脚拉高以启用I2C模式
  pinMode(BMP_CS, OUTPUT);
  digitalWrite(BMP_CS, HIGH);
  delay(100);
  
  Serial.println("I2C模式引脚配置:");
  Serial.print("SCL: GPIO"); Serial.println(BMP_SCL);
  Serial.print("SDA: GPIO"); Serial.println(BMP_SDA);
  Serial.print("CS: GPIO"); Serial.print(BMP_CS); Serial.println(" (拉高启用I2C)");
  
  // 初始化I2C，指定自定义引脚
  Wire.begin(BMP_SDA, BMP_SCL);
  
  // 初始化OLED显示屏
  Serial.println("初始化OLED显示屏...");
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    // 继续运行，即使OLED初始化失败
  } else {
    Serial.println("OLED显示屏初始化成功!");
    
    // Show startup screen
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("PID Temperature");
    display.println("Control System");
    display.println("");
    display.println("Initializing...");
    display.display();
    delay(2000);
  }
  
  Serial.println("Initializing BMP3XX sensor in I2C mode...");

  // Try different I2C addresses
  Serial.println("尝试默认I2C地址 0x77...");
  if (!bmp.begin_I2C(0x77, &Wire)) {   
    Serial.println("地址0x77失败，尝试地址0x76...");
    if (!bmp.begin_I2C(0x76, &Wire)) {
      Serial.println("Could not find a valid BMP3 sensor, check wiring!");
      Serial.println("程序将继续尝试连接...");
      for(int i = 0; i < 10; i++) {
        delay(1000);
        Serial.print("传感器连接失败，尝试次数: ");
        Serial.println(i + 1);
      }
      Serial.println("继续运行程序用于测试...");
      return;
    } else {
      Serial.println("在地址0x76找到传感器!");
    }
  } else {
    Serial.println("在地址0x77找到传感器!");
  }

  Serial.println("BMP3XX sensor found and initialized successfully!");
  Serial.println("等待传感器稳定...");
  delay(1000);
  
  Serial.println("Configuring sensor settings...");

  // Use basic configuration
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_6_25_HZ);  // 进一步降低数据率
  
  Serial.println("配置完成，等待传感器准备...");
  delay(2000);  // 给传感器更多时间准备
  
  // Initialize PID timing
  lastTime = millis();
  
  Serial.println("PID温度控制系统启动完成!");
  Serial.println("开始温度控制...");
  Serial.println("=================================");
}

void loop() {
  // Read sensor data
  if (! bmp.performReading()) {
    Serial.println("传感器读取失败，等待重试...");
    delay(2000);
    return;
  }
  
  float currentTemp = bmp.temperature;
  float currentPressure = bmp.pressure / 100.0;
  
  // Calculate PID output
  float pidOutput = calculatePID(currentTemp, targetTemp);
  
  // Control relays
  controlRelays(pidOutput);
  
  // Update rotary encoder values
  updateEncoderValues();
  updateNeoPixels();  // Update NeoPixel display every loop
  
  // Update OLED display
  updateDisplay(currentTemp, currentPressure, pidOutput);
  
  // Print information
  Serial.println("=================================");
  Serial.print("当前温度: "); Serial.print(currentTemp); Serial.println("°C");
  Serial.print("目标温度: "); Serial.print(targetTemp); Serial.println("°C");
  Serial.print("温度误差: "); Serial.print(targetTemp - currentTemp); Serial.println("°C");
  Serial.print("气压: "); Serial.print(currentPressure); Serial.println(" hPa");
  Serial.print("PID输出: "); Serial.println(pidOutput);
  
  // Print RGB and white values
  Serial.print("RGB值: "); 
  Serial.print(redValue); Serial.print(",");
  Serial.print(greenValue); Serial.print(",");
  Serial.println(blueValue);
  Serial.print("白光亮度: "); Serial.println(whiteValue);
  Serial.println("灯带配置: 1,3,5号RGB | 2,4号白光");
  
  // Print relay status
  Serial.print("制热状态: "); Serial.println(digitalRead(RELAY_HEAT) ? "开启" : "关闭");
  Serial.print("制冷状态: "); Serial.println(digitalRead(RELAY_COOL) ? "开启" : "关闭");
  Serial.println("=================================");
  
  delay(2000);  // Update every 2 seconds
}