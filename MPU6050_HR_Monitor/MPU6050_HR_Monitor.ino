#include <rgb_lcd.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

rgb_lcd lcd;  // Initialize LCD
Adafruit_MPU6050 mpu;  // Initialize MPU6050

#define pulsePin 34  // Pin for the heart rate sensor

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Initialize I2C bus
  Wire.begin();
  
  // Initialize LCD
  lcd.begin(16, 2);  // 16x2 LCD screen
  lcd.setRGB(0, 0, 255);  // Set background color (blue)
  lcd.clear();  // Clear LCD
  
  // Display test message on LCD
  lcd.setCursor(0, 0);
  lcd.print("MPU6050 Test");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
  
  delay(2000);  // Wait for 2 seconds
  
  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("MPU6050 not detected");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("MPU6050 Fail");
    while (1);  // Stop execution if MPU6050 is not found
  }
  
  // MPU6050 initialized
  Serial.println("MPU6050 Found!");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("MPU6050 Found!");
  delay(1000);  // Wait for 1 second before moving on
  
  // Set sensor configurations
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  delay(100);  // Wait for setup to finish
}

void loop() {
  // Get sensor readings from MPU6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Read pulse sensor value
  int pulseValue = analogRead(pulsePin);

  // Convert pulseValue to voltage
  float voltage = pulseValue * (5.0 / 1023.0);

  // Calculate heartRate from voltage (adjust this formula as needed)
  int heartRate = (voltage / 3.3) * 675;

  // Display data on LCD
  lcd.clear();
  
  // Display Acceleration X on first line
  lcd.setCursor(0, 0);
  lcd.print("Accel X: ");
  lcd.print(a.acceleration.x, 2);  // Print Acceleration X with 2 decimal places
  lcd.print(" m/s^2");

  // Display Heart Rate on second line
  lcd.setCursor(0, 1);
  lcd.print("HR: ");
  lcd.print(heartRate);  // Display the heart rate

  // Print Acceleration, Rotation, and Heart Rate data to Serial Monitor
  Serial.print("Accel X: ");
  Serial.print(a.acceleration.x);
  Serial.print(" Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(" Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rot X: ");
  Serial.print(g.gyro.x);
  Serial.print(" Y: ");
  Serial.print(g.gyro.y);
  Serial.print(" Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Heart Rate: ");
  Serial.println(heartRate);

  delay(500);  // Wait for 0.5 seconds before updating the display
}
