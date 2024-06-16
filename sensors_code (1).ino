#include "DHT.h"
#include <Servo.h>

#define DHTPIN 2                      // DHT11 data pin
#define DHTTYPE DHT11                 // DHT 11

#define RAIN_SENSOR_PIN 3             // Rainfall sensor pin
#define SOIL_MOISTURE_SENSOR_PIN A0   // Soil moisture sensor analog pin
#define BUZZER_PIN 12                 // Buzzer pin
#define LED_PIN 4                     // Blue LED pin

#define relay_pin 7                   // Relay Pin
#define red_led_pin 9                     // Red LED Pin 
#define green_led_pin 8               // Green LED Pin 

#define tap_servo_pin 10           // Servo motor pin

// Initialize DHT sensor.
DHT dht(DHTPIN, DHTTYPE);
Servo tap_servo;

void setup() {
  Serial.begin(9600);
  dht.begin();   // Initialize DHT sensor
  
  pinMode(BUZZER_PIN, OUTPUT);        // Initialize buzzer pin as output
  pinMode(LED_PIN, OUTPUT);           // Initialize LED pin as output
  pinMode(relay_pin, OUTPUT);
  pinMode(red_led_pin, OUTPUT);
  pinMode(green_led_pin, OUTPUT);
  pinMode(RAIN_SENSOR_PIN, INPUT);    // Initialize rain sensor pin as input

  tap_servo.attach(10);    // Attach the servo motor to the specified pin
}

void loop() {
  static unsigned long lastTime = 0;
  const unsigned long interval = 2000; // Read sensors every 2 seconds

  unsigned long currentTime = millis();
  if (currentTime - lastTime >= interval) {
    lastTime = currentTime;

    // Read temperature and humidity from DHT11 sensor
    float humidity = dht.readHumidity();
    float temperatureC = dht.readTemperature();
    float temperatureF = dht.readTemperature(true);

    // Read soil moisture sensor
    int soilMoisture = analogRead(SOIL_MOISTURE_SENSOR_PIN);
    String soilStatus;

    // Check soil moisture and control the relay, LEDs, and water pump
    if (soilMoisture >= 940) {
      soilStatus = "No moisture, Soil is dry";
      digitalWrite(relay_pin, LOW);   // Turn on relay
      digitalWrite(red_led_pin, HIGH); // Turn on red LED
      digitalWrite(green_led_pin, LOW);// Turn off green LED
    } else if (soilMoisture >= 324 && soilMoisture < 940) {
      soilStatus = "There is some moisture, Soil is medium";
      digitalWrite(relay_pin, HIGH);    // Turn off relay
      digitalWrite(red_led_pin, LOW);  // Turn off red LED
      digitalWrite(green_led_pin, HIGH); // Turn on green LED
    } else {
      soilStatus = "Soil is wet";
      digitalWrite(relay_pin, HIGH);    // Turn off relay
      digitalWrite(red_led_pin, LOW);  // Turn off red LED
      digitalWrite(green_led_pin, HIGH); // Turn on green LED
    }

    // Read rainfall sensor
    int rainfall = digitalRead(RAIN_SENSOR_PIN);

    // Activate buzzer, LED, and servo if rainfall is detected
    if (rainfall == 0) {
      digitalWrite(BUZZER_PIN, HIGH);
      digitalWrite(LED_PIN, HIGH);
      tap_servo.write(0);           // Turn the servo motor to 180 degrees
    } else {
      digitalWrite(BUZZER_PIN, LOW);
      digitalWrite(LED_PIN, LOW);
      tap_servo.write(90);             // Turn the servo motor back to 0 degrees
    }

    // Print sensor readings to serial monitor
    Serial.print("Temperature (C): ");
    Serial.println(temperatureC);
    Serial.print("Temperature (F): ");
    Serial.println(temperatureF);
    Serial.print("Humidity (%): ");
    Serial.println(humidity);
    Serial.print("Soil Moisture: ");
    Serial.print(soilMoisture);
    Serial.print(" (");
    Serial.print(soilStatus);
    Serial.println(")");
    Serial.print("Rainfall: ");
    Serial.println(rainfall);
  }
}
