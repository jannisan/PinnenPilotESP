#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <MPU9250_asukiaaa.h>
#include <WiFi.h>
#include <Preferences.h>

// --- Globale Variablen & Konstanten ---
// SSID und Passwort für den Access Point
const char* ssid = "ESP32-Access-Point";
const char* password = "123456789";

// Setzt den Webserver-Port auf 80
WiFiServer server(80);

// Variable zum Speichern des HTTP-Requests
String header;

// OLED-Display-Einstellungen
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// I2C-Pins für den ESP32-DevKitC
const uint8_t SDA_PIN = 21;
const uint8_t SCL_PIN = 22;

// Instanz der MPU9250-Klasse
MPU9250_asukiaaa mySensor;

// Motor-Pins für den L298N
const int motor1Pin1 = 27;
const int motor1Pin2 = 26;
const int enable1Pin = 14;

// PWM-Einstellungen für den Motor
const int freq = 10000;
const int pwmChannel = 0;
const int resolution = 8;
const int maxMotorSpeed = 255; // Maximaler PWM-Wert für 8-Bit-Auflösung
const int manMotorSpeed = 120; // PWM-Wert im manuellen Modus 
int motorSpeed = 0;
int yrTarget = 0;


// Autopilot-Statusvariablen
float magneticHeading = 0.0;
float targetHeading = 0.0;
float headingError = 0.0;
float gZ = 0.0;
bool autopilotActive = true;

// Komplementär-Filter
float complementaryHeading = 0.0;
const float ALPHA = 0.98;
const float DT = 200.0 / 1000.0;

// Toleranzen
float HEADING_TOLERANCE = 5.0;
float FINE_CONTROL_THRESHOLD = 15.0;
float MAX_GYRO_RATE = 60.0;


// Zeitsteuerung
unsigned long previousMillis = 0;
const long interval = 200;

// Preferences
Preferences preferences;

// --- NEU: Kompass-Kalibrierungswerte ---
float offsetX = 0, offsetY = 0, offsetZ = 0;
float scaleX = 1, scaleY = 1, scaleZ = 1;

// --- Funktionendeklarationen ---
void setupWiFi();
void readSensors();
void controlMotor();
void updateOLED();
void handleWebServer();
void calibrateCompass();
void stopMotor();
void turnLeft();
void turnRight();
void startMotor(int speed);
void saveCalibration();
void loadCalibration();

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Systemstart...");

  // I2C-Bus
  Wire.begin(SDA_PIN, SCL_PIN);
  mySensor.setWire(&Wire);

  // Sensor-Setup
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();

  // OLED-Setup
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("SSD1306-Initialisierung fehlgeschlagen");
    for (;;);
  }
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.clearDisplay();
  display.println("Starting...");
  display.display();

  // Motor-Setup
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  // Konfiguriert den Pin direkt als PWM-Ausgang
  ledcAttachChannel(enable1Pin, freq, resolution, pwmChannel);

  stopMotor();
  
  // Gespeicherte Kalibrierungswerte laden
  loadCalibration();

  // Initialisiere den stabilisierten Kurs und Sollkurs mit aktuellem Kurs
  readSensors();
  complementaryHeading = magneticHeading;
  targetHeading = complementaryHeading;

  // WiFi-Access-Point starten
  setupWiFi();
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    readSensors();
    if (autopilotActive) {
      controlMotor();
    }
    updateOLED();
  }
  handleWebServer();
}

// --- WiFi ---
void setupWiFi() {
  Serial.print("Access Point wird gestartet...");
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP: ");
  Serial.println(IP);
  server.begin();
}

// --- Preferences ---
void saveCalibration() {
  preferences.begin("compass_cal", false);
  preferences.putFloat("target_h", targetHeading);
  preferences.putFloat("tol", HEADING_TOLERANCE);
  preferences.putFloat("thresh", FINE_CONTROL_THRESHOLD);
  preferences.putFloat("gyro_r", MAX_GYRO_RATE);
  preferences.putFloat("offX", offsetX);
  preferences.putFloat("offY", offsetY);
  preferences.putFloat("offZ", offsetZ);
  preferences.putFloat("sclX", scaleX);
  preferences.putFloat("sclY", scaleY);
  preferences.putFloat("sclZ", scaleZ);
  preferences.end();
  Serial.println("Kalibrierungsdaten gespeichert.");
}

void loadCalibration() {
  preferences.begin("compass_cal", true);
  targetHeading = preferences.getFloat("target_h", targetHeading);
  HEADING_TOLERANCE = preferences.getFloat("tol", HEADING_TOLERANCE);
  FINE_CONTROL_THRESHOLD = preferences.getFloat("thresh", FINE_CONTROL_THRESHOLD);
  MAX_GYRO_RATE = preferences.getFloat("gyro_r", MAX_GYRO_RATE);
  offsetX = preferences.getFloat("offX", 0.0);
  offsetY = preferences.getFloat("offY", 0.0);
  offsetZ = preferences.getFloat("offZ", 0.0);
  scaleX = preferences.getFloat("sclX", 1.0);
  scaleY = preferences.getFloat("sclY", 1.0);
  scaleZ = preferences.getFloat("sclZ", 1.0);
  preferences.end();
  Serial.println("Kalibrierungsdaten geladen.");
}

// Kompass-Kalibrierung ---
void calibrateCompass() {


  float minX = 9999, minY = 9999, minZ = 9999;
  float maxX = -9999, maxY = -9999, maxZ = -9999;

  unsigned long startTime = millis();
  while (millis() - startTime < 20000) { // 20 Sekunden
    mySensor.magUpdate();
    float x = mySensor.magX();
    float y = mySensor.magY();
    float z = mySensor.magZ();

    if (x < minX) minX = x;
    if (y < minY) minY = y;
    if (z < minZ) minZ = z;
    if (x > maxX) maxX = x;
    if (y > maxY) maxY = y;
    if (z > maxZ) maxZ = z;

    delay(100);
    long remaining = (20000 - (millis() - startTime)) / 1000;
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Cal...");
    display.setCursor(0, 16);
    display.print("Time: ");
    display.print(remaining);
    display.display();
  }

  // Hard-Iron Offsets
  offsetX = (maxX + minX) / 2.0;
  offsetY = (maxY + minY) / 2.0;
  offsetZ = (maxZ + minZ) / 2.0;

  // Soft-Iron Skalierung
  float avgDelta = ((maxX - minX) + (maxY - minY) + (maxZ - minZ)) / 3.0;
  scaleX = avgDelta / (maxX - minX);
  scaleY = avgDelta / (maxY - minY);
  scaleZ = avgDelta / (maxZ - minZ);

  saveCalibration();

  Serial.println("Kalibrierung abgeschlossen.");
  Serial.printf("Offsets: X=%.2f Y=%.2f Z=%.2f\n", offsetX, offsetY, offsetZ);
  Serial.printf("Scales : X=%.3f Y=%.3f Z=%.3f\n", scaleX, scaleY, scaleZ);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Calibration");
  display.setCursor(0, 16);
  display.println("done!");
  display.display();
  delay(1000);
}

// --- Sensoren ---
void readSensors() {
  mySensor.magUpdate();

  // Rohdaten + Hard/Soft-Iron Korrektur
  float x = (mySensor.magX() - offsetX) * scaleX;
  float y = (mySensor.magY() - offsetY) * scaleY;
  float z = (mySensor.magZ() - offsetZ) * scaleZ;

  magneticHeading = atan2(-x, -y) * 180 / PI;
  if (magneticHeading < 0) {
    magneticHeading += 360;
  }

  // Gyro
  mySensor.gyroUpdate();
  gZ = mySensor.gyroZ();

  // Komplementär-Filter
  float gyroRateTerm = complementaryHeading + gZ * DT;
  float error = magneticHeading - gyroRateTerm;
  if (error > 180) error -= 360;
  if (error < -180) error += 360;

  complementaryHeading = gyroRateTerm + (1.0 - ALPHA) * error;
  if (complementaryHeading >= 360) complementaryHeading -= 360;
  if (complementaryHeading < 0) complementaryHeading += 360;
}

// --- Motorsteuerung ---
void stopMotor() {
  ledcWrite(enable1Pin, 0);
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
}

void turnLeft() {
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  ledcWrite(enable1Pin, 255 - manMotorSpeed);
}

void turnRight() {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  ledcWrite(enable1Pin, manMotorSpeed);
}

void startMotor(int speed) {
  if (yrTarget > gZ) {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    ledcWrite(enable1Pin, speed);
    Serial.print("R");
  } else {
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    ledcWrite(enable1Pin, 255 - speed);
    Serial.print("L");
  }
  Serial.println(speed);
  Serial.println(yrTarget);
}

// --- Autopilot ---
void controlMotor() {
  // P-Regler-Logik für kontinuierliche Korrektur
  headingError = complementaryHeading - targetHeading;
  if (headingError > 180) {
    headingError -= 360;
  }
  if (headingError < -180) {
    headingError += 360;
  }

  if (abs(headingError) > FINE_CONTROL_THRESHOLD) {
    // Aggressive Steuerung (P-Regler) bei großer Abweichung
    motorSpeed = maxMotorSpeed;
    if (headingError > 0) {
      yrTarget = - 5;
      //turnLeft(motorSpeed);
    } else {
      yrTarget = 5;
      //turnRight(motorSpeed);
    }
    startMotor(motorSpeed);
  }
  // Abweichung zwischen 5 und 15 Grad (sanfte Steuerung)
  else if (abs(headingError) > HEADING_TOLERANCE) {
    motorSpeed = int(maxMotorSpeed / 2);
    if (headingError > 0) {
      yrTarget = - 2;
      //turnLeft(motorSpeed);
    } else {
      yrTarget = 2;
      //turnRight(motorSpeed);
    }
    startMotor(motorSpeed);
  }
  // Abweichung weniger als 5 Grad (Dämpfung durch Gyro)
  else {
    // Gyroskop-basierte Dämpfung, um ein Überschwingen zu verhindern
    // Steuerung basierend auf der Drehrate gZ
    yrTarget = 0;
    if (abs(gZ) > 1.5) { // Kleiner Schwellenwert, um Rauschen zu ignorieren
      motorSpeed = int(maxMotorSpeed / 2);
      startMotor(motorSpeed);
    }
    else {
      stopMotor();
    }
  }
}

// --- OLED ---
void updateOLED() {
  char chStr[4];
  char thStr[4];
  char gzStr[4];

  sprintf(chStr, "%03d", (int)complementaryHeading);
  sprintf(thStr, "%03d", (int)targetHeading);
  sprintf(gzStr, "%02d", (int)gZ);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("MH");
  display.print(chStr);
  display.println(autopilotActive ? " HDG" : " MAN");
  display.setCursor(0, 16);
  display.print("MC");
  display.print(thStr);
  display.print(" gZ");
  display.println(gzStr);
  display.display();
}

// --- Webserver ---
void handleWebServer() {
  WiFiClient client = server.available();
  if (!client) {
    return;
  }

  String requestLine = "";
  String header = "";
  String postData = "";
  bool isPost = false;
  int contentLength = 0;

  // Lese die erste Zeile der Anfrage
  requestLine = client.readStringUntil('\n');

  // Lese restliche Header und suche nach Content-Length
  while (client.connected() && client.available()) {
    String line = client.readStringUntil('\n');
    header += line;
    if (line.startsWith("Content-Length: ")) {
      contentLength = line.substring(16).toInt();
    }
    if (line.length() == 1 && line.charAt(0) == '\r') {
      // End of headers
      break;
    }
  }

  // Lese den POST-Body, falls es sich um einen POST-Request handelt
  if (requestLine.startsWith("POST") && contentLength > 0) {
    isPost = true;
    char postBuffer[contentLength + 1];
    client.readBytes(postBuffer, contentLength);
    postBuffer[contentLength] = '\0';
    postData = String(postBuffer);

    // Parsen der POST-Daten
    int toleranceIndex = postData.indexOf("tolerance=");
    if (toleranceIndex != -1) {
      String toleranceStr = postData.substring(toleranceIndex + 10);
      int andIndex = toleranceStr.indexOf('&');
      if (andIndex != -1) {
        toleranceStr = toleranceStr.substring(0, andIndex);
      }
      HEADING_TOLERANCE = toleranceStr.toFloat();
    }

    int thresholdIndex = postData.indexOf("threshold=");
    if (thresholdIndex != -1) {
      String thresholdStr = postData.substring(thresholdIndex + 10);
      int andIndex = thresholdStr.indexOf('&');
      if (andIndex != -1) {
        thresholdStr = thresholdStr.substring(0, andIndex);
      }
      FINE_CONTROL_THRESHOLD = thresholdStr.toFloat();
    }

    int gyroRateIndex = postData.indexOf("gyro_rate=");
    if (gyroRateIndex != -1) {
      String gyroRateStr = postData.substring(gyroRateIndex + 10);
      MAX_GYRO_RATE = gyroRateStr.toFloat();
    }

    saveCalibration();
  }

  // --- Verarbeite die Befehle basierend auf der requestLine ---
  bool commandExecuted = false;

  // Manuelle Buttons (nur im AP off-Modus)
  if (!autopilotActive) {
    if (requestLine.indexOf("GET /left") >= 0) {
      turnLeft();
      commandExecuted = true;
    } else if (requestLine.indexOf("GET /stop") >= 0) {
      stopMotor();
      commandExecuted = true;
    } else if (requestLine.indexOf("GET /right") >= 0) {
      turnRight();
      commandExecuted = true;
    }
  }

  // AP-Steuerung
  if (requestLine.indexOf("GET /plus") >= 0) {
    targetHeading = (targetHeading + 10);
    commandExecuted = true;
  } else if (requestLine.indexOf("GET /minus") >= 0) {
    targetHeading = (targetHeading - 10);
    commandExecuted = true;
  } else if (requestLine.indexOf("GET /sync") >= 0) {
    targetHeading = complementaryHeading; // Verwende stabilisierten Kurs zum Synchronisieren
    commandExecuted = true;
  } else if (requestLine.indexOf("GET /toggle") >= 0) {
    autopilotActive = !autopilotActive;
    if (!autopilotActive) {
      stopMotor();
    }
    commandExecuted = true;
  } else if (requestLine.indexOf("GET /calibrate") >= 0) {
    calibrateCompass();
    commandExecuted = true;
  }

  // Wenn ein Befehl ausgeführt wurde oder POST-Daten empfangen wurden, umleiten
  if (commandExecuted || isPost) {
    client.println("HTTP/1.1 302 Found");
    client.println("Location: /");
    client.println("Connection: close");
    client.println();
    client.stop();
    return;
  }

  // Kurs normalisieren
  if (targetHeading >= 360) targetHeading -= 360;
  if (targetHeading < 0) targetHeading += 360;

  // --- Hier wird die Webseite generiert, abhängig von der URL ---

  // Sende HTTP-Header
  client.println("HTTP/1.1 200 OK");
  client.println("Content-type:text/html");
  client.println("Connection: close");
  client.println();

  // HTML für die Settings-Seite
  if (requestLine.indexOf("GET /settings") >= 0) {
    // KEIN REFRESH-TAG
    client.println("<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1'><style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;} body { margin: 0; padding: 20px;} .button { border: none; color: white; padding: 16px 40px; text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer; border-radius: 8px;} .button-container { display: flex; justify-content: center; gap: 10px;}.green-button {background-color: #4CAF50;}.red-button {background-color: #f44336;}.status-text {font-size: 1.5em;} input[type=number] { width: 80px; padding: 5px; margin: 5px; font-size: 1em;}.form-container {display: flex; flex-direction: column; align-items: center; gap: 10px; border: 2px solid #ccc; padding: 10px; border-radius: 8px;}</style></head><body>");

    client.println("<h2>set PID-parameters</h2>");
    client.println("<form action='/' method='POST' class='form-container'>");
    client.println("<label for='tolerance'>MH-tolerance (&deg;):</label>");
    client.println("<input type='number' step='0.1' name='tolerance' value='" + String(HEADING_TOLERANCE) + "' required>");
    client.println("<label for='threshold'>threshold finecontrol (&deg;):</label>");
    client.println("<input type='number' step='0.1' name='threshold' value='" + String(FINE_CONTROL_THRESHOLD) + "' required>");
    client.println("<label for='gyro_rate'>max. gyrorate (&deg;/s):</label>");
    client.println("<input type='number' step='0.1' name='gyro_rate' value='" + String(MAX_GYRO_RATE) + "' required>");
    client.println("<button type='submit' class='button green-button'>save</button>");
    client.println("</form>");

    client.println("<p class='button-container' style='margin-top: 20px;'>");
    client.println("<a href='/calibrate'><button class='button green-button'>Calibrate Compass</button></a>");
    client.println("<a href='/'><button class='button green-button'>back</button></a>");
    client.println("</p>");

    client.println("</body></html>");
    client.println();
    client.stop();
  }

  // HTML für die Hauptseite
  else {
    // Bestimmt die CSS-Klasse für den AP-Button basierend auf dem Status
    String apButtonClass = autopilotActive ? "green-button" : "red-button";

    // Formatiert die Kurswerte als dreistellige Strings mit führenden Nullen
    char chStr[4];
    char thStr[4];
    sprintf(chStr, "%03d", (int)complementaryHeading);
    sprintf(thStr, "%03d", (int)targetHeading);

    // HIER WIRD DER REFRESH-TAG EINGEFÜGT
    client.println("<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-s cal e=1'><meta http-equiv='refresh' content='5'><style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;} body { margin: 0; padding: 20px;} .button { border: none; color: white; padding: 16px 40px; text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer; border-radius: 8px;} .button-container { display: flex; justify-content: center; gap: 10px;}.green-button {background-color: #4CAF50;}.red-button {background-color: #f44336;}.status-text {font-size: 1.5em;} input[type=number] { width: 80px; padding: 5px; margin: 5px; font-size: 1em;}.form-container {display: flex; flex-direction: column; align-items: center; gap: 10px; border: 2px solid #ccc; padding: 10px; border-radius: 8px;}</style></head><body>");

    // Anzeige des Status in fünf separaten Zeilen
    client.println("<p style='font-size: 2em; font-weight: bold;'>MH: " + String(chStr) + "</p>");
    client.println("<p style='font-size: 2em; font-weight: bold;'>steer: " + String(thStr) + "</p>");
    client.println("<p style='font-size: 2em; font-weight: bold;'>AP: " + String(autopilotActive ? "ON" : "OFF") + "</p>");
    client.println("<p class='button-container'>");
    client.println("<a href='/minus'><button class='button green-button'>-10&deg;</button></a>");
    client.println("<a href='/plus'><button class='button green-button'>+10&deg;</button></a>");
    client.println("</p>");
    client.println("<p class='button-container'>");
    client.println("<a href='/sync'><button class='button green-button'>Sync Kurs</button></a>");
    client.println("</p>");
    client.println("<p class='button-container'>");
    client.println("<a href='/toggle'><button class='button " + apButtonClass + "'>AP on/off</button></a>");
    client.println("</p>");

    //Buttons, nur im AP off-Modus anzeigen
    if (!autopilotActive) {
      client.println("<p class='button-container'>");
      client.println("<a href='/left'><button class='button green-button'>L</button></a>");
      client.println("<a href='/stop'><button class='button red-button'>Stop</button></a>");
      client.println("<a href='/right'><button class='button green-button'>R</button></a>");
      client.println("</p>");
    }

    //Settings-Button
    client.println("<hr>");
    client.println("<p class='button-container'>");
    client.println("<a href='/settings'><button class='button green-button'>Settings</button></a>");
    client.println("</p>");

    client.println("</body></html>");
    client.println();
    client.stop();
  }
}
