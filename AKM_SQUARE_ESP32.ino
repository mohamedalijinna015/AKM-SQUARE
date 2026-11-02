/*
 * AKM SQUARE
 *
 * STM32 USART6 TX -> ESP32 GPIO16 RX2
 * STM32 GND       -> ESP32 GND
 *
 * UART: 115200, 8N1
 *
 * Firebase latest data:  /AKM%20SQUARE.json
 * Firebase history:      /AKM%20SQUARE/history.json
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

// ---------- Wi-Fi ----------
#define WIFI_SSID "YOUR_WIFI_NAME"
#define WIFI_PASS "YOUR_WIFI_PASSWORD"

// ---------- Firebase ----------
#define FIREBASE_HOST "akm-square-default-rtdb.firebaseio.com"

// Leave EMPTY if your database rules allow writes (test mode).
// A Web API key (AIza...) is NOT an auth token - do not put it here.
#define FIREBASE_AUTH_TOKEN "AIzaSyBP1UrNYegVvdZoxWrWrdB2F_hdlNI6GvE"

// ---------- UART ----------
#define STM32_RX_PIN 16
#define STM32_TX_PIN 17
#define STM32_BAUD 115200

// ---------- Upload settings ----------
#define UPLOAD_PERIOD_MS 2000UL
#define HISTORY_EVERY 10
#define LINE_MAX_LEN 160

struct SensorData {
  float temperature = 0.0f;
  float current = 0.0f;

  char temperatureStatus[24] = "UNKNOWN";
  char currentStatus[24] = "UNKNOWN";
  char vibration[24] = "UNKNOWN";
  char vibrationNote[32] = "";
  char motor[24] = "UNKNOWN";
  char speed[24] = "UNKNOWN";

  bool temperatureOK = false;
  bool currentOK = false;

  bool gotTemperature = false;
  bool gotCurrent = false;
  bool gotVibration = false;
  bool gotMotor = false;
  bool gotSpeed = false;
};

SensorData incoming;
SensorData latest;

char lineBuf[LINE_MAX_LEN];
size_t lineLen = 0;

bool reportReady = false;
bool haveLatest = false;

unsigned long lastUpload = 0;
unsigned long lastRx = 0;
unsigned long lastWifiTry = 0;
unsigned long lastWarning = 0;

uint32_t uploadCount = 0;

// ---------- Text helpers ----------

void copyTrimmed(char *dst, size_t size, const char *src) {
  if (size == 0) return;

  while (*src == ' ' || *src == '\t') src++;

  size_t n = strlen(src);

  while (n > 0 &&
         (src[n - 1] == ' ' || src[n - 1] == '\r' ||
          src[n - 1] == '\n' || src[n - 1] == '\t')) {
    n--;
  }

  if (n >= size) n = size - 1;

  memcpy(dst, src, n);
  dst[n] = '\0';
}

void resetIncomingFlags() {
  incoming.gotTemperature = false;
  incoming.gotCurrent = false;
  incoming.gotVibration = false;
  incoming.gotMotor = false;
  incoming.gotSpeed = false;
}

// ---------- Numeric parsing ----------

void parseNumeric(const char *value, float &number, bool &valid,
                  char *status, size_t statusSize) {
  while (*value == ' ' || *value == '\t') value++;

  char *end = nullptr;
  float result = strtof(value, &end);

  valid = (end != value);
  if (valid) number = result;

  const char *open = strchr(value, '(');

  if (open) {
    open++;
    const char *close = strchr(open, ')');
    size_t n = close ? (size_t)(close - open) : strlen(open);

    char temp[24];
    if (n >= sizeof(temp)) n = sizeof(temp) - 1;
    memcpy(temp, open, n);
    temp[n] = '\0';

    copyTrimmed(status, statusSize, temp);
  } else if (!valid) {
    copyTrimmed(status, statusSize, "UNKNOWN");
  } else {
    copyTrimmed(status, statusSize, "OK");
  }
}

// ---------- Display ----------

void printReport(const SensorData &d) {
  Serial.println();
  Serial.println("====================================");
  Serial.println("       AKM SQUARE SENSOR MONITOR");
  Serial.println("====================================");

  Serial.printf("WiFi        : %s\n",
                WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");

  if (d.temperatureOK)
    Serial.printf("Temperature : %.2f C (%s)\n", d.temperature, d.temperatureStatus);
  else
    Serial.printf("Temperature : N/A (%s)\n", d.temperatureStatus);

  if (d.currentOK)
    Serial.printf("Current     : %.3f A (%s)\n", d.current, d.currentStatus);
  else
    Serial.printf("Current     : N/A (%s)\n", d.currentStatus);

  if (d.vibrationNote[0])
    Serial.printf("Vibration   : %s (%s)\n", d.vibration, d.vibrationNote);
  else
    Serial.printf("Vibration   : %s\n", d.vibration);

  Serial.printf("Motor       : %s\n", d.motor);
  Serial.printf("Speed       : %s\n", d.speed);
  Serial.println("====================================");
}

// ---------- STM32 UART parsing ----------

void processLine(const char *line) {
  const char *colon = strchr(line, ':');
  if (!colon) return;

  char label[32];
  size_t labelLen = colon - line;
  if (labelLen >= sizeof(label)) labelLen = sizeof(label) - 1;

  memcpy(label, line, labelLen);
  label[labelLen] = '\0';

  char normalized[32];
  copyTrimmed(normalized, sizeof(normalized), label);

  for (size_t i = 0; normalized[i]; i++) {
    normalized[i] = (char)tolower((unsigned char)normalized[i]);
  }

  const char *value = colon + 1;

  if (strcmp(normalized, "temperature") == 0) {
    parseNumeric(value, incoming.temperature, incoming.temperatureOK,
                 incoming.temperatureStatus, sizeof(incoming.temperatureStatus));
    incoming.gotTemperature = true;

  } else if (strcmp(normalized, "current") == 0) {
    parseNumeric(value, incoming.current, incoming.currentOK,
                 incoming.currentStatus, sizeof(incoming.currentStatus));
    incoming.gotCurrent = true;

  } else if (strcmp(normalized, "vibration") == 0) {
    char temp[64];
    copyTrimmed(temp, sizeof(temp), value);

    incoming.vibrationNote[0] = '\0';

    char *paren = strchr(temp, '(');
    if (paren) {
      char *close = strchr(paren, ')');
      if (close) *close = '\0';
      copyTrimmed(incoming.vibrationNote, sizeof(incoming.vibrationNote), paren + 1);
      *paren = '\0';
    }

    copyTrimmed(incoming.vibration, sizeof(incoming.vibration), temp);
    incoming.gotVibration = true;

  } else if (strcmp(normalized, "motor") == 0) {
    copyTrimmed(incoming.motor, sizeof(incoming.motor), value);
    incoming.gotMotor = true;

  } else if (strcmp(normalized, "speed") == 0) {
    copyTrimmed(incoming.speed, sizeof(incoming.speed), value);
    incoming.gotSpeed = true;

    // Speed is the final line in the STM32 report.
    if (incoming.gotTemperature && incoming.gotCurrent &&
        incoming.gotVibration && incoming.gotMotor) {

      latest = incoming;
      reportReady = true;
      haveLatest = true;

      printReport(latest);
    }

    resetIncomingFlags();
  }
}

void readStm32() {
  while (Serial2.available()) {
    char c = (char)Serial2.read();
    lastRx = millis();

    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      if (lineLen > 0) processLine(lineBuf);
      lineLen = 0;

    } else if (c != '\r') {
      if (lineLen < LINE_MAX_LEN - 1) {
        lineBuf[lineLen++] = c;
      } else {
        lineLen = 0;
      }
    }
  }

  if (millis() - lastRx > 5000UL && millis() - lastWarning > 5000UL) {
    Serial.println("[WARN] No STM32 UART data. Check GPIO16 RX, GND and baud.");
    lastWarning = millis();
  }
}

// ---------- JSON helpers ----------

String jsonEscape(const char *input) {
  String out;

  while (*input) {
    char c = *input++;
    if (c == '"' || c == '\\') out += '\\';
    if ((unsigned char)c >= 0x20) out += c;
  }

  return out;
}

String numberOrNull(bool valid, float value, int decimals) {
  if (!valid) return "null";
  return String(value, decimals);
}

String buildJson(const SensorData &d) {
  String json = "{";

  json += "\"temperature\":";
  json += numberOrNull(d.temperatureOK, d.temperature, 2);
  json += ",";

  json += "\"temperature_status\":\"";
  json += jsonEscape(d.temperatureStatus);
  json += "\",";

  json += "\"current\":";
  json += numberOrNull(d.currentOK, d.current, 3);
  json += ",";

  json += "\"current_status\":\"";
  json += jsonEscape(d.currentStatus);
  json += "\",";

  json += "\"vibration\":\"";
  json += jsonEscape(d.vibration);
  json += "\",";

  json += "\"motor\":\"";
  json += jsonEscape(d.motor);
  json += "\",";

  json += "\"speed\":\"";
  json += jsonEscape(d.speed);
  json += "\",";

  json += "\"test\":\"AKM_SQUARE\",";

  json += "\"updated_at_ms\":";
  json += String(millis());

  json += "}";

  return json;
}

// ---------- Firebase REST API ----------

bool firebaseSend(const String &path, bool post, const String &json) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Firebase    : Failed (Wi-Fi disconnected)");
    return false;
  }

  String url = String("https://") + FIREBASE_HOST + "/" + path + ".json";

  if (strlen(FIREBASE_AUTH_TOKEN) > 0) {
    url += "?auth=";
    url += FIREBASE_AUTH_TOKEN;
  }

  WiFiClientSecure client;
  client.setInsecure();  // Testing only.

  HTTPClient http;
  http.setTimeout(10000);

  if (!http.begin(client, url)) {
    Serial.println("Firebase    : Failed (HTTP begin)");
    return false;
  }

  http.addHeader("Content-Type", "application/json");

  int code = post ? http.POST(json) : http.PUT(json);
  String response = http.getString();
  http.end();

  bool ok = (code >= 200 && code < 300);

  if (!ok) {
    Serial.printf("Firebase    : Failed (HTTP %d)\n", code);
    if (response.length() > 0) {
      Serial.print("Response    : ");
      Serial.println(response);
    }
  }

  return ok;
}

// ---------- Wi-Fi ----------

void connectWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("Connecting to Wi-Fi");

  unsigned long start = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000UL) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected");
    Serial.print("IP Address  : ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi not connected. Will retry...");
  }
}

void keepWifiAlive() {
  static bool wasConnected = false;
  bool connected = (WiFi.status() == WL_CONNECTED);

  if (connected && !wasConnected && millis() > 20000UL) {
    Serial.println("WiFi connected");
  }
  wasConnected = connected;

  if (!connected && millis() - lastWifiTry > 10000UL) {
    Serial.println("[Wi-Fi] Reconnecting...");
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    lastWifiTry = millis();
  }
}

// ---------- Upload scheduler ----------

void uploadIfReady() {
  if (!reportReady) return;
  if (WiFi.status() != WL_CONNECTED) return;
  if (millis() - lastUpload < UPLOAD_PERIOD_MS) return;

  String json = buildJson(latest);

  bool success = firebaseSend("AKM%20SQUARE", false, json);

  lastUpload = millis();

  if (!success) return;  // will retry

  reportReady = false;
  uploadCount++;

  Serial.println("Firebase    : Data updated in Firebase");

  if (uploadCount % HISTORY_EVERY == 0) {
    if (firebaseSend("AKM%20SQUARE/history", true, buildJson(latest))) {
      Serial.println("Firebase    : History record saved");
    }
  }
}

// ---------- Setup ----------

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial2.setRxBufferSize(2048);
  Serial2.begin(STM32_BAUD, SERIAL_8N1, STM32_RX_PIN, STM32_TX_PIN);

  Serial.println();
  Serial.println("====================================");
  Serial.println("       AKM SQUARE SENSOR MONITOR");
  Serial.println("====================================");

  lastRx = millis();

  connectWifi();
}

// ---------- Main loop ----------

void loop() {
  readStm32();
  keepWifiAlive();
  uploadIfReady();
}
