/*
  SMART FRIDGE COMPARTMENT CONTROLLER
  -------------------------------------------------
  - ESP32 reads DHT11 (temp + humidity) and MQ-2/MQ-135 (gas, via voltage divider)
  - Controls compressor via relay on the AC mains LIVE line
  - Auto cutoff when fridge reaches target temperature (or gas threshold exceeded)
  - Manual ON/OFF override from phone browser
  - Live readings visible on a web page hosted by the ESP32 itself (no app needed)

  WIRING (matches what we discussed):
    DHT11 DATA  -> GPIO 4
    MQ sensor   -> AOUT through 10k/20k divider -> GPIO 34
    Relay IN    -> GPIO 26
    Relay VCC   -> 5V, GND -> GND
    DHT11 VCC   -> 3.3V, GND -> GND
    MQ VCC      -> 5V, GND -> GND

  LOGIC:
    - If AUTO mode: relay stays ON (compressor running) while temp > target.
      When temp <= target, relay turns OFF (compressor stops / "trips off").
      It turns back ON once temp rises above (target + hysteresis) again.
    - If gas level crosses the danger threshold, relay is forced OFF immediately
      regardless of mode, as a safety cutoff.
    - MANUAL mode: phone fully controls relay ON/OFF, ignoring temperature logic.
*/

#include <WiFi.h>
#include <WebServer.h>
#include <DHT.h>

// ---------- USER CONFIG ----------
const char* WIFI_SSID     = "Engr.Yinka";
const char* WIFI_PASSWORD = "olayinka";

float targetTempC   = 4.0;   // compressor stops at/below this temp
float hysteresisC   = 1.5;   // compressor restarts at targetTempC + hysteresisC
int   gasDangerLevel = 5000; // raw ADC threshold (0-4095) - TUNE after testing in clean air

// ---------- AI INFERENCE ENGINE CONFIG ----------
// Rule-based reasoning layer: combines temp trend + gas trend into a single
// risk score, with predictive early-warning before hard thresholds are hit.
int   gasWarnLevel   = (int)(gasDangerLevel * 0.7); // early-warning band starts here
const int TREND_WINDOW = 5;       // number of past readings used to estimate trend
float tempHistory[TREND_WINDOW];
int   gasHistory[TREND_WINDOW];
int   historyCount = 0;
int   historyIndex = 0;
float tempTrendPerMin = 0;   // estimated rate of change, deg C per minute
float gasTrendPerMin  = 0;   // estimated rate of change, raw ADC units per minute
int   riskScore       = 0;   // 0-100 combined risk score
String riskLabel      = "NORMAL";
String aiReasoning    = "Initializing model...";

// ---------- PIN CONFIG ----------
#define DHTPIN    4
#define DHTTYPE   DHT11
#define GASPIN    34
#define RELAYPIN  26

DHT dht(DHTPIN, DHTTYPE);
WebServer server(80);

// ---------- STATE ----------
bool relayState   = false;   // true = relay energized = compressor ON
bool autoMode     = true;    // true = temperature logic controls relay
float currentTemp = 0;
float currentHumidity = 0;
int   currentGas  = 0;
bool  gasAlert    = false;
unsigned long lastSensorRead = 0;
const unsigned long SENSOR_INTERVAL = 2000; // read sensors every 2s

// ---------- RELAY HELPERS ----------
void setRelay(bool on) {
  relayState = on;
  digitalWrite(RELAYPIN, on ? HIGH : LOW);
}

// ---------- AI INFERENCE ENGINE ----------
// Updates rolling history, estimates trends (rate of change per minute),
// then fuses temp + gas trend into a risk score and human-readable reasoning.
// This is a transparent rule-based model (not a trained neural net) - 
// it reasons over recent sensor history rather than just the instant reading.
void updateAIInference(float temp, int gas) {
  // store into circular buffer
  tempHistory[historyIndex] = temp;
  gasHistory[historyIndex]  = gas;
  historyIndex = (historyIndex + 1) % TREND_WINDOW;
  if (historyCount < TREND_WINDOW) historyCount++;

  if (historyCount >= 2) {
    // oldest sample in buffer vs newest sample
    int oldestIdx = (historyIndex + TREND_WINDOW - historyCount) % TREND_WINDOW;
    int newestIdx = (historyIndex + TREND_WINDOW - 1) % TREND_WINDOW;

    float tempDelta = tempHistory[newestIdx] - tempHistory[oldestIdx];
    float gasDelta  = gasHistory[newestIdx]  - gasHistory[oldestIdx];
    float elapsedMin = (historyCount - 1) * (SENSOR_INTERVAL / 1000.0) / 60.0;

    if (elapsedMin > 0) {
      tempTrendPerMin = tempDelta / elapsedMin;
      gasTrendPerMin  = gasDelta / elapsedMin;
    }
  }

  // ---- Risk scoring (0-100), combining temp deviation + gas level + trends ----
  float tempDeviation = temp - targetTempC; // positive = too warm
  float tempScore = constrain(tempDeviation * 12.0, 0, 50);       // up to 50 pts
  float gasScore  = constrain(((float)gas / gasDangerLevel) * 40.0, 0, 40); // up to 40 pts
  float trendScore = 0;
  if (tempTrendPerMin > 0.3)  trendScore += 5;  // warming fast
  if (gasTrendPerMin > 50)    trendScore += 5;  // gas rising fast
  trendScore = constrain(trendScore, 0, 10);     // up to 10 pts

  riskScore = (int)constrain(tempScore + gasScore + trendScore, 0, 100);

  if (gas >= gasDangerLevel || riskScore >= 75) {
    riskLabel = "CRITICAL";
  } else if (gas >= gasWarnLevel || riskScore >= 40) {
    riskLabel = "CAUTION";
  } else {
    riskLabel = "NORMAL";
  }

  // ---- Build human-readable reasoning string ----
  String tempDesc = (tempTrendPerMin > 0.3) ? "warming" :
                     (tempTrendPerMin < -0.3) ? "cooling" : "stable";
  String gasDesc  = (gasTrendPerMin > 50) ? "rising" :
                     (gasTrendPerMin < -50) ? "falling" : "nominal";

  aiReasoning = "Predicted spoilage risk: " + riskLabel + " -- temp " + tempDesc +
                " (" + String(tempTrendPerMin, 2) + " C/min), gas " + gasDesc +
                " (score " + String(riskScore) + "/100)";

  if (riskLabel == "CAUTION" && gas < gasDangerLevel && gasTrendPerMin > 50) {
    aiReasoning += " | Early warning: gas trending toward threshold faster than usual.";
  }
}

// ---------- SENSOR READ + CONTROL LOGIC ----------
void readSensorsAndControl() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();

  if (!isnan(t)) currentTemp = t;
  if (!isnan(h)) currentHumidity = h;

  currentGas = analogRead(GASPIN);
  gasAlert = (currentGas >= gasDangerLevel);

  updateAIInference(currentTemp, currentGas);

  // Safety cutoff overrides everything - now also reacts to AI risk score,
  // not just the raw hard threshold, so it can act earlier on a fast trend.
  if (gasAlert || riskLabel == "CRITICAL") {
    setRelay(false);
    return;
  }

  if (autoMode) {
    if (currentTemp <= targetTempC) {
      setRelay(false);
    } else if (currentTemp >= targetTempC + hysteresisC) {
      setRelay(true);
    }
    // if temp is between target and target+hysteresis, leave relay as-is (hysteresis band)
  }
  // if manual mode, relay state is left alone here - phone controls it directly
}

// ---------- WEB PAGE ----------
String htmlPage() {
  String html = "<!DOCTYPE html><html><head><title>Fridge Monitor</title>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<meta http-equiv='refresh' content='5'>";
  html += "<style>";
  html += "body{font-family:Arial,sans-serif;background:#0e1117;color:#fff;text-align:center;padding:20px;}";
  html += ".card{background:#1c2129;border-radius:12px;padding:20px;margin:10px auto;max-width:380px;}";
  html += ".val{font-size:2.2em;font-weight:bold;}";
  html += ".on{color:#4caf50;} .off{color:#f44336;} .alert{color:#ff9800;}";
  html += ".risk-normal{color:#4caf50;} .risk-caution{color:#ff9800;} .risk-critical{color:#f44336;}";
  html += "button{padding:14px 24px;font-size:1em;margin:8px;border:none;border-radius:8px;cursor:pointer;}";
  html += ".btnOn{background:#4caf50;color:white;} .btnOff{background:#f44336;color:white;}";
  html += ".btnMode{background:#2196f3;color:white;}";
  html += "</style></head><body>";

  html += "<h2>Fridge Compartment Monitor</h2>";

  html += "<div class='card'><div>Temperature</div><div class='val'>" + String(currentTemp, 1) + " &deg;C</div></div>";
  html += "<div class='card'><div>Humidity</div><div class='val'>" + String(currentHumidity, 1) + " %</div></div>";
  html += "<div class='card'><div>Gas Level</div><div class='val " + String(gasAlert ? "alert" : "") + "'>" + String(currentGas) + (gasAlert ? " ALERT" : "") + "</div></div>";

  String riskClass = (riskLabel == "CRITICAL") ? "risk-critical" : (riskLabel == "CAUTION") ? "risk-caution" : "risk-normal";
  html += "<div class='card'><div>AI Risk Assessment</div>";
  html += "<div class='val " + riskClass + "'>" + riskLabel + " (" + String(riskScore) + "/100)</div>";
  html += "<div style='font-size:0.85em;margin-top:8px;color:#aaa;'>" + aiReasoning + "</div>";
  html += "</div>";

  html += "<div class='card'><div>Compressor (Relay)</div>";
  html += "<div class='val " + String(relayState ? "on" : "off") + "'>" + String(relayState ? "ON" : "OFF") + "</div>";
  html += "<div>Mode: " + String(autoMode ? "AUTO" : "MANUAL") + "</div>";
  html += "<a href='/toggleMode'><button class='btnMode'>Switch to " + String(autoMode ? "MANUAL" : "AUTO") + "</button></a><br>";

  if (!autoMode) {
    html += "<a href='/relayOn'><button class='btnOn'>Turn ON</button></a>";
    html += "<a href='/relayOff'><button class='btnOff'>Turn OFF</button></a>";
  } else {
    html += "<p>(Switch to MANUAL to control relay directly)</p>";
  }
  html += "</div>";

  html += "<div class='card'><div>Target Temp: " + String(targetTempC, 1) + " &deg;C (hysteresis " + String(hysteresisC, 1) + " &deg;C)</div></div>";

  html += "</body></html>";
  return html;
}

// ---------- ROUTES ----------
void handleRoot() {
  server.send(200, "text/html", htmlPage());
}

void handleRelayOn() {
  if (!autoMode) setRelay(true);
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleRelayOff() {
  if (!autoMode) setRelay(false);
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleToggleMode() {
  autoMode = !autoMode;
  server.sendHeader("Location", "/");
  server.send(303);
}

// Simple JSON endpoint in case you want to pull data elsewhere later
void handleData() {
  String json = "{";
  json += "\"temp\":" + String(currentTemp, 1) + ",";
  json += "\"humidity\":" + String(currentHumidity, 1) + ",";
  json += "\"gas\":" + String(currentGas) + ",";
  json += "\"gasAlert\":" + String(gasAlert ? "true" : "false") + ",";
  json += "\"relay\":" + String(relayState ? "true" : "false") + ",";
  json += "\"autoMode\":" + String(autoMode ? "true" : "false") + ",";
  json += "\"riskScore\":" + String(riskScore) + ",";
  json += "\"riskLabel\":\"" + riskLabel + "\",";
  json += "\"tempTrendPerMin\":" + String(tempTrendPerMin, 2) + ",";
  json += "\"gasTrendPerMin\":" + String(gasTrendPerMin, 2);
  json += "}";
  server.send(200, "application/json", json);
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);

  pinMode(RELAYPIN, OUTPUT);
  setRelay(false); // start with compressor OFF until we know temp is safe... 
                    // change to true if you'd rather it default ON at boot

  dht.begin();

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("Open this IP in your phone browser (same WiFi network).");

  server.on("/", handleRoot);
  server.on("/relayOn", handleRelayOn);
  server.on("/relayOff", handleRelayOff);
  server.on("/toggleMode", handleToggleMode);
  server.on("/data", handleData);
  server.begin();
}

// ---------- LOOP ----------
void loop() {
  server.handleClient();

  if (millis() - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = millis();
    readSensorsAndControl();

    Serial.print("Temp: "); Serial.print(currentTemp);
    Serial.print(" C | Hum: "); Serial.print(currentHumidity);
    Serial.print(" % | Gas: "); Serial.print(currentGas);
    Serial.print(" | Relay: "); Serial.print(relayState ? "ON" : "OFF");
    Serial.print(" | Mode: "); Serial.println(autoMode ? "AUTO" : "MANUAL");
  }
}
