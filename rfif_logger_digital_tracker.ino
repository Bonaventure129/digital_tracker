#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <RTClib.h>
#include <Adafruit_PN532.h>
#include <TinyGPSPlus.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// =================== PIN Definitions ===================
#define LED_PIN 26      // Built-in LED on most ESP32 boards
#define SD_CS_PIN 5
#define GPS_RX 16
#define GPS_TX 17

// =================== Configuration ===================
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* mqtt_data_topic = "esp32/rfid/data";
const char* mqtt_status_topic = "esp32/rfid/status";
const char* mqtt_command_topic = "esp32/rfid/command";

// =================== Global Objects ===================
WebServer server(80);
RTC_DS1307 rtc;
Adafruit_PN532 nfc(-1, -1);
HardwareSerial gpsSerial(2);
TinyGPSPlus gps;
WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastStatusPublish = 0;
const long statusPublishInterval = 30000;

// =================== Function Prototypes ===================
void mqttCallback(char* topic, byte* payload, unsigned int length);
void publishStatus();
String readFile(fs::FS &fs, const char * path);
void writeFile(fs::FS &fs, const char * path, const char * message);
void logToSD(const String& timestamp, const String& eventType, const String& identifier, const String& details);

// =================== SD Card Functions ===================
void logToSD(const String& timestamp, const String& eventType, const String& identifier, const String& details) {
    File dataFile = SD.open("/datalog.csv", FILE_APPEND);
    if (dataFile) {
        dataFile.print(timestamp);
        dataFile.print(",");
        dataFile.print(eventType);
        dataFile.print(",");
        dataFile.print(identifier);
        dataFile.print(",");
        dataFile.println(details);
        dataFile.close();
    } else {
        Serial.println("Error opening datalog.csv for appending.");
    }
}

String readFile(fs::FS &fs, const char * path) {
    File file = fs.open(path);
    if (!file || file.isDirectory()) { return String(); }
    String fileContent;
    while (file.available()) { fileContent += (char)file.read(); }
    file.close();
    return fileContent;
}

void writeFile(fs::FS &fs, const char * path, const char * message) {
    File file = fs.open(path, FILE_WRITE);
    if (!file) { Serial.println("Failed to open file for writing"); return; }
    if (file.print(message)) { Serial.println("File written"); } else { Serial.println("Write failed"); }
    file.close();
}

// =================== Web Server Handlers ===================
void handleDownloadLog() {
    File download = SD.open("/datalog.csv", "r");
    if (download) {
        server.sendHeader("Content-Disposition", "attachment; filename=datalog.csv");
        server.streamFile(download, "text/csv");
        download.close();
    } else {
        server.send(404, "text/plain", "File not found");
    }
}

// =================== Wi-Fi & MQTT Setup ===================
void setup_wifi() {
    String wifiConfig = readFile(SD, "/config/wifi.json");
    String ssid, password;
    if (wifiConfig.length() > 0) {
        StaticJsonDocument<128> doc;
        deserializeJson(doc, wifiConfig);
        ssid = doc["ssid"].as<String>();
        password = doc["password"].as<String>();
    } else {
        ssid = "CONRAD";
        password = "ROBOTICS";
        StaticJsonDocument<128> doc;
        doc["ssid"] = ssid;
        doc["password"] = password;
        String defaultConfig;
        serializeJson(doc, defaultConfig);
        writeFile(SD, "/config/wifi.json", defaultConfig.c_str());
    }
    WiFi.begin(ssid.c_str(), password.c_str());
    Serial.print("Connecting to: "); Serial.println(ssid);
    int attempt = 0;
    while (WiFi.status() != WL_CONNECTED && attempt++ < 20) {
        delay(500); Serial.print(".");
    }
    if(WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi Connected!"); Serial.print("IP: "); Serial.println(WiFi.localIP());
    } else { Serial.println("\nFailed to connect.");}
}

void reconnect_mqtt() {
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        String clientId = "ESP32-RFID-Logger-" + String(random(0xffff), HEX);
        if (client.connect(clientId.c_str())) {
            Serial.println("connected!");
            client.subscribe(mqtt_command_topic);
            publishStatus();
        } else {
            Serial.print("failed, rc="); Serial.print(client.state()); Serial.println(" try again in 5s");
            delay(5000);
        }
    }
}

// =================== SETUP ===================
void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    Wire.begin();
    if (!SD.begin(SD_CS_PIN)) { Serial.println("Card Mount Failed!"); while(1); }
    if (!SD.exists("/config")) { SD.mkdir("/config"); }
    if (!rtc.begin()) { Serial.println("Couldn't find RTC!"); }
    if (!rtc.isrunning()) { rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); }
    nfc.begin();
    if (!nfc.getFirmwareVersion()) { Serial.println("Didn't find PN532!"); while(1); }
    nfc.SAMConfig();
    gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
    setup_wifi();
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(mqttCallback);
    server.on("/downloadlog", handleDownloadLog);
    server.begin();
}

// =================== MAIN LOOP ===================
void loop() {
    if (WiFi.status() == WL_CONNECTED) {
        if (!client.connected()) { reconnect_mqtt(); }
        client.loop();
    }
    server.handleClient();
    if (millis() - lastStatusPublish > statusPublishInterval) {
        lastStatusPublish = millis();
        if (client.connected()) { publishStatus(); }
    }
    while(gpsSerial.available() > 0) gps.encode(gpsSerial.read());

    uint8_t uid[7];
    uint8_t uidLength;
    if (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 100)) {
        digitalWrite(LED_PIN, HIGH);
        String newUID = "";
        for (uint8_t i = 0; i < uidLength; i++) {
            if (uid[i] < 0x10) newUID += "0";
            newUID += String(uid[i], HEX);
        }
        newUID.toUpperCase();
        
        DateTime now = rtc.now();
        char timestamp[20];
        sprintf(timestamp, "%04d-%02d-%02d %02d:%02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
        String latitude = gps.location.isValid() ? String(gps.location.lat(), 6) : "0.0";
        String longitude = gps.location.isValid() ? String(gps.location.lng(), 6) : "0.0";
        
        // --- FEATURE: Look up registered name ---
        String namesJson = readFile(SD, "/config/names.json");
        DynamicJsonDocument namesDoc(1024);
        deserializeJson(namesDoc, namesJson);
        String displayName = namesDoc[newUID] | "Unknown"; // Default to "Unknown" if not found

        // Log to SD card with the name
        logToSD(timestamp, "SCAN", newUID, displayName + " (" + latitude + "," + longitude + ")");
        
        // Publish to MQTT with the name
        StaticJsonDocument<256> doc;
        doc["timestamp"] = timestamp;
        doc["identifier"] = newUID;
        doc["displayName"] = displayName; // Send the name to the dashboard
        doc["latitude"] = latitude;
        doc["longitude"] = longitude;
        char jsonBuffer[256];
        serializeJson(doc, jsonBuffer);
        if (client.connected()) client.publish(mqtt_data_topic, jsonBuffer);
        
        delay(100);
        digitalWrite(LED_PIN, LOW);
        delay(1000);
    }
}

// =================== Command Handling ===================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    if (String(topic) != String(mqtt_command_topic)) { return; }
    StaticJsonDocument<256> doc;
    deserializeJson(doc, payload, length);
    String action = doc["action"];
    Serial.print("Command received: "); Serial.println(action);

    if (action == "update_wifi") {
        String ssid = doc["ssid"]; String pass = doc["password"];
        StaticJsonDocument<128> wifiDoc;
        wifiDoc["ssid"] = ssid; wifiDoc["password"] = pass;
        String newConfig; serializeJson(wifiDoc, newConfig);
        writeFile(SD, "/config/wifi.json", newConfig.c_str());
        delay(3000); ESP.restart();
    } else if (action == "register_card") {
        String uid = doc["uid"]; String name = doc["name"];
        String namesJson = readFile(SD, "/config/names.json");
        DynamicJsonDocument namesDoc(1024);
        if (namesJson.length() > 0) deserializeJson(namesDoc, namesJson);
        namesDoc[uid] = name;
        String newNamesJson; serializeJson(namesDoc, newNamesJson);
        writeFile(SD, "/config/names.json", newNamesJson.c_str());
        
        DateTime now = rtc.now(); char timestamp[20];
        sprintf(timestamp, "%04d-%02d-%02d %02d:%02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
        logToSD(timestamp, "REGISTER", uid, "Assigned name: " + name);
        
        publishStatus();
    } else if (action == "clear_log") {
        SD.remove("/datalog.csv");
        publishStatus();
    }
}

void publishStatus() {
    StaticJsonDocument<1024> doc;
    doc["ip"] = WiFi.localIP().toString();
    doc["ssid"] = WiFi.SSID();
    doc["uptime_ms"] = millis();
    if (SD.cardType() != CARD_NONE) {
        doc["sd_total_gb"] = (float)SD.cardSize() / (1024 * 1024 * 1024);
        doc["sd_used_gb"] = (float)SD.usedBytes() / (1024 * 1024 * 1024);
    } else { doc["sd_status"] = "Not Found"; }
    String namesJson = readFile(SD, "/config/names.json");
    if (namesJson.length() > 0) {
        StaticJsonDocument<512> namesDoc;
        deserializeJson(namesDoc, namesJson);
        doc["registered_names"] = namesDoc;
    }
    char buffer[1024];
    serializeJson(doc, buffer);
    client.publish(mqtt_status_topic, buffer);
}