#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <WebServer.h> // For serving downloads
#include <Wire.h>
#include <RTClib.h>
#include <Adafruit_PN532.h>
#include <TinyGPSPlus.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// =================== PIN Definitions ===================
#define LED_PIN 2       // Built-in LED on most ESP32 boards
#define SD_CS_PIN 5     // SD Card Chip Select pin


// =================== Configuration ===================
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* mqtt_data_topic = "esp32/rfid/data";
const char* mqtt_status_topic = "esp32/rfid/status";
const char* mqtt_command_topic = "esp32/rfid/command"; // For receiving all commands

// =================== Global Objects ===================
File dataFile;
WebServer server(80);
RTC_DS1307 rtc;
Adafruit_PN532 nfc(-1, -1);
#define GPS_RX 16
#define GPS_TX 17
HardwareSerial gpsSerial(2);
TinyGPSPlus gps;
WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastStatusPublish = 0;
const long statusPublishInterval = 30000; // Publish status every 30 seconds

// =================== Function Prototypes ===================
void mqttCallback(char* topic, byte* payload, unsigned int length);
void publishStatus();

// =================== SD Card Functions ===================
void logToSD(String dataLine) {
    dataFile = SD.open("/datalog.csv", FILE_APPEND);
    if (dataFile) {
        dataFile.println(dataLine);
        dataFile.close();
        Serial.println("Logged to SD card.");
    } else {
        Serial.println("Error opening datalog.csv for appending.");
    }
}

// Reads a file from the SD card into a String
String readFile(fs::FS &fs, const char * path) {
    File file = fs.open(path);
    if (!file || file.isDirectory()) {
        return String();
    }
    String fileContent;
    while (file.available()) {
        fileContent += (char)file.read();
    }
    file.close();
    return fileContent;
}

// Writes a String to a file on the SD card
void writeFile(fs::FS &fs, const char * path, const char * message) {
    File file = fs.open(path, FILE_WRITE);
    if (!file) {
        Serial.println("Failed to open file for writing");
        return;
    }
    if (file.print(message)) {
        Serial.println("File written");
    } else {
        Serial.println("Write failed");
    }
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
    // Read Wi-Fi config from SD card
    String wifiConfig = readFile(SD, "/config/wifi.json");
    String ssid;
    String password;

    if (wifiConfig.length() > 0) {
        StaticJsonDocument<128> doc;
        deserializeJson(doc, wifiConfig);
        ssid = doc["ssid"].as<String>();
        password = doc["password"].as<String>();
        Serial.println("Read Wi-Fi config from SD.");
    } else {
        // If file doesn't exist, create it with default values
        Serial.println("Wi-Fi config not found, creating default.");
        ssid = "CONRAD";
        password = "ROBOTICS";
        StaticJsonDocument<128> doc;
        doc["ssid"] = ssid;
        doc["password"] = password;
        String defaultConfig;
        serializeJson(doc, defaultConfig);
        writeFile(SD, "/config/wifi.json", defaultConfig.c_str());
    }

    Serial.print("Connecting to: ");
    Serial.println(ssid);
    WiFi.begin(ssid.c_str(), password.c_str());

    int attempt = 0;
    while (WiFi.status() != WL_CONNECTED && attempt < 20) {
        delay(500);
        Serial.print(".");
        attempt++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi Connected!");
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\nFailed to connect to WiFi.");
    }
}

void reconnect_mqtt() {
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        String clientId = "ESP32-RFID-Logger-" + String(random(0xffff), HEX);
        if (client.connect(clientId.c_str())) {
            Serial.println("connected!");
            client.subscribe(mqtt_command_topic);
            Serial.print("Subscribed to command topic: ");
            Serial.println(mqtt_command_topic);
            // Publish status immediately on connection
            publishStatus();
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

// =================== SETUP ===================
void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    
    Wire.begin(21, 22);

    // --- Initialize SD Card ---
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("Card Mount Failed. Halting.");
        while (1);
    }
    // Create config directory if it doesn't exist
    if (!SD.exists("/config")) {
        SD.mkdir("/config");
    }
    Serial.println("SD Card initialized.");
    
    // --- Initialize Peripherals ---
    if (!rtc.begin()) { Serial.println("Couldn't find RTC!"); }
    if (!rtc.isrunning()) { rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); }
    
    nfc.begin();
    if (!nfc.getFirmwareVersion()) { Serial.println("Didn't find PN532!"); while(1); }
    nfc.SAMConfig();

    gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
    
    // --- Connect & Start Services ---
    setup_wifi();
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(mqttCallback);
    server.on("/downloadlog", handleDownloadLog);
    server.begin();
    Serial.println("HTTP server started.");
}

// =================== MAIN LOOP ===================
void loop() {
    // Handle networking
    if (WiFi.status() == WL_CONNECTED) {
        if (!client.connected()) {
            reconnect_mqtt();
        }
        client.loop();
    }
    server.handleClient();

    // Periodically publish status
    if (millis() - lastStatusPublish > statusPublishInterval) {
        lastStatusPublish = millis();
        if (client.connected()) {
            publishStatus();
        }
    }

    // --- GPS & RFID Logic ---
    while(gpsSerial.available() > 0) gps.encode(gpsSerial.read());

    uint8_t uid[7];
    uint8_t uidLength;
    if (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 100)) {
        // Blink LED on scan
        digitalWrite(LED_PIN, HIGH);
        
        String newUID = "";
        for (uint8_t i = 0; i < uidLength; i++) {
            if (uid[i] < 0x10) newUID += "0";
            newUID += String(uid[i], HEX);
        }
        newUID.toUpperCase();
        
        Serial.print("Card Scanned: ");
        Serial.println(newUID);
        
        DateTime now = rtc.now();
        char timestamp[20];
        sprintf(timestamp, "%04d-%02d-%02d %02d:%02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
        String latitude = gps.location.isValid() ? String(gps.location.lat(), 6) : "0.0";
        String longitude = gps.location.isValid() ? String(gps.location.lng(), 6) : "0.0";
        
        // --- Log to SD Card ---
        String csvLine = String(timestamp) + "," + newUID + "," + latitude + "," + longitude;
        logToSD(csvLine);
        
        // --- Publish to MQTT ---
        StaticJsonDocument<256> doc;
        doc["timestamp"] = timestamp;
        doc["identifier"] = newUID;
        doc["latitude"] = latitude;
        doc["longitude"] = longitude;
        char jsonBuffer[256];
        serializeJson(doc, jsonBuffer);
        if (client.connected()) {
            client.publish(mqtt_data_topic, jsonBuffer);
        }
        
        delay(100); // LED on time
        digitalWrite(LED_PIN, LOW);
        delay(1000); // Debounce delay to prevent multiple quick scans
    }
}


// =================== Command Handling ===================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    if (String(topic) != String(mqtt_command_topic)) {
        return; // Ignore messages not on the command topic
    }

    StaticJsonDocument<256> doc;
    deserializeJson(doc, payload, length);
    String action = doc["action"];

    Serial.print("Command received: ");
    Serial.println(action);

    if (action == "update_wifi") {
        String ssid = doc["ssid"];
        String pass = doc["password"];
        StaticJsonDocument<128> wifiDoc;
        wifiDoc["ssid"] = ssid;
        wifiDoc["password"] = pass;
        String newConfig;
        serializeJson(wifiDoc, newConfig);
        writeFile(SD, "/config/wifi.json", newConfig.c_str());
        Serial.println("Wi-Fi credentials updated. Rebooting in 3s.");
        delay(3000);
        ESP.restart();

    } else if (action == "register_card") {
        String uid = doc["uid"];
        String name = doc["name"];
        String namesJson = readFile(SD, "/config/names.json");

        DynamicJsonDocument namesDoc(1024);
        if (namesJson.length() > 0) {
            deserializeJson(namesDoc, namesJson);
        }
        namesDoc[uid] = name; // Add or update the name for the UID

        String newNamesJson;
        serializeJson(namesDoc, newNamesJson);
        writeFile(SD, "/config/names.json", newNamesJson.c_str());
        
        Serial.println("Card name registered.");
        publishStatus(); // Publish updated status with new names list

    } else if (action == "clear_log") {
        SD.remove("/datalog.csv");
        Serial.println("Data log file cleared.");
        publishStatus(); // Publish updated status
    }
}

void publishStatus() {
    StaticJsonDocument<1024> doc;
    doc["ip"] = WiFi.localIP().toString();
    doc["ssid"] = WiFi.SSID();
    doc["uptime_ms"] = millis();
    
    // SD Card Info
    if (SD.cardType() != CARD_NONE) {
        doc["sd_total_gb"] = (float)SD.cardSize() / (1024 * 1024 * 1024);
        doc["sd_used_gb"] = (float)SD.usedBytes() / (1024 * 1024 * 1024);
    } else {
        doc["sd_status"] = "Not Found";
    }

    // Read registered names and add them to the status
    String namesJson = readFile(SD, "/config/names.json");
    if (namesJson.length() > 0) {
        StaticJsonDocument<512> namesDoc;
        deserializeJson(namesDoc, namesJson);
        doc["registered_names"] = namesDoc;
    }

    char buffer[1024];
    serializeJson(doc, buffer);
    client.publish(mqtt_status_topic, buffer);
    Serial.println("Status published.");
}