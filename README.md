#ifndef ARDUINO_USB_MODE
#define ARDUINO_USB_MODE 1
#endif
#ifndef ARDUINO_USB_CDC_ON_BOOT
#define ARDUINO_USB_CDC_ON_BOOT 1
#endif

#include <Arduino.h>
#include <SPI.h>
#include <Preferences.h>
#include <esp_task_wdt.h>

#define TINY_GSM_MODEM_A7672X
#define TINY_GSM_RX_BUFFER 1024
#include <TinyGsmClient.h>

#include <mcp_can.h>
#include <MFRC522.h>

#define APN             "airtelgprs.com"
const String webhookURL = "http://ntfy.sh/kingsgas_telematics";

// ============================================================
// HARDWARE PIN CONFIGURATION
// ============================================================
#define MODEM_TX        17 
#define MODEM_RX        18 
#define PW_KEY          10    

#define CAN1_CS_PIN     5  
#define CAN_MISO        13 
#define CAN_MOSI        12 
#define CAN_SCK         14 

#define RFID_CS_PIN     21 
#define RFID_RST_PIN    4  
#define RFID_MISO       39 
#define RFID_MOSI       40 
#define RFID_SCK        38 

#define LNG_LEVEL_RAW_MAX       255.0f 
#define MAX_QUEUE_PACKETS       180    

enum SystemState { STATE_MODEM_INIT, STATE_NETWORK_SEARCH, STATE_GPRS_CONNECT, STATE_TELEMETRY_RUN };
SystemState currentState = STATE_MODEM_INIT;

struct TelemetryPacket {
    char        driverID[32];       
    double      lat;                
    double      lon;                
    float       p1; float       p2; float       p8; float       p33;
    uint32_t    p23; float       p35; float       p15; uint16_t   fuel_volume; 
    uint32_t    timestamp;          
};
TelemetryPacket livePacket;

double lastSavedLat = 0.0; double lastSavedLon = 0.0; unsigned long lastSavedTime = 0;
bool gps_fix = false; float gps_heading = 0.0; bool can1_ok = false;
unsigned long lastGpsPoll = 0; unsigned long lastThrottledDrain = 0; unsigned long lastRfidCheck = 0; unsigned long stateTimer = 0; unsigned long lastCanMessageTime = 0; 
uint8_t queueHead = 0; uint8_t queueTail = 0; uint8_t queueCount = 0; uint8_t retryCount = 0; const uint8_t MAX_RETRIES = 3;
uint8_t currentPIDIndex = 0; uint8_t obdPIDs[3] = {0x0C, 0x0D, 0x05}; 

MCP_CAN CAN1(CAN1_CS_PIN);
MFRC522 rfid(RFID_CS_PIN, RFID_RST_PIN);
HardwareSerial modemSerial(1);
TinyGsm modem(modemSerial);
Preferences prefs;

double calculateDistance(double lat1, double lon1, double lat2, double lon2);
bool sendATCommand(String cmd, const char* expectedResponse, unsigned long timeout);
void initGPS(); void pollGPS(); void readCAN(); void sendOBDRequest(uint8_t pid); void pollRFID();
String serializePacket(const TelemetryPacket &pkt); void enqueuePacket(const TelemetryPacket &pkt); bool dequeueAndSend(); bool sendNativeHTTP(String payload); void triggerTransmit(); void loadQueueState(); void saveQueueState();

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("\n==================================================");
    Serial.println("  UNIVERSAL FIRMWARE v20.6 — FIX BRACKETS ERROR   ");
    Serial.println("==================================================");

    #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        esp_task_wdt_config_t twdt_config = {
            .timeout_ms = 30000,
            .idle_core_mask = (1 << 0) | (1 << 1),
            .trigger_panic = true
        };
        esp_task_wdt_reconfigure(&twdt_config);
    #else
        esp_task_wdt_init(30, true); 
    #endif
    esp_task_wdt_add(NULL);

    pinMode(CAN1_CS_PIN, OUTPUT); digitalWrite(CAN1_CS_PIN, HIGH);
    pinMode(RFID_CS_PIN, OUTPUT); digitalWrite(RFID_CS_PIN, HIGH);

    prefs.begin("telQ", false); prefs.clear(); prefs.end();

    memset(&livePacket, 0, sizeof(livePacket));
    strlcpy(livePacket.driverID, "NO_CARD", sizeof(livePacket.driverID));

    loadQueueState();
    
    SPI.begin(CAN_SCK, CAN_MISO, CAN_MOSI, CAN1_CS_PIN);
    if (CAN1.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
        CAN1.setMode(MCP_NORMAL);
        can1_ok = true;
    }
}

void loop() {
    unsigned long now = millis();

    if (can1_ok) {
        SPI.end(); SPI.begin(CAN_SCK, CAN_MISO, CAN_MOSI, CAN1_CS_PIN);
        if (CAN1.checkReceive() == CAN_MSGAVAIL) { readCAN(); lastCanMessageTime = millis(); }
    }

    if (now - lastCanMessageTime > 5000UL) {
        livePacket.p1 = 0.0f; livePacket.p2 = 0.0f; livePacket.p8 = 0.0f; livePacket.p33 = 0.0f; livePacket.p15 = 0.0f; 
    }

    if (now - lastRfidCheck >= 400UL) {
        lastRfidCheck = now;
        pollRFID();
    }

    switch (currentState) {
        case STATE_MODEM_INIT: {
            Serial.println("\n[STATE] -> STATE_MODEM_INIT");
            pinMode(PW_KEY, OUTPUT);
            digitalWrite(PW_KEY, HIGH); delay(1000); 
            digitalWrite(PW_KEY, LOW);  delay(3500); 
            digitalWrite(PW_KEY, HIGH); 
            Serial.println("[MODEM] PW_KEY Released. Waiting for Module Internal Boot...");
            
            for(int i = 0; i < 8; i++) {
                esp_task_wdt_reset();
                delay(1000);
            }
            
            modemSerial.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
            delay(1000);

            retryCount = 0; bool modemReady = false;
            while (retryCount < 8) {
                esp_task_wdt_reset();
                if (sendATCommand("AT", "OK", 2000)) { modemReady = true; break; }
                retryCount++; delay(2000);
            }

            if (modemReady) {
                sendATCommand("ATE0", "OK", 1000); 
                initGPS(); 
                currentState = STATE_NETWORK_SEARCH;
            } else {
                Serial.println("[CRITICAL ERROR] Modem responsive fail.");
                delay(3000);
            }
            break;
        }

        case STATE_NETWORK_SEARCH: {
            Serial.println("[STATE] -> STATE_NETWORK_SEARCH");
            bool networkRegistered = false; stateTimer = millis();
            while (millis() - stateTimer < 45000UL) { 
                modemSerial.println("AT+CREG?"); delay(500);
                if (modemSerial.available()) {
                    String resp = modemSerial.readString();
                    if (resp.indexOf("+CREG: 0,1") >= 0 || resp.indexOf("+CREG: 0,5") >= 0 ||
                        resp.indexOf("+CREG: 1,1") >= 0 || resp.indexOf("+CREG: 1,5") >= 0) {
                        networkRegistered = true; break;
                    }
                }
                delay(2500);
            }
            if (networkRegistered) {
                Serial.println("[SUCCESS] Registered on Airtel Network.");
                currentState = STATE_GPRS_CONNECT; 
            } else {
                currentState = STATE_MODEM_INIT;
            }
            break;
        }

        case STATE_GPRS_CONNECT: {
            Serial.println("[STATE] -> STATE_GPRS_CONNECT");
            sendATCommand("AT+CGDCONT=1,\"IP\",\"" + String(APN) + "\"", "OK", 2000);
            if (modem.gprsConnect(APN, "", "")) {
                Serial.printf("[SUCCESS] GPRS Active. IP: %s\n", modem.localIP().toString().c_str());
                currentState = STATE_TELEMETRY_RUN; 
            } else {
                currentState = STATE_NETWORK_SEARCH;
            }
            break;
        }

        case STATE_TELEMETRY_RUN: {
            if (!modem.isGprsConnected()) { 
                Serial.println("[WARNING] GPRS Lost. Reconnecting...");
                currentState = STATE_GPRS_CONNECT; 
                break; 
            }

            if (now - lastGpsPoll >= 7000UL) {
                lastGpsPoll = now;
                pollGPS(); 
                
                sendOBDRequest(obdPIDs[currentPIDIndex]);
                currentPIDIndex = (currentPIDIndex + 1) % 3;

                // 🚀 सक्तीने ट्रान्समिट: दर ७ सेकंदाला डेटा डायरेक्ट इंटरनेटवर जाणार!
                if (livePacket.lat > 1.0) {
                    triggerTransmit(); 
                } else {
                    Serial.println("[INFO] Skipping Transmit: Valid GPS coordinates not parsed yet.");
                }
            }

            if (queueCount > 0 && (now - lastThrottledDrain >= 8000UL)) {
                lastThrottledDrain = now; dequeueAndSend(); 
            }
            break;
        }
    }
} // 🏆 🌟 हा तो लूपचा मुख्य क्लोजिंग कंस जो सुटला होता!

bool sendNativeHTTP(String payload) {
    if (!sendATCommand("AT+HTTPINIT", "OK", 2000)) {
        sendATCommand("AT+HTTPTERM", "OK", 1000); 
        if (!sendATCommand("AT+HTTPINIT", "OK", 2000)) return false;
    }
    sendATCommand("AT+HTTPPARA=\"CID\",1", "OK", 1000); 
    sendATCommand("AT+HTTPPARA=\"URL\",\"" + webhookURL + "\"", "OK", 2000);
    sendATCommand("AT+HTTPPARA=\"CONTENT\",\"application/json\"", "OK", 2000);
    modemSerial.print("AT+HTTPDATA=" + String(payload.length()) + ",5000\r\n"); delay(200);
    modemSerial.print(payload); delay(500);
    modemSerial.println("AT+HTTPACTION=1");
    unsigned long start = millis(); bool success = false;
    while (millis() - start < 15000UL) {
        if (modemSerial.available()) {
            String resp = modemSerial.readString();
            if (resp.indexOf(",200,") >= 0 || resp.indexOf(",201,") >= 0) { success = true; break; }
        }
        yield();
    }
    sendATCommand("AT+HTTPTERM", "OK", 2000);
    return success;
}

void triggerTransmit() {
    livePacket.timestamp = millis(); String payload = serializePacket(livePacket);
    Serial.println("\n[NTFY TRANSMIT ATTEMPT] -> " + payload);
    if (sendNativeHTTP(payload)) { Serial.println("[HTTP SUCCESS] Pushed directly to ntfy Dashboard."); } 
    else { enqueuePacket(livePacket); }
}

bool dequeueAndSend() {
    if (queueCount == 0) return true;
    char key[8]; snprintf(key, sizeof(key), "pk%03d", queueHead); TelemetryPacket storedPacket;
    prefs.begin("telQ", true); prefs.getBytes(key, &storedPacket, sizeof(TelemetryPacket)); prefs.end();
    if (sendNativeHTTP(serializePacket(storedPacket))) {
        prefs.begin("telQ", false); prefs.remove(key); prefs.end();
        queueHead = (queueHead + 1) % MAX_QUEUE_PACKETS; queueCount--; saveQueueState(); return true;
    }
    return false;
}

bool sendATCommand(String cmd, const char* expectedResponse, unsigned long timeout) {
    modemSerial.println(cmd); unsigned long start = millis();
    while (millis() - start < timeout) { if (modemSerial.available()) { if (modemSerial.readString().indexOf(expectedResponse) >= 0) return true; } yield(); }
    return false;
}

void initGPS() { 
    esp_task_wdt_reset();
    Serial.println("[GPS] Activating SIMCom GNSS Power...");
    sendATCommand("AT+CGNSSPWR=1", "OK", 3000); 
    delay(500);
    
    // 🌟 सर्व सॅटेलाईट सिस्टीम्स (GPS+GLONASS+Galileo+BDS) एकत्र सुरू करा 
    // यामुळे इमारतींच्या बाजूला असूनही अचूकता कडक मिळेल!
    sendATCommand("AT+CGNSSMODE=15", "OK", 3000); 
    delay(500);
    
    // 🌟 अचूकता वाढवण्यासाठी मिनिमम NMEA हाय-फिडेलिटी डेटा रेट सेट करा
    sendATCommand("AT+CGNSSCMD=0,1", "OK", 2000);
    delay(500);
    
    Serial.println("[GPS] SIMCom GNSS Active with Maximum Accuracy Mode!");
}

void pollGPS() {
    esp_task_wdt_reset();
    while(modemSerial.available()) { modemSerial.read(); }
    
    modemSerial.println("AT+CGNSSINFO");
    
    unsigned long start = millis();
    String gpsRawResponse = "";
    while (millis() - start < 1500UL) {
        if (modemSerial.available()) {
            gpsRawResponse += modemSerial.readString();
            if (gpsRawResponse.indexOf("+CGNSSINFO:") >= 0) break;
        }
        yield();
    }
    
    Serial.print("\n[REAL SIMCOM RESPONSE] -> ");
    Serial.println(gpsRawResponse);

    int locIdx = gpsRawResponse.indexOf("+CGNSSINFO:");
    if (locIdx < 0) return; 
    
    String dataStr = gpsRawResponse.substring(locIdx + 11);
    int endLine = dataStr.indexOf('\n');
    if (endLine >= 0) dataStr = dataStr.substring(0, endLine);
    dataStr.trim();

    if (dataStr.length() < 15) {
        Serial.println("GPS Searching... (Waiting for Sky Lock)"); 
        livePacket.lat = 0.0; livePacket.lon = 0.0; gps_fix = false; 
        return;
    }

    // 🌟 जादुई आणि बुलेटप्रूफ कॉमा-पार्सर (इंडेक्सचा घोळच मिटवला)
    int commaCount = 0;
    String latStr = "";
    String lonStr = "";
    
    for (int i = 0; i < (int)dataStr.length(); i++) {
        if (dataStr[i] == ',') {
            commaCount++;
            continue;
        }
        
        // सिमकोमच्या रिस्पॉन्समध्ये ५ व्या स्वल्पविरामानंतर Latitude सुरू होतो
        if (commaCount == 5) {
            latStr += dataStr[i];
        }
        // ७ व्या स्वल्पविरामानंतर Longitude सुरू होतो
        if (commaCount == 7) {
            lonStr += dataStr[i];
        }
    }

    latStr.trim();
    lonStr.trim();

    if (latStr.length() > 3 && lonStr.length() > 3) {
        double tempLat = latStr.toDouble();
        double tempLon = lonStr.toDouble();

        if (tempLat > 1.0 && tempLon > 1.0) {
            livePacket.lat = tempLat; 
            livePacket.lon = tempLon;
            gps_fix = true;
            
            Serial.printf("🏆 [SUCCESS] GPS FIX LOCKED! Lat: %.6f, Lon: %.6f\n", livePacket.lat, livePacket.lon);
            return;
        }
    }
    
    Serial.println("[INFO] Parsing failed or formatting issue.");
    livePacket.lat = 0.0;
    livePacket.lon = 0.0;
    gps_fix = false;
}

void sendOBDRequest(uint8_t pid) { 
    uint8_t reqData[8] = {0x02, 0x01, pid, 0x00, 0x00, 0x00, 0x00, 0x00}; 
    if(can1_ok) {
        SPI.end(); SPI.begin(CAN_SCK, CAN_MISO, CAN_MOSI, CAN1_CS_PIN);
        CAN1.sendMsgBuf(0x7DF, 0, 8, reqData); 
    }
}

void readCAN() {
    unsigned long canID = 0; uint8_t ext = 0, dlc = 0, data[8];
    // 🌟 कॅन रिड फिक्स जोडला (&dlc)
    if (CAN1.readMsgBuf(&canID, &ext, &dlc, data) == CAN_OK) {
        if (canID == 0x7E8 && data[1] == 0x41) { 
            switch (data[2]) { case 0x0C: livePacket.p2 = ((float)(data[3] * 256) + data[4]) / 4.0f; break; case 0x0D: livePacket.p1 = data[3]; break; case 0x05: livePacket.p8 = data[3] - 40; break; } return;
        }
        uint8_t pf = (canID >> 16) & 0xFF; uint8_t ps = (canID >> 8) & 0xFF; uint8_t dp = (canID >> 24) & 0x01;
        uint32_t pgn = (pf >= 0xF0) ? ((uint32_t)dp << 16) | ((uint32_t)pf << 8) | ps : ((uint32_t)dp << 16) | ((uint32_t)pf << 8);
        switch (pgn) {
            case 0xFE6C: if (dlc >= 8) livePacket.p1 = ((uint16_t)data[7] << 8 | data[6]) / 256.0f; break;
            case 0xF004: if (dlc >= 5) livePacket.p2 = ((uint16_t)data[4] << 8 | data[3]) * 0.125f; break;
            case 0xFEEE: if (dlc >= 1) livePacket.p8 = data[0] - 40.0f; break;
            case 0xFEEF: if (dlc >= 4) livePacket.p33 = data[3] * 4.0f; break;
            case 0xFEC1: if (dlc >= 4) { uint32_t rawOdo = (uint32_t)(((uint32_t)data[3] << 24) | ((uint32_t)data[2] << 16) | ((uint32_t)data[1] << 8) | data[0]); uint32_t calculatedOdo = (uint32_t)(rawOdo * 0.005f); if (calculatedOdo < 2000000UL) livePacket.p23 = calculatedOdo; } break;
            case 0xFEF7: if (dlc >= 2) livePacket.p35 = ((uint16_t)data[1] << 8 | data[0]) * 0.05f; break;
            case 0xFFEB: if (dlc >= 8) { livePacket.p15 = (data[5] / LNG_LEVEL_RAW_MAX) * 100.0f; livePacket.fuel_volume = ((uint16_t)data[7] << 8) | data[6]; } break;
        }
    }
}

void pollRFID() {
    digitalWrite(CAN1_CS_PIN, HIGH); SPI.end();
    SPI.begin(RFID_SCK, RFID_MISO, RFID_MOSI, RFID_CS_PIN); rfid.PCD_Init();
    
    if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) {
        digitalWrite(RFID_CS_PIN, HIGH); SPI.end(); SPI.begin(CAN_SCK, CAN_MISO, CAN_MOSI, CAN1_CS_PIN); 
        return; 
    }
    char uid[32] = {0};
    for (uint8_t i = 0; i < rfid.uid.size; i++) { char hex[4]; snprintf(hex, sizeof(hex), "%02X", rfid.uid.uidByte[i]); strncat(uid, hex, sizeof(uid) - strlen(uid) - 1); }
    rfid.PICC_HaltA(); rfid.PCD_StopCrypto1();
    if (strlen(uid) < 4 || uid[0] == '\u001b' || uid[0] == ' ') {
        digitalWrite(RFID_CS_PIN, HIGH); SPI.end(); SPI.begin(CAN_SCK, CAN_MISO, CAN_MOSI, CAN1_CS_PIN);
        return; 
    }
    
    if (strcmp(uid, livePacket.driverID) != 0) { 
        strlcpy(livePacket.driverID, uid, sizeof(livePacket.driverID)); 
        Serial.printf("\n[DRIVER CHANGED] Active Driver UID Locked: %s\n", livePacket.driverID);
        if (currentState == STATE_TELEMETRY_RUN) {
            triggerTransmit(); 
        }
    }
    digitalWrite(RFID_CS_PIN, HIGH); SPI.end(); SPI.begin(CAN_SCK, CAN_MISO, CAN_MOSI, CAN1_CS_PIN); delay(5);
}

double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    double dLat = (lat2 - lat1) * DEG_TO_RAD; double dLon = (lon2 - lon1) * DEG_TO_RAD;
    double a = sin(dLat/2) * sin(dLat/2) + cos(lat1*DEG_TO_RAD) * cos(lat2*DEG_TO_RAD) * sin(dLon/2) * sin(dLon/2);
    return 6371000.0 * (2 * atan2(sqrt(a), sqrt(1-a)));
}

String serializePacket(const TelemetryPacket &pkt) {
    String json = "{";
    json += "\"driver_uid\":\"" + String(pkt.driverID) + "\",";
    json += "\"lat\":" + String(pkt.lat, 6) + ","; // 🏆 🌟 शुद्ध 'lat' केला!
    json += "\"lon\":" + String(pkt.lon, 6) + ",";
    json += "\"heading\":" + String(gps_heading, 1) + ",";
    json += "\"speed_kmh\":" + String(pkt.p1, 1) + ","; 
    json += "\"rpm\":" + String(pkt.p2, 0) + ",";
    json += "\"coolant_c\":" + String(pkt.p8, 1) + ",";
    json += "\"lng_level_pct\":" + String(pkt.p15, 1) + ",";
    json += "\"lng_volume_l\":" + String(pkt.fuel_volume) + ",";
    json += "\"odometer_km\":" + String(pkt.p23) + ","; 
    json += "\"battery_v\":" + String(pkt.p35 == 0 ? 12.6f : pkt.p35, 2) + ",";
    json += "\"uptime_sec\":" + String(millis() / 1000);
    json += "}"; 
    return json;
}

void loadQueueState() { prefs.begin("telQ", false); queueHead = prefs.getUChar("head", 0); queueTail = prefs.getUChar("tail", 0); queueCount = prefs.getUChar("count", 0); prefs.end(); }
void saveQueueState() { prefs.begin("telQ", false); prefs.putUChar("head", queueHead); prefs.putUChar("tail", queueTail); prefs.putUChar("count", queueCount); prefs.end(); }
void enqueuePacket(const TelemetryPacket &pkt) { if (queueCount >= MAX_QUEUE_PACKETS) { queueHead = (queueHead + 1) % MAX_QUEUE_PACKETS; queueCount--; } char key[8]; snprintf(key, sizeof(key), "pk%03d", queueTail); prefs.begin("telQ", false); prefs.putBytes(key, &pkt, sizeof(TelemetryPacket)); prefs.end(); queueTail = (queueTail + 1) % MAX_QUEUE_PACKETS; queueCount++; saveQueueState(); }
