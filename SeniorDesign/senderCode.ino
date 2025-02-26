#include <esp_now.h>
#include <WiFi.h>

// Replace with the receiver ESP32's MAC address
uint8_t RECEIVER_MAC[] = {0xCC, 0xDB, 0xA7, 0x9F, 0x72, 0x54};  
static unsigned long lastPrintTime = 0;


// Structure to send X, Y coordinates
typedef struct struct_message {
    float x;
    float y;
} struct_message;

// Create a struct message instance
struct_message coords;

void onSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("Delivery status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_MODE_STA);
    
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW Init Failed");
        return;
    }
    
    esp_now_register_send_cb(onSent);
    
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, RECEIVER_MAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }

    Serial.println("Enter X and Y values in Serial Monitor (format: X Y)");
}

void loop() {
    if (Serial.available() > 0) {
        // Read X and Y values from Serial input
        coords.x = Serial.parseFloat();
        coords.y = Serial.parseFloat();

        while (Serial.available() > 0) {
            Serial.read();
        }
        
        // Send data via ESP-NOW
        esp_err_t result = esp_now_send(RECEIVER_MAC, (uint8_t *)&coords, sizeof(coords));
        delay(100);
        
        if (result == ESP_OK) {
            if (millis() - lastPrintTime > 1000) {  // Print once every second
              Serial.print("Sent: X=");
              Serial.print(coords.x);
              Serial.print(", Y=");
              Serial.println(coords.y);
              lastPrintTime = millis();
            }

        } else {
            Serial.println("Failed to send coordinates");
        }

        delay(500); // Small delay to avoid spamming
    }
}
