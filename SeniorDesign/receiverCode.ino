#include <esp_now.h>
#include <WiFi.h>

// Structure to receive X, Y coordinates
typedef struct struct_message {
    float x;
    float y;
} struct_message;

struct_message incomingCoords;

// Updated callback function to match ESP-NOW changes
void onReceive(const esp_now_recv_info_t * info, const uint8_t *incomingData, int len) {
    memcpy(&incomingCoords, incomingData, sizeof(incomingCoords));
    Serial.print("Received: X=");
    Serial.print(incomingCoords.x);
    Serial.print(", Y=");
    Serial.println(incomingCoords.y);
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_MODE_STA);
    
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW Init Failed");
        return;
    }

    esp_now_register_recv_cb(onReceive);  // Updated function signature
}

void loop() {}
