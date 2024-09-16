#include <ModbusMaster.h>
#include <WiFi.h>
#include <PubSubClient.h>

/* Modbus settings */
#define MODBUS_DIR_PIN  23
#define MODBUS_RX_PIN   16
#define MODBUS_TX_PIN   17
#define MODBUS_SERIAL Serial2
#define MODBUS_SERIAL_BAUD 9600

// Initialize the ModbusMaster object as node
ModbusMaster node;

/* WiFi settings */
const char* ssid = "raspi4-iiot";
const char* password = "iota2024";

/* MQTT settings */
const char* mqtt_server = "10.42.0.1";
const int mqtt_port = 1883;
const char* mqtt_user = "npdtom"; // Optional
const char* mqtt_password = "npd@tom"; // Optional
const char* mqtt_topic = "die_height";
const char* mqtt_subscribe_topic = "die_signal"; // Topic to subscribe to

// Initialize the WiFi and MQTT clients
WiFiClient espClient;
PubSubClient client(espClient);

// Relay pins
#define RELAY_UP_PIN 21
#define RELAY_DOWN_PIN 19

// Function prototypes
void modbusPreTransmission();
void modbusPostTransmission();
void setup_wifi();
void reconnect();
void callback(char* topic, byte* payload, unsigned int length);
bool readCurrentDieHeight(float& dieHeight);
void operateRelay(uint8_t relayPin, float targetHeight, float threshold, bool isUpward);

// Setup function
void setup() {
    Serial.begin(115200);
    pinMode(MODBUS_DIR_PIN, OUTPUT);
    digitalWrite(MODBUS_DIR_PIN, LOW);

    MODBUS_SERIAL.begin(MODBUS_SERIAL_BAUD, SERIAL_8N1, MODBUS_RX_PIN, MODBUS_TX_PIN);
    MODBUS_SERIAL.setTimeout(1000);
    node.begin(8, MODBUS_SERIAL);
    node.preTransmission(modbusPreTransmission);
    node.postTransmission(modbusPostTransmission);

    Serial.println("Modbus setup complete");

    setup_wifi();

    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);

    // Initialize relay pins
    pinMode(RELAY_UP_PIN, OUTPUT);
    pinMode(RELAY_DOWN_PIN, OUTPUT);
    digitalWrite(RELAY_UP_PIN, LOW);
    digitalWrite(RELAY_DOWN_PIN, LOW);
}

// Main loop function
void loop() {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();

    float height;
    if (readCurrentDieHeight(height)) {
        char msg[50];
        snprintf(msg, 50, "Height: %.4f", height);
        client.publish(mqtt_topic, msg);
    } else {
        Serial.println("Failed to read Modbus registers");
    }

    delay(1000);
}

// Modbus transmission modes
void modbusPreTransmission() {
    digitalWrite(MODBUS_DIR_PIN, HIGH);
}

void modbusPostTransmission() {
    digitalWrite(MODBUS_DIR_PIN, LOW);
}

// WiFi setup
void setup_wifi() {
    delay(10);
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

// MQTT reconnect
void reconnect() {
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (client.connect("ESP32Client", mqtt_user, mqtt_password)) {
            Serial.println("connected");
            client.subscribe(mqtt_subscribe_topic);
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

// MQTT callback function
void callback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    
    String message = "";
    for (int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    Serial.println(message);

    if (message == "up") {
        digitalWrite(RELAY_UP_PIN, HIGH); // Activate the relay
        delay(585);
        digitalWrite(RELAY_UP_PIN, LOW);  // Deactivate the relay
    } else if (message == "down") {
        digitalWrite(RELAY_DOWN_PIN, HIGH);  // Activate the relay
        delay(585);
        digitalWrite(RELAY_DOWN_PIN, LOW);   // Deactivate the relay
    } else {
        float dieSignal = message.toFloat();
        if (dieSignal < 250.0000 || dieSignal > 350.0000) {
            Serial.println("Invalid input: Out of range");
            return;
        }

        float currentDieHeight;
        if (!readCurrentDieHeight(currentDieHeight)) {
            Serial.println("Failed to read Modbus registers");
            return;
        }

        float difference = currentDieHeight - dieSignal;
        const float computationalThreshold = 0.125;

        if (difference > computationalThreshold) {
            Serial.println("Operating down relay");
            operateRelay(RELAY_DOWN_PIN, dieSignal, computationalThreshold, false);
        } else if (difference < -computationalThreshold) {
            Serial.println("Operating up relay");
            operateRelay(RELAY_UP_PIN, dieSignal, computationalThreshold, true);
        } else {
            Serial.println("Height is within the acceptable range.");
        }
    }
}

// Read current die height from Modbus
bool readCurrentDieHeight(float& dieHeight) {
    uint8_t result = node.readHoldingRegisters(0x0000, 2);
    if (result != node.ku8MBSuccess) {
        return false;
    }

    uint16_t register1 = node.getResponseBuffer(0x00);
    uint16_t register2 = node.getResponseBuffer(0x01);

    uint8_t byte1 = register1 >> 8;
    uint8_t byte2 = register1 & 0xFF;
    uint8_t byte3 = register2 >> 8;
    uint8_t byte4 = register2 & 0xFF;

    uint32_t combinedValue = ((uint32_t)byte1 << 24) | ((uint32_t)byte2 << 16) | ((uint32_t)byte3 << 8) | byte4;
    dieHeight = combinedValue / 100000.0;

    Serial.print("Current Die Height: ");
    Serial.println(dieHeight, 4);

    return true;
}

void operateRelay(uint8_t relayPin, float targetHeight, float threshold, bool isUpward) {
    digitalWrite(relayPin, HIGH);  // Activate the relay

    float currentDieHeight;
    while (true) {
        if (!readCurrentDieHeight(currentDieHeight)) {
            Serial.println("Error updating die height");
            digitalWrite(relayPin, LOW);  // Deactivate the relay
            return;
        }

        float difference = targetHeight - currentDieHeight;

        // Fine-tuning logic to prevent overshoot or undershoot
        if (isUpward) {
            if (difference <= threshold) {
                break;
            }
            if (difference < 0.02) {
                delay(50);  // Small delay to ensure relay deactivation timing
                break;
            }
        } else {
            if (-difference <= threshold) {
                break;
            }
            if (difference > -0.02) {
                delay(50);  // Small delay to ensure relay deactivation timing
                break;
            }
        }

        delay(50);  // Small delay to prevent rapid iterations
    }

    digitalWrite(relayPin, LOW);  // Deactivate the relay
    Serial.println("Target height reached with fine precision.");
}
