#include <esp_now.h>
#include <WiFi.h>
#include <Robojax_L298N_DC_motor.h>

// motor 1 settings
#define CHA 0
#define ENA 19 // this pin must be PWM enabled pin if Arduino board is used
#define IN1 18
#define IN2 5

// motor 2 settings
#define IN3 17
#define IN4 16
#define ENB 4// this pin must be PWM enabled pin if Arduino board is used
#define CHB 1

#define CHANNEL 1

const int CCW = 2; // do not change
const int CW  = 1; // do not change

#define motor1 1 // do not change
#define motor2 2 // do not change

typedef struct struct_message {
    float x;
    float y;
} struct_message;

struct_message MPU6050Readings;

Robojax_L298N_DC_motor robot(IN1, IN2, ENA, CHA,  IN3, IN4, ENB, CHB);

// Initialize ESP-NOW
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  } else {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }
}

// Configure device as an Access Point (AP)
void configDeviceAP() {
  const char *SSID = "Slave_1";
  bool result = WiFi.softAP(SSID, "Slave_1_Password", CHANNEL, 0);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
    Serial.print("AP CHANNEL "); Serial.println(WiFi.channel());
  }
}

// Callback when data is received from Master
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Recv from: "); Serial.println(macStr);

  if (data_len == sizeof(MPU6050Readings)) {
    memcpy(&MPU6050Readings, data, sizeof(MPU6050Readings));
    Serial.print("Received: X = "); Serial.print(MPU6050Readings.x);
    Serial.print(" Y = "); Serial.println(MPU6050Readings.y);
  } else {
    Serial.println("Received data length mismatch");
  }
}

void setup() {
  Serial.begin(115200);
  robot.begin();
  Serial.println("ESPNow/Basic/Slave Example");

  // Set device in AP mode to begin with
  WiFi.mode(WIFI_AP);
  
  // Configure device AP mode
  configDeviceAP();
  
  // Print the MAC address of the Slave in AP Mode
  Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());
  
  // Initialize ESP-NOW
  InitESPNow();
  
  // Register for receive callback to get packet info
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  
  //Forward
  if(MPU6050Readings.x > 0 && MPU6050Readings.y < -4.5)
  { 
  robot.rotate(motor1, 100, CCW);
  robot.rotate(motor2, 100, CCW); 
  }

  //Backward
  else if(MPU6050Readings.x > 0 && MPU6050Readings.y > 3.0)
  { 
  robot.rotate(motor1, 100, CW);
  robot.rotate(motor2, 100, CW); 
  }

  //Right
  else if(MPU6050Readings.x > 3.0 && MPU6050Readings.y < 1.0)
  { 
  robot.rotate(motor1, 100, CCW);
  robot.rotate(motor2, 100, CW); 
  }

  //Left
  else if(MPU6050Readings.x < - 3.0 && MPU6050Readings.y < 1)
  { 
  robot.rotate(motor1, 100, CW);
  robot.rotate(motor2, 100, CCW); 
  }

  //Stop
  else
  {
  robot.brake(1);
  robot.brake(2);  
  }
}
