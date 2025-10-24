#include <Wire.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"  // In ra thông tin token
#include "addons/RTDBHelper.h"   // In ra thông tin RTDB
#include "DHT.h"

#define SOIL_PIN 34
#define PUMP_PIN 2
#define LIGHT_PIN 15
#define SDA_PIN 21
#define SCL_PIN 22
#define LDR_PIN 35
#define PWMA 16 // bơm
#define PWMB 17 // đèn
#define DHTPIN 25          // Chân DATA của DHT22 nối với GPIO 4 của ESP32
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// 🛜 WiFi
#define WIFI_SSID "realme Q5"
#define WIFI_PASSWORD "duy0109@"

// Trạng thái kết nối
bool wifiConnected = false;

/*
// 🧠 Hàm callback khi có sự kiện WiFi
void WiFiEventHandler(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_START:
      Serial.println("WiFi bắt đầu kết nối...");
      break;

    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println("Đã kết nối tới WiFi!");
      break;

    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.print("Đã nhận IP: ");
      Serial.println(WiFi.localIP());
      wifiConnected = true;
      break;

    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("Mất kết nối WiFi, đang thử kết nối lại...");
      wifiConnected = false;
      WiFi.reconnect();
      break;

    default:
      break;
  }
}
*/

// 🔥 Firebase
#define API_KEY "AIzaSyARpYop_hO0Z7Jf7N478kbT1niQ2eugevg"
#define DATABASE_URL "https://esp32-d4aae-default-rtdb.firebaseio.com/"  // nhớ có "/"

// Cấu trúc Firebase
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

uint8_t data[5]; // 5 byte dữ liệu: [Hh, Hl, Th, Tl, checksum]

// I2C for OLED
#define SSD1306_ADDR 0x3C

// Display dimensions
#define WIDTH 128
#define HEIGHT 64
#define PAGES (HEIGHT/8)

uint8_t ssd1306_buffer[WIDTH * PAGES];

// --- font 5x7 (ASCII 32..127), each char 5 bytes ---
const uint8_t font5x7[] = {
  // font data for 96 chars (32..127) -> 96*5 = 480 bytes
  // This is standard 5x7 font (omitted comments for brevity)
  0x00,0x00,0x00,0x00,0x00, // ' '
  0x00,0x00,0x5F,0x00,0x00, // !
  0x00,0x07,0x00,0x07,0x00, // "
  0x14,0x7F,0x14,0x7F,0x14, // #
  0x24,0x2A,0x7F,0x2A,0x12, // $
  0x23,0x13,0x08,0x64,0x62, // %
  0x36,0x49,0x55,0x22,0x50, // &
  0x00,0x05,0x03,0x00,0x00, // '
  0x00,0x1C,0x22,0x41,0x00, // (
  0x00,0x41,0x22,0x1C,0x00, // )
  0x14,0x08,0x3E,0x08,0x14, // *
  0x08,0x08,0x3E,0x08,0x08, // +
  0x00,0x50,0x30,0x00,0x00, // ,
  0x08,0x08,0x08,0x08,0x08, // -
  0x00,0x60,0x60,0x00,0x00, // .
  0x20,0x10,0x08,0x04,0x02, // /
  0x3E,0x51,0x49,0x45,0x3E, // 0
  0x00,0x42,0x7F,0x40,0x00, // 1
  0x42,0x61,0x51,0x49,0x46, // 2
  0x21,0x41,0x45,0x4B,0x31, // 3
  0x18,0x14,0x12,0x7F,0x10, // 4
  0x27,0x45,0x45,0x45,0x39, // 5
  0x3C,0x4A,0x49,0x49,0x30, // 6
  0x01,0x71,0x09,0x05,0x03, // 7
  0x36,0x49,0x49,0x49,0x36, // 8
  0x06,0x49,0x49,0x29,0x1E, // 9
  0x00,0x36,0x36,0x00,0x00, // :
  0x00,0x56,0x36,0x00,0x00, // ;
  0x08,0x14,0x22,0x41,0x00, // <
  0x14,0x14,0x14,0x14,0x14, // =
  0x00,0x41,0x22,0x14,0x08, // >
  0x02,0x01,0x51,0x09,0x06, // ?
  0x32,0x49,0x79,0x41,0x3E, // @
  0x7E,0x11,0x11,0x11,0x7E, // A
  0x7F,0x49,0x49,0x49,0x36, // B
  0x3E,0x41,0x41,0x41,0x22, // C
  0x7F,0x41,0x41,0x22,0x1C, // D
  0x7F,0x49,0x49,0x49,0x41, // E
  0x7F,0x09,0x09,0x09,0x01, // F
  0x3E,0x41,0x49,0x49,0x7A, // G
  0x7F,0x08,0x08,0x08,0x7F, // H
  0x00,0x41,0x7F,0x41,0x00, // I
  0x20,0x40,0x41,0x3F,0x01, // J
  0x7F,0x08,0x14,0x22,0x41, // K
  0x7F,0x40,0x40,0x40,0x40, // L
  0x7F,0x02,0x0C,0x02,0x7F, // M
  0x7F,0x04,0x08,0x10,0x7F, // N
  0x3E,0x41,0x41,0x41,0x3E, // O
  0x7F,0x09,0x09,0x09,0x06, // P
  0x3E,0x41,0x51,0x21,0x5E, // Q
  0x7F,0x09,0x19,0x29,0x46, // R
  0x46,0x49,0x49,0x49,0x31, // S
  0x01,0x01,0x7F,0x01,0x01, // T
  0x3F,0x40,0x40,0x40,0x3F, // U
  0x1F,0x20,0x40,0x20,0x1F, // V
  0x3F,0x40,0x38,0x40,0x3F, // W
  0x63,0x14,0x08,0x14,0x63, // X
  0x07,0x08,0x70,0x08,0x07, // Y
  0x61,0x51,0x49,0x45,0x43, // Z
  0x00,0x7F,0x41,0x41,0x00, // [
  0x02,0x04,0x08,0x10,0x20, // backslash
  0x00,0x41,0x41,0x7F,0x00, // ]
  0x04,0x02,0x01,0x02,0x04, // ^
  0x40,0x40,0x40,0x40,0x40, // _
  0x00,0x01,0x02,0x04,0x00, // `
  0x20,0x54,0x54,0x54,0x78, // a
  0x7F,0x48,0x44,0x44,0x38, // b
  0x38,0x44,0x44,0x44,0x20, // c
  0x38,0x44,0x44,0x48,0x7F, // d
  0x38,0x54,0x54,0x54,0x18, // e
  0x08,0x7E,0x09,0x01,0x02, // f
  0x0C,0x52,0x52,0x52,0x3E, // g
  0x7F,0x08,0x04,0x04,0x78, // h
  0x00,0x44,0x7D,0x40,0x00, // i
  0x20,0x40,0x44,0x3D,0x00, // j
  0x7F,0x10,0x28,0x44,0x00, // k
  0x00,0x41,0x7F,0x40,0x00, // l
  0x7C,0x04,0x18,0x04,0x78, // m
  0x7C,0x08,0x04,0x04,0x78, // n
  0x38,0x44,0x44,0x44,0x38, // o
  0x7C,0x14,0x14,0x14,0x08, // p
  0x08,0x14,0x14,0x18,0x7C, // q
  0x7C,0x08,0x04,0x04,0x08, // r
  0x48,0x54,0x54,0x54,0x20, // s
  0x04,0x3F,0x44,0x40,0x20, // t
  0x3C,0x40,0x40,0x20,0x7C, // u
  0x1C,0x20,0x40,0x20,0x1C, // v
  0x3C,0x40,0x30,0x40,0x3C, // w
  0x44,0x28,0x10,0x28,0x44, // x
  0x0C,0x50,0x50,0x50,0x3C, // y
  0x44,0x64,0x54,0x4C,0x44, // z
  0x00,0x08,0x36,0x41,0x00, // {
  0x00,0x00,0x7F,0x00,0x00, // |
  0x00,0x41,0x36,0x08,0x00, // }
  0x10,0x08,0x08,0x10,0x08, // ~
  0x00 // padding to reach 480 (not used)
};

void ssd1306_command(uint8_t c) { 
  Wire.beginTransmission(SSD1306_ADDR); 
  Wire.write(0x00); // control byte: Co = 0, D/C# = 0 => next byte is command 
  Wire.write(c); 
  Wire.endTransmission(); 
}
void ssd1306_init() { 
  Wire.begin(SDA_PIN, SCL_PIN); 
  delay(100); 
  ssd1306_command(0xAE); // display off 
  ssd1306_command(0x20); 
  ssd1306_command(0x00); // addressing mode 0 = horizontal 
  ssd1306_command(0xB0); // set page start address 
  ssd1306_command(0xC8); // COM output scan direction 
  ssd1306_command(0x00); // low column addr 
  ssd1306_command(0x10); // high column addr 
  ssd1306_command(0x40); // start line = 0 
  ssd1306_command(0x81); 
  ssd1306_command(0x7F); // contrast 
  ssd1306_command(0xA1); // segment remap 
  ssd1306_command(0xA6); // normal display 
  ssd1306_command(0xA8); 
  ssd1306_command(0x3F); // multiplex 1/64 
  ssd1306_command(0xA4); // output RAM to display 
  ssd1306_command(0xD3); ssd1306_command(0x00); // display offset 
  ssd1306_command(0xD5); ssd1306_command(0x80); // display clock divide 
  ssd1306_command(0xD9); ssd1306_command(0xF1); // pre-charge 
  ssd1306_command(0xDA); ssd1306_command(0x12); // COM pins 
  ssd1306_command(0xDB); ssd1306_command(0x40); // VCOM detect 
  ssd1306_command(0x8D); ssd1306_command(0x14); // charge pump on 
  ssd1306_command(0xAF); // display ON 
  ssd1306_clear(); 
  ssd1306_display(); 
}
void ssd1306_clear() { 
  memset(ssd1306_buffer, 0, sizeof(ssd1306_buffer)); 
}
void ssd1306_display() { // write buffer page by page 
for (uint8_t page = 0; page < PAGES; page++) { 
  ssd1306_command(0xB0 + page); // set page 
  ssd1306_command(0x00); // low col = 0 
  ssd1306_command(0x10); // high col = 0 
  Wire.beginTransmission(SSD1306_ADDR); 
  Wire.write(0x40); // control byte: Co = 0, D/C# = 1 => following are data bytes // send 128 bytes of this page 
  for (uint16_t col = 0; col < WIDTH; col++) { 
    Wire.write(ssd1306_buffer[page * WIDTH + col]); // if Wire buffer limit, endTransmission() and begin again; on ESP32 this often okay 
    if ( (col & 0x1F) == 0x1F ) { // flush every 32 bytes to be safe 
    Wire.endTransmission(); 
    Wire.beginTransmission(SSD1306_ADDR); 
    Wire.write(0x40); 
      } 
    } 
    Wire.endTransmission(); 
  } 
}
// draw pixel in buffer 
void ssd1306_drawPixel(int16_t x, int16_t y, bool color) { 
  if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) return; 
  uint16_t index = (y / 8) * WIDTH + x; 
  if (color) ssd1306_buffer[index] |= (1 << (y & 7)); 
  else ssd1306_buffer[index] &= ~(1 << (y & 7)); 
}
// draw char using 5x7 font 
void ssd1306_drawChar(int16_t x, int16_t y, char c) { 
  if (c < 32 || c > 127) c = '?'; 
  const uint8_t* glyph = &font5x7[(c - 32) * 5]; 
  for (uint8_t i = 0; i < 5; i++) { 
    uint8_t col = glyph[i]; 
    for (uint8_t j = 0; j < 8; j++) { 
      bool pix = col & (1 << j); 
      ssd1306_drawPixel(x + i, y + j, pix); 
      } 
    } 
  }
// write string 
void ssd1306_drawString(int16_t x, int16_t y, const char* s) { 
  while (*s) { ssd1306_drawChar(x, y, *s++); x += 6; // 5px glyph +1 spacing 
  if (x + 5 >= WIDTH) break; 
  } 
}
unsigned long lastCheck = 0;
char bu[32];
char buf[32];
void setup() {
  Serial.begin(9600);
  dht.begin();
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(LIGHT_PIN, OUTPUT);
  ssd1306_init();
  // ✅ Kết nối WiFi
  // Gắn sự kiện WiFi
  //WiFi.onEvent(WiFiEventHandler);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  //while (WiFi.status() != WL_CONNECTED) {}

  ledcAttachPin(PWMA, 0); // Channel 0
      ledcSetup(0, 5000, 8);  // Tần số 5kHz, độ phân giải 8 bit (0-255)
            ledcAttachPin(PWMB, 1);
      ledcSetup(1, 5000, 8);
}

void loop() {
  if((WiFi.status() == WL_CONNECTED) && (wifiConnected == false)){
    wifiConnected == true;
    config.database_url = "https://esp32-d4aae-default-rtdb.firebaseio.com/";
  config.signer.tokens.legacy_token = "jefxIAsgn9YvCc9RASqdfeCgPnbz8SFlHstXAAb8";

  Firebase.begin(&config, &auth);
  }
  else{
    wifiConnected == false;
  }
  if (millis() - lastCheck > 2000) {
  float hum = dht.readHumidity();
  float temp = dht.readTemperature();      // °C
  float f = dht.readTemperature(true);  // °F

  // Kiểm tra lỗi khi đọc dữ liệu
  if (isnan(hum) || isnan(temp) || isnan(f)) {
    Serial.println("❌ Lỗi đọc cảm biến DHT22!");
  }
   Serial.print("Nhiệt độ: ");
  Serial.print(temp);
  Serial.print(" °C  |  Độ ẩm: ");
  Serial.print(hum);
  Serial.print(" %  ");
  snprintf(bu, sizeof(bu), "Nhiet do: %.2f *C", temp);
  snprintf(buf, sizeof(buf), "Do am:  %.2f %%", hum);
  Firebase.RTDB.setInt(&fbdo, "/ESP32/Sensor/Temp", temp);     // Ghi vào Temp
  Firebase.RTDB.setInt(&fbdo, "/ESP32/Sensor/Humid", hum);    // Ghi vào Humid
  }
    
    
    ssd1306_drawString(0, 0, bu);
    ssd1306_drawString(0, 10, buf);

  int ldrValue = analogRead(LDR_PIN);  // đọc giá trị ADC (0 - 4095)
  int lightPercent = map(ldrValue, 0, 4095, 0, 100);
  Serial.print("Gia tri LDR: ");
  Serial.println(ldrValue);
  int soil = analogRead(SOIL_PIN);
  int soilPercent = 100 - map(soil, 0, 4095, 0, 100);
    Serial.print("Do am dat: ");
    Serial.println(soilPercent);
    Serial.println(" %");
  if (Firebase.RTDB.getInt(&fbdo, "/ESP32/OperationMode")) {
      int mode = fbdo.intData();
    Serial.printf("OperationMode: %d\n", mode);
    if(mode == 1){
    //if (soilPercent < 80) {
      digitalWrite(PUMP_PIN, HIGH);
      int x = round(((100 - soilPercent) * 255) / 100.0);
      ledcWrite(0, x);   // PWM cho bơm
      Serial.print("Giá trị bơm:");
      Serial.print(x);
      Firebase.RTDB.setInt(&fbdo, "/ESP32/Actuator/PumpVal", 100 - soilPercent);
      /*
    } else {
      digitalWrite(PUMP_PIN, LOW);
    }*/
    //if (lightPercent < 60){
      digitalWrite(LIGHT_PIN, HIGH);
      int y = round(((100 - lightPercent) * 255) / 100.0);
      ledcWrite(1, y);
      Serial.print("Giá trị đèn:");
      Serial.print(y);
      Firebase.RTDB.setInt(&fbdo, "/ESP32/Actuator/Brightness", 100 - lightPercent);
      /*
    }
    else {
      digitalWrite(LIGHT_PIN, LOW);
    }*/
  }
  else {
    if (Firebase.RTDB.getInt(&fbdo, "/ESP32/Actuator/PumpVal")) {
       int PumpVal = fbdo.intData();
       int z = round((PumpVal * 255) / 100.0);
       digitalWrite(PUMP_PIN, HIGH);
      ledcWrite(0, z);   // PWM cho bơm
    Serial.printf("PumpVal: %d\n", z);
  }
    if (Firebase.RTDB.getInt(&fbdo, "/ESP32/Actuator/Brightness")) {
       int brightness = fbdo.intData();
       int t = round((brightness * 255) / 100.0);
       digitalWrite(LIGHT_PIN, HIGH);
      ledcWrite(1, t);
    Serial.printf("Brightness: %d\n", t);
  }
}
  }
  
  char buf2[32];
  snprintf(buf2, sizeof(buf2), "Light: %d %%", lightPercent);
  ssd1306_drawString(0, 20, buf2);
  char buf3[32];
  snprintf(buf3, sizeof(buf3), "Do am dat: %d %%", soilPercent);
  ssd1306_drawString(0, 30, buf3);

  ssd1306_drawPixel(127, 63, true);
ssd1306_display();
delay(500);
ssd1306_drawPixel(127, 63, false);
ssd1306_display();
delay(500);



   // 📤 Gửi dữ liệu cảm biến
  Firebase.RTDB.setInt(&fbdo, "/ESP32/Sensor/Illum", lightPercent);    // Ghi vào Illum
  Firebase.RTDB.setInt(&fbdo, "/ESP32/Sensor/Pres", soilPercent); 
  
}