#define DHT_PIN 4
#define SOIL_PIN 32
#define PUMP_PIN 2
#define SDA_PIN 21
#define SCL_PIN 22
#define LDR_PIN 34

const int DRY_THRESHOLD = 300;
const int WET_THRESHOLD = 700;

uint8_t data[5]; // 5 byte dữ liệu: [Hh, Hl, Th, Tl, checksum]


// I2C for OLED
#define SSD1306_ADDR 0x3C

// Display dimensions
#define WIDTH 128
#define HEIGHT 64
#define PAGES (HEIGHT/8)

uint8_t ssd1306_buffer[WIDTH * PAGES];

// Pins sensors
#define DHT_PIN 4        // DHT22 data pin
#define LDR_PIN 34       // ADC1_CH6 on many ESP32 boards

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

// --- SSD1306 low-level functions ---
void i2c_delay() { delayMicroseconds(5); }

void i2c_sda_high() { pinMode(SDA_PIN, INPUT_PULLUP); }
void i2c_sda_low()  { pinMode(SDA_PIN, OUTPUT); digitalWrite(SDA_PIN, LOW); }
void i2c_scl_high() { pinMode(SCL_PIN, INPUT_PULLUP); }
void i2c_scl_low()  { pinMode(SCL_PIN, OUTPUT); digitalWrite(SCL_PIN, LOW); }

void i2c_start() {
  i2c_sda_high(); i2c_scl_high(); i2c_delay();
  i2c_sda_low(); i2c_delay();
  i2c_scl_low(); i2c_delay();
}

void i2c_stop() {
  i2c_sda_low(); i2c_delay();
  i2c_scl_high(); i2c_delay();
  i2c_sda_high(); i2c_delay();
}

void i2c_writeBit(bool bit) {
  if (bit) i2c_sda_high(); else i2c_sda_low();
  i2c_delay();
  i2c_scl_high(); i2c_delay();
  i2c_scl_low(); i2c_delay();
}

bool i2c_writeByte(uint8_t byte) {
  for (int i = 7; i >= 0; i--) {
    i2c_writeBit((byte >> i) & 1);
  }
  // đọc ACK
  i2c_sda_high();
  i2c_delay();
  i2c_scl_high();
  bool ack = !digitalRead(SDA_PIN);
  i2c_scl_low();
  return ack;
}

//========================= SSD1306 commands =========================//

void ssd1306_command(uint8_t c) {
  i2c_start();
  i2c_writeByte(SSD1306_ADDR << 1);
  i2c_writeByte(0x00); // Co=0, D/C#=0 (command)
  i2c_writeByte(c);
  i2c_stop();
}

//========================= Khởi tạo màn hình =========================//

void ssd1306_init() {
  pinMode(SDA_PIN, OUTPUT);
  pinMode(SCL_PIN, OUTPUT);
  digitalWrite(SDA_PIN, HIGH);
  digitalWrite(SCL_PIN, HIGH);

  delay(100);
  ssd1306_command(0xAE);
  ssd1306_command(0x20); ssd1306_command(0x00);
  ssd1306_command(0xB0);
  ssd1306_command(0xC8);
  ssd1306_command(0x00);
  ssd1306_command(0x10);
  ssd1306_command(0x40);
  ssd1306_command(0x81); ssd1306_command(0x7F);
  ssd1306_command(0xA1);
  ssd1306_command(0xA6);
  ssd1306_command(0xA8); ssd1306_command(0x3F);
  ssd1306_command(0xA4);
  ssd1306_command(0xD3); ssd1306_command(0x00);
  ssd1306_command(0xD5); ssd1306_command(0x80);
  ssd1306_command(0xD9); ssd1306_command(0xF1);
  ssd1306_command(0xDA); ssd1306_command(0x12);
  ssd1306_command(0xDB); ssd1306_command(0x40);
  ssd1306_command(0x8D); ssd1306_command(0x14);
  ssd1306_command(0xAF);

  ssd1306_clear();
  ssd1306_display();
}

//========================= Buffer & hiển thị =========================//

void ssd1306_clear() {
  memset(ssd1306_buffer, 0, sizeof(ssd1306_buffer));
}

void ssd1306_display() {
  for (uint8_t page = 0; page < PAGES; page++) {
    ssd1306_command(0xB0 + page);
    ssd1306_command(0x00);
    ssd1306_command(0x10);

    i2c_start();
    i2c_writeByte(SSD1306_ADDR << 1);
    i2c_writeByte(0x40); // data mode

    for (uint16_t col = 0; col < WIDTH; col++) {
      i2c_writeByte(ssd1306_buffer[page * WIDTH + col]);
    }
    i2c_stop();
  }
}

//========================= Vẽ điểm ảnh =========================//

void ssd1306_drawPixel(int16_t x, int16_t y, bool color) {
  if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) return;
  uint16_t index = (y / 8) * WIDTH + x;
  if (color) ssd1306_buffer[index] |= (1 << (y & 7));
  else ssd1306_buffer[index] &= ~(1 << (y & 7));
}

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

void ssd1306_drawString(int16_t x, int16_t y, const char* s) {
  while (*s) {
    ssd1306_drawChar(x, y, *s++);
    x += 6;
    if (x + 5 >= WIDTH) break;
  }
}

bool readDHT22() {
  uint8_t bits[40];
  uint8_t byteIndex = 0, bitIndex = 7;

  // Reset buffer
  for (int i = 0; i < 5; i++) data[i] = 0;

  // Gửi tín hiệu Start: kéo thấp ít nhất 1ms
  pinMode(DHT_PIN, OUTPUT);
  digitalWrite(DHT_PIN, LOW);
  delay(2);  // 2ms
  digitalWrite(DHT_PIN, HIGH);
  delayMicroseconds(30);
  pinMode(DHT_PIN, INPUT);

  // Đợi phản hồi từ DHT (LOW ~80us, HIGH ~80us)
  unsigned long start = micros();
  while (digitalRead(DHT_PIN) == HIGH) { if (micros()-start>100) return false; }
  start = micros();
  while (digitalRead(DHT_PIN) == LOW) { if (micros()-start>100) return false; }
  start = micros();
  while (digitalRead(DHT_PIN) == HIGH) { if (micros()-start>100) return false; }

  // Đọc 40 bit dữ liệu
  for (int i = 0; i < 40; i++) {
    // mỗi bit bắt đầu bằng xung LOW 50us
    while (digitalRead(DHT_PIN) == LOW);
    unsigned long t = micros();
    while (digitalRead(DHT_PIN) == HIGH);
    if ((micros() - t) > 40) {
      data[byteIndex] |= (1 << bitIndex);  // nếu HIGH lâu hơn 40us thì là 1
    }
    if (bitIndex == 0) { bitIndex = 7; byteIndex++; }
    else bitIndex--;
  }

  // Kiểm tra checksum
  if ((uint8_t)(data[0] + data[1] + data[2] + data[3]) != data[4]) return false;

  return true;
}

void setup() {
  Serial.begin(115200);
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);
  ssd1306_init();
}

void loop() {
  if (readDHT22()) {
    // Ghép giá trị độ ẩm
    int humidity = (data[0] << 8) | data[1];
    int temperature = (data[2] << 8) | data[3];

    float hum = humidity / 10.0;
    float temp = temperature / 10.0;

    // Nếu bit dấu nhiệt độ (bit 15) là 1 thì là số âm
    if (temperature & 0x8000) {
      temp = -( (temperature & 0x7FFF) / 10.0 );
    }

    Serial.print("Nhiet do: ");
    Serial.print(temp);
    Serial.print(" *C, Do am: ");
    Serial.print(hum);
    Serial.println(" %");
    char buf[32];
    snprintf(buf, sizeof(buf), "Nhiet do: %.2f *C", temp);
    ssd1306_drawString(0, 0, buf);
    snprintf(buf, sizeof(buf), "Do am:  %.2f %%", hum);
    ssd1306_drawString(0, 10, buf);
  } else {
    Serial.println("Loi doc DHT22!");
    ssd1306_drawString(0, 0, "DHT read error");
  }
  int ldrValue = analogRead(LDR_PIN);  // đọc giá trị ADC (0 - 4095)
  int lightPercent = map(ldrValue, 0, 4095, 0, 100);
  Serial.print("Gia tri LDR: ");
  Serial.println(ldrValue);
  int soil = analogRead(SOIL_PIN);
  int soilPercent = 100 - map(soil, 0, 4095, 0, 100);
    Serial.print("Do am dat: ");
    Serial.println(soilPercent);
    Serial.println(" %");

    if (soil < DRY_THRESHOLD) {
      digitalWrite(PUMP_PIN, HIGH);
    } else if (soil > WET_THRESHOLD) {
      digitalWrite(PUMP_PIN, LOW);
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

  delay(2000);
}
