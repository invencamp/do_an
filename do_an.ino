#define DHT_PIN 22
#define SOIL_PIN 32
#define PUMP_PIN 2

const int DRY_THRESHOLD = 300;
const int WET_THRESHOLD = 700;

uint8_t data[5]; // 5 byte dữ liệu: [Hh, Hl, Th, Tl, checksum]
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
  Serial.println("Bat dau chuong trinh");
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
  } else {
    Serial.println("Loi doc DHT22!");
  }
  int soil = analogRead(SOIL_PIN);
    Serial.print("Do am dat: ");
    Serial.println(soil);

    if (soil < DRY_THRESHOLD) {
      digitalWrite(PUMP_PIN, HIGH);
    } else if (soil > WET_THRESHOLD) {
      digitalWrite(PUMP_PIN, LOW);
    }
  delay(2000);
}
