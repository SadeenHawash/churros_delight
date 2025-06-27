#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>

// --- إعدادات الواي فاي ---
const char* ssid = "";
const char* password = "";

// --- إعدادات HiveMQ Cloud ---
const char* mqttServer = "";
const int mqttPort = ;  // منفذ TLS
const char* mqttUser = "";    // نفس المستخدم من HiveMQ Cloud
const char* mqttPassword = "";

// --- موضوع الـ MQTT ---
const char* topic = "machine/package-size";

// --- تعريف دبابيس Serial1 بين ESP32 و Arduino ---
const int RXPin = 16;  // ESP32 RX
const int TXPin = 17;  // ESP32 TX

// --- كائنات ---
WiFiClientSecure wifiClient;   // اتصال آمن TLS
PubSubClient client(wifiClient);

// --- دالة استقبال الرسائل ---
void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.print("MQTT message received: ");
  Serial.println(message);

  // أرسل الحرف الأول فقط للأردوينو إذا كانت الرسالة S أو M أو L
  if (message == "S" || message == "M" || message == "L") {
    Serial1.write(message[0]);
  }
}

// --- اتصال بالواي فاي ---
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
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

// --- إعادة اتصال بالـ MQTT ---
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client", mqttUser, mqttPassword)) {
      Serial.println("connected");
      client.subscribe(topic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, RXPin, TXPin);
  setup_wifi();

  wifiClient.setInsecure();           // 1- تجاوز التحقق من الشهادة
  client.setServer(mqttServer, mqttPort);  // 2- إعداد السيرفر
  client.setCallback(callback);  
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}
