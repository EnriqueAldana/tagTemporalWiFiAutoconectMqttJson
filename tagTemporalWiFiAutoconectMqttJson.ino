// Referencia . https://hieromon.github.io/AutoConnect/index.html
// Modo de uso.
// Dentro de los primeros 30 segundos , el dispositivo tratara de conectarse al ultimo
// Punto de Acceso cuyos datos se encuentran guardados en el EPROM a partir de su ultima conexion exitosa.
// SI la conexion es exitosa, en la salida ydel puerto serial, sera mostrada la IP asignada. Usted podra acceder mediante un navegador a la pagina de inicio.
// SI la conexion no puede ser establecida, el dispositivo se pondra en modo de Access point, busque en su red Wifi un nombre tal como esp8266ap y acceda
// mediante la contraseña 12345678.
// Vaya al menu principal indicado por unas rayitas y busque la opcion AutoConnect
// se le mostrara una lista de Accesos cercanos en funcion de la señal.
// Usted puede elegir uno de ellos y conectar su dispositivo. Se le mostrara una pantalla con la informacion de la conexion.
//de Access Point.
// El acceso al menu de configuracion es mediante la IP Asignada.
// http://192.168.1.87/_aclo
// Referencia Arduino
// https://www.arduino.cc/reference/es/
// Zonas horarias
// Impementacion Date Time
// https://randomnerdtutorials.com/esp8266-nodemcu-date-time-ntp-client-server-arduino/
// Uinformacion Json
// https://arduinojson.org/
// Tipos de datos en Arduino https://programarfacil.com/blog/arduino-blog/tipos-de-datos-en-arduino/
// Video para control de relevador https://www.youtube.com/watch?v=idJDYJ0PtWs
//
// CAUSAS de excepciones https://esp8266-arduino-spanish.readthedocs.io/es/latest/exception_causes.html

// Formato JSON para mensajes.
// suscripcion propia 4c00:7500:2500:b00:6f00:7700
//{
// "device_name": "4c00:7500:2500:b00:6f00:7700",
// "iDCommand": "12345678",
// "command":["VehicularAccessBarrier","UP"]
//}

//{
// "device_name": "4c00:7500:2500:b00:6f00:7700",
// "iDCommand": "12345678",
// "command":["KeepAlive",""]
//}


// Mejoras.
// 1.- Implementar recuperacion de conexion luego de falla mediante FREERTOS https://www.luisllamas.es/como-usar-freertos-en-arduino/
#include <AsyncMqttClient.h>
#include <ESP8266WiFi.h>          // Replace with WiFi.h for ESP32
#include <ESP8266WebServer.h>     // Replace with WebServer.h for ESP32
#include <AutoConnect.h>
#include <ArduinoJson.h>

ESP8266WebServer Server;          // Replace with WebServer for ESP32
AutoConnect      Portal;
AutoConnectConfig Config;

byte mac[6];
Ticker tic_WiFiLed;
static const uint8_t D0   = 16;
static const uint8_t D1   = 5;
static const uint8_t D2   = 4;
static const uint8_t D3   = 0;
static const uint8_t D4   = 2;
static const uint8_t D5   = 14;
static const uint8_t D6   = 12;
static const uint8_t D7   = 13;
static const uint8_t D8   = 15;
static const uint8_t D9   = 3;
static const uint8_t D10  = 1;
static const uint8_t ledWiFi = D4;

String content = "";
String deviceName = "Por definir en funcion del MAC Address";
const uint8_t releUP = D0;
const uint8_t releDOWN = D1;
static const uint8_t accessPosition = D2;
int currentAccessPosition = 0; // Variable temporal para ller el estatus del pin INPUT D2
int lastAccessPosition = 0;
int turnCloseAccess = 0;
int turnOpenAccess = 0;
String lastIdCommand = "";
String currentIdCommand = "";
String lastCommand = "";
String currentCommand = "";
String lastInstruction = "";
String currentInstruction = "";


//======================     Inicia implementacion asincrona  ==================
const IPAddress MQTT_HOST_Async(74, 208, 108, 189);           // Estos datos deben ser configurados por autoConfig  OBLIGATORIO
const int MQTT_PORT_Async = 1883;                             // Estos datos deben ser configurados por autoConfig
const char* MQTT_TOPIC_Async = "tagTemporal";                 // Estos datos deben ser configurados por autoConfig
const char* MQTT_TOPIC_OWN_Async = "4c00:7500:2500:b00:6f00:7700"; // Estos datos deben ser configurados por autoConfig  OBLIGATORIO
AsyncMqttClient mqttClientAsync;


void OnMqttConnectAsync(bool sessionPresent)
{
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  SuscribeMqttAsync();
}
void SuscribeMqttAsync()
{
  uint16_t packetIdSub = mqttClientAsync.subscribe(MQTT_TOPIC_OWN_Async, 0);  // Se suscribe al topico propio
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub);
}

void OnMqttSubscribeAsync(uint16_t packetId, uint8_t qos)
{
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}
String GetPayloadContent(char* data, size_t len)
{
  String content = "";
  for (size_t i = 0; i < len; i++)
  {
    content.concat(data[i]);
  }
  return content;
}
void OnMqttReceivedAsync(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total)
{
  Serial.print("Received on ");
  Serial.print(topic);
  Serial.print(": ");

  String content = GetPayloadContent(payload, len);
  Serial.print(content);
  Serial.println();
}
void OnMqttReceivedAsyncJson(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total)
{
  Serial.println("");
  Serial.print("Json Received on ");
  Serial.print(topic);
  Serial.println(": ");

  String content = "";
  for (size_t i = 0; i < len; i++) {
    content.concat((char)payload[i]);
  }
  Serial.print(content);
  Serial.println();
  char json[] = "{\"device_name\":\"algo\",\"iDCommand\":\"algo\",\"command\":["",""]}";

  DynamicJsonDocument doc(1024);
  deserializeJson(doc, json);
  DeserializationError error = deserializeJson(doc, content);
  if (error) return;

  // Aqui tenemos un doc
  const String device_nameReceived = doc["device_name"];
  const String idCommand = doc["iDCommand"];
  const String command    = doc["command"][0];
  const String instruction   = doc["command"][1];
  Serial.println("device_name:" + device_nameReceived);
  Serial.println("iDCommand: " + idCommand);
  Serial.println("command: " + command);
  Serial.println("instruction: " + instruction);

  // Aqui  ejecutar accion en funcion del commando para el dispositivo propio
  if (deviceName == device_nameReceived) {
    if (command == "VehicularAccessBarrier") {
      currentIdCommand = idCommand;
      if (currentIdCommand != lastIdCommand) {
        Serial.println("Ejecutando comando " + command + " instruccion " + instruction);
        currentCommand = command;
        currentInstruction = instruction;
        // Comando para la pluma del acceso
        if (instruction == "UP") {
          turnOpenAccess = 1;
        } else if (instruction == "DOWN") {
          turnCloseAccess = 1;
        }
      }

    } else if (command == "KeepAlive") {
      String payload = "";                                   // Publica sobre la plataforma
      DynamicJsonDocument doc(1024);
      doc["device_name"] = deviceName;
      doc["currentIdCommand"]   = currentIdCommand;
      doc["command"][0] = currentCommand;
      doc["command"][1] = currentInstruction;

      if (currentAccessPosition) {
        doc["currentAccessPosition"] = "DOWN";
      } else {
        doc["currentAccessPosition"] = "UP";
      }
      //serializeJson(doc, Serial);
      serializeJson(doc, payload);
      mqttClientAsync.publish(MQTT_TOPIC_Async, 0, true, (char*)payload.c_str());   // Publica sobre la plataforma
    }
  }
}


void OnMqttPublishAsync(uint16_t packetId)
{
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}
void PublisMqttAsync(unsigned long data)
{
  String payload = String(data);
  mqttClientAsync.publish(MQTT_TOPIC_Async, 0, true, (char*)payload.c_str());   // Publica sobre la plataforma
  Serial.println("Enviando Id. -> " + deviceName);
}
void PublisMqttAsync(String data)
{
  String payload = String(data);
  mqttClientAsync.publish(MQTT_TOPIC_Async, 0, true, (char*)payload.c_str());   // Publica sobre la plataforma
  Serial.println("Enviando Id. -> " + data);
}

void PublisMqttAsyncJson()
{

  // Reportar estado del dispositivo
  // Hay algun cambio
  if ( (currentAccessPosition != lastAccessPosition) || (currentIdCommand != lastIdCommand)) {
    String payload = "";                                   // Publica sobre la plataforma
    DynamicJsonDocument doc(1024);
    doc["device_name"] = deviceName;
    doc["currentIdCommand"]   = currentIdCommand;
    doc["command"][0] = currentCommand;
    doc["command"][1] = currentInstruction;

    if (currentAccessPosition) {
      doc["currentAccessPosition"] = "DOWN";
    } else {
      doc["currentAccessPosition"] = "UP";
    }
    //serializeJson(doc, Serial);
    serializeJson(doc, payload);
    mqttClientAsync.publish(MQTT_TOPIC_Async, 0, true, (char*)payload.c_str());   // Publica sobre la plataforma
  }


}


void ConnectToMqttAsync()
{
  Serial.println("Connecting to MQTT Async...");
  mqttClientAsync.connect();
}
void InitMqttAsync() {
  mqttClientAsync.onConnect(OnMqttConnectAsync);           // Se suscribe al Propio
  mqttClientAsync.onSubscribe(OnMqttSubscribeAsync);       // Confirma suscripcion con detalles
  mqttClientAsync.onMessage(OnMqttReceivedAsyncJson);         // Recibe sobre el topico suscrito y registra OnMqttReceivedAsyncJson como funcion de recibo
  //mqttClientAsync.onMessage(OnMqttReceivedAsync);         // Recibe sobre el topico suscrito y registra OnMqttReceivedAsync como funcion de recibo
  mqttClientAsync.onPublish(OnMqttPublishAsync);          // Confirma publicador.

  mqttClientAsync.setServer(MQTT_HOST_Async, MQTT_PORT_Async);
}
// ======================  Termina programacion Asincrona


void parpadeoLedWiFi() {
  byte estado = digitalRead(ledWiFi);
  digitalWrite(ledWiFi, !estado);
}
void parpadeaLedWiFi_Inicio() {

  tic_WiFiLed.attach(0.2, parpadeoLedWiFi);
}
void parpadeaLedWiFi_Fin() {
  tic_WiFiLed.detach();
  digitalWrite(ledWiFi, HIGH);
}


void onConnect(IPAddress& ipaddr) {
  Serial.print("WiFi conectado con  ");
  Serial.print(WiFi.SSID());
  Serial.print(", IP:");
  Serial.println(ipaddr.toString());
}



void rootPage() {
  char content[] = "Usted está conectado al WiFi y direccionado a la plataforma de tagTemporal. ";
  Server.send(200, "text/plain", content);
}


void setup() {
  delay(1000);
  pinMode(releUP, OUTPUT);   // D0
  digitalWrite(releUP, HIGH); // Apagamos
  pinMode(releDOWN, OUTPUT);   // D1
  digitalWrite(releDOWN, HIGH); // Apagamos
  pinMode(accessPosition, INPUT);  // Posicion del actuador normalmente abierto = 0
  pinMode(ledWiFi, OUTPUT);   // D4  led de la ESP8266
  Serial.begin(115200);
  Serial.println();
  parpadeaLedWiFi_Inicio();
  // Configuracion autoconnect e reconnect en background
  Config.autoReset = false;     // Not reset the module even by intentional disconnection using AutoConnect menu.
  Config.autoReconnect = true;  // Reconnect to known access points.
  Config.reconnectInterval = 3; // Reconnection attempting interval is 3[min].
  Config.retainPortal = true;   // Keep the captive portal open.
  Portal.config(Config);

  Server.on(" / ", rootPage);

  Portal.onConnect(onConnect);  // Register the ConnectExit function.
  Portal.begin();
  // Fijar la MAC Adress al nombre del dispositivo
  WiFi.macAddress(mac);
  Serial.print("MAC: ");
  Serial.print(mac[0], HEX);
  Serial.print(": ");
  Serial.print(mac[1], HEX);
  Serial.print(": ");
  Serial.print(mac[2], HEX);
  Serial.print(": ");
  Serial.print(mac[3], HEX);
  Serial.print(": ");
  Serial.print(mac[4], HEX);
  Serial.print(": ");
  Serial.println(mac[5], HEX);
  Serial.println("Conexion al WiFi conforme.");
  int int_1 = (byte)mac[0] << 8;
  int int_2 = (byte)mac[1] << 8;
  int int_3 = (byte)mac[2] << 8;
  int int_4 = (byte)mac[3] << 8;
  int int_5 = (byte)mac[4] << 8;
  int int_6 = (byte)mac[5] << 8;
  deviceName = String(int_1, HEX) + ":" + String(int_2, HEX) + ":" + String(int_3, HEX) + ":" + String(int_4, HEX) + ":" + String(int_5, HEX) + ":" + String(int_6, HEX);
  //deviceName = String(HEX) + ":" + String(HEX) + ":" + String(HEX) + ":" + String(HEX) + ":" + String(HEX) + ":" + String(HEX);

  parpadeaLedWiFi_Fin();

  // ============= Inicializacion proceso Asincrono  =======
  InitMqttAsync();
  ConnectToMqttAsync();

}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    // Aqui implementar MQTT
    delay(1000);
    //PublisMqttAsync(millis());
    //PublisMqttAsync(deviceName);
    PublisMqttAsyncJson();
  } else {
    Serial.println("Conexion al WiFi perdida...el dispositivo tratara de reconectarse luego de 3 minutos...");
  }
  // Posicion del actuador
  lastAccessPosition = currentAccessPosition;
  currentAccessPosition = digitalRead(accessPosition);

  // Evaluar instruccion
  if (turnCloseAccess == 1) {
    digitalWrite(releDOWN, LOW);  // Encendido
    // Esperamos poquito para evitar el ruido
    delay(50);
    digitalWrite(releDOWN, HIGH);  // Encendido
    turnCloseAccess = 0;
    lastIdCommand = currentIdCommand;

  }
  if (turnOpenAccess == 1) {
    digitalWrite(releUP, LOW);  // Encendido
    // Esperamos poquito para evitar el ruido
    delay(50);
    digitalWrite(releUP, HIGH);  // Encendido
    turnOpenAccess = 0;
    lastIdCommand = currentIdCommand;
  }



}
