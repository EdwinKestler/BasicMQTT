///----------------------------------------------------------------------librerias de Librerias de ESP // MQTT/ JSON FORMAT data
#include <ESP8266WiFi.h>                                              //Libreira de ESPCORE ARDUINO
#include <PubSubClient.h>                                             //https://github.com/knolleary/pubsubclient/releases/tag/v2.3
#include <ArduinoJson.h>                                              //https://github.com/bblanchon/ArduinoJson/releases/tag/v5.0.7
#include <TimeLib.h>  
//----------------------------------------------------------------------Librerias de manejo de setup de redes 
#include <ESP8266WebServer.h>                                         //Libreira de html para ESP8266
#include <DNSServer.h>                                                //Libreria de DNS para resolucion de Nombres
#include <WiFiManager.h>    
//----------------------------------------------------------------------Librerias donde se manejan las variables de conexion a servicio
#include "settings.h"                                                 //Libreria local que contiene valores configurables de conexion 
//----------------------------------------------------------------------Poner el Pin de ADC en modo de sensar el voltaje da la bateria
//----------------------------------------------------------------------Variables de verificacion de fallas de capa de conexion con servicio
int failed, sent, published;                                          //Variables de conteo de envios 
//----------------------------------------------------------------------Variables del servicio de envio de datos MQTT
char server[] = "192.168.10.58";       //EL ORG es la organizacion configurada para el servicio de Bluemix
//char authMethod[] = "use-token-auth";                                 //Tipo de Autenticacion para el servicio de Bluemix (la calve es unica por cada nodo)
//char token[] = TOKEN;                                                 //Variable donde se almacena el Token provisto por el servicio (ver Settings.h)
char clientId[] = "d:" ORG ":" DEVICE_TYPE ":" DEVICE_ID;             //Variable de Identificacion de Cliente para servicio de MQTT Bluemix 
//----------------------------------------------------------------------Declaracion de Variables Globales (procuar que sean las minimas requeridas.)
int FWVERSION = 1;                                                    //Variable configurable remotamente sobre la vesion e firmware
int SleepState = 0;
int rotation =1;
unsigned long lastPublishMillis, lastDisplayMillis;                                      //Variable para llevar conteo del tiempo desde la ultima publicacion 
//----------------------------------------------------------------------declaracion de luces

int luzVerde = 12;
int luzAzul  = 13;
int luzRoja =  14;

//----------------------------------------------------------------------Variables Propias del CORE ESP8266 Para la administracion del Modulo
String NodeID = String(ESP.getChipId());                              //Variable Global que contiene la identidad del nodo (ChipID) o numero unico
//----------------------------------------------------------------------Funcion remota para mandar a dormir el esp despues de enviar un RFID
void handleVar (byte* payloadrsp) {
  StaticJsonBuffer<200> jsonBuffer;                                   //Se establece un Buffer de 1o suficientemente gande para almacenar los menasajes JSON
  JsonObject& root = jsonBuffer.parseObject((char*)payloadrsp);       //Se busca la raiz del mensaje Json convirtiendo los Bytes del Payload a Caracteres en el buffer
  if (!root.success()) {                                              //Si no se encuentra el objeto Raiz del Json
    Serial.println(F("ERROR en la Letura del JSON Entrante"));        //Se imprime un mensaje de Error en la lectura del JSON
    return;                                                           //Nos salimos de la funcion
  }                                                                   //se cierra el condicional
  Serial.println(F("Json recibido:"));                                //si se pudo encontrar la raiz del objeto JSON se imprime u mensje
  root.prettyPrintTo(Serial);                                         //y se imprime el mensaje recibido al Serial
  Serial.println();                                                   //dejamos una linea de pormedio para continuar con los mensajes de debugging
  const char* new_msg = root["Mensaje"];                                   //se indica donde buscar el valor a insertar dentro de la lista de codigos
  Serial.print(F("Nuevo Mensaje:"));                                       //se imprime un mensaje con ka variable que acaba de modificarse remotamente
  Serial.println(new_msg);                                           //se imprime el nuevo valor de la variable actualizada    
  delay(1); // Allow ESP8266 to handle watchdog & WiFi stuff
  String Mensaje((char*)new_msg);
  if(Mensaje == "Rojo"){
      digitalWrite(luzRoja, HIGH);   // turn the LED on (HIGH is the voltage level)
      digitalWrite(luzAzul, LOW);   // turn the LED on (HIGH is the voltage level)
      digitalWrite(luzVerde, LOW);   // turn the LED on (HIGH is the voltage level)
      Serial.println("encendiendo luz roja");                                           //se imprime el nuevo valor de la variable actualizada
  }
  if(Mensaje == "Azul"){
      digitalWrite(luzAzul, HIGH);   // turn the LED on (HIGH is the voltage level)
      digitalWrite(luzRoja, LOW);   // turn the LED on (HIGH is the voltage level)
      digitalWrite(luzVerde, LOW);   // turn the LED on (HIGH is the voltage level)
  }
  if(Mensaje == "Verde"){
      digitalWrite(luzVerde, HIGH);   // turn the LED on (HIGH is the voltage level)
      digitalWrite(luzRoja, LOW);   // turn the LED on (HIGH is the voltage level)
      digitalWrite(luzAzul, LOW);   // turn the LED on (HIGH is the voltage level)
  } 
  else if(Mensaje == "off"){
      digitalWrite(luzRoja, LOW);   // turn the LED on (HIGH is the voltage level)
      digitalWrite(luzVerde, LOW);   // turn the LED on (HIGH is the voltage level)
      digitalWrite(luzAzul, LOW);   // turn the LED on (HIGH is the voltage level)
  }
}
//----------------------------------------------------------------------Funcion de vigilancia sobre mensajeria remota desde el servicion de IBM bluemix
void callback(char* topic, byte* payload, unsigned int payloadLength){//Esta Funcion vigila los mensajes que se reciben por medio de los Topicos de respuesta;
  Serial.print(F("callback invoked for topic: "));                    //Imprimir un mensaje señalando sobre que topico se recibio un mensaje
  //Serial.print(F("callback invoked for topic: ")); 
  Serial.println(topic);                                              //Imprimir el Topico
  //Serial.println(topic); 
  
  if (strcmp (varTopic, topic) == 0) {                            //verificar si el topico conicide con el Topico responseTopic[] definido en el archivo settings.h local
    handleVar(payload);
    Serial.println(F("vartopic"));
    //return; // just print of response for now                         //Hacer algo si conicide (o en este caso hacer nada)
  }
  
  if (strcmp (rebootTopic, topic) == 0) {                             //verificar si el topico conicide con el Topico rebootTopic[] definido en el archivo settings.h local
    Serial.println(F("Rebooting..."));                                //imprimir mensaje de Aviso sobre reinicio remoto de unidad.
    ESP.restart();                                                    //Emitir comando de reinicio para ESP8266
  }
}

//----------------------------------------------------------------------definicion de Cliente WIFI para ESP8266 y cliente de publicacion y subcripcion
WiFiClient wifiClient;                                                //Se establece el Cliente Wifi
PubSubClient client(server, 1883, callback, wifiClient);              //se establece el Cliente para el servicio MQTT
//----------------------------------------------------------------------Funcion de Conexion a Servicio de MQTT
void mqttConnect() {
  if (!!!client.connected()) {                                         //Verificar si el cliente se encunetra conectado al servicio
    Serial.print(F("Reconnecting MQTT client to "));                    //Si no se encuentra conectado imprimir un mensake de error y de reconexion al servicio
    Serial.println(server);
    char charBuf[30];
    String CID (clientId + NodeID);
    CID.toCharArray(charBuf, 30);
    //if (!!!client.connect(clientId, USER, PASS)) {                                //Si no se encuentra conectado al servicio intentar la conexion con las credenciales Clientid, Metodo de autenticacion y el Tokeno password
   if (!!!client.connect(clientId, USER, PASS)) {                                //Si no se encuentra conectado al servicio intentar la conexion con las credenciales Clientid, Metodo de autenticacion y el Tokeno password

      int i = 0;
    if ( i < 60){
       Serial.print(F("."));
       delay (500);
       i++;
    }else if (i=60){
       Serial.print(F("Conection Lost 30s+: Resetting"));
       delay (500);
       i++;
    }else if (i> 60){
       ESP.reset();
    } 
   }
  Serial.println();                                                   //dejar un espacio en la terminal para diferenciar los mensajes.
 }
}

//----------------------------------------------------------------------Funcion encargada de subscribir el nodo a los servicio de administracion remota y de notificar los para metros configurables al mismo
void initManagedDevice() {
  if (client.subscribe(varTopic)) {                         //Subscribir el nodo al servicio de mensajeria de respuesta
    Serial.println(F("subscribe to varTopic OK"));                   //si se logro la sibscripcion entonces imprimir un mensaje de exito
    Serial.println(F("subscribe to varTopic:OK"));                   //si se logro la sibscripcion entonces imprimir un mensaje de exito
  }
  else {
    Serial.println(F("subscribe to varTopic FAILED"));               //Si no se logra la subcripcion imprimir un mensaje de error
    Serial.println(F("subscribe to varTopic:FAILED"));               //Si no se logra la subcripcion imprimir un mensaje de error
  }
  
  if (client.subscribe(rebootTopic)) {                                //Subscribir el nodo al servicio de mensajeria de reinicio remoto
    Serial.println(F("subscribe to reboot OK"));                      //si se logro la sibscripcion entonces imprimir un mensaje de exito
    Serial.println(F("subscribe to reboot OK"));                      //si se logro la sibscripcion entonces imprimir un mensaje de exito
  }
  else {
    Serial.println(F("subscribe to reboot FAILED"));                  //Si no se logra la subcripcion imprimir un mensaje de error                
    Serial.println(F("subscribe to reboot:FAILED"));                  //Si no se logra la subcripcion imprimir un mensaje de error                
 }
  
  if (client.subscribe(responseTopic)) {                    //Subscribir el nodo al servicio de mensajeria de reinicio remoto
    Serial.println(F("subscribe to responseTopic OK"));                      //si se logro la sibscripcion entonces imprimir un mensaje de exito
    Serial.println(F("subscribe to responseTopic:OK"));                      //si se logro la sibscripcion entonces imprimir un mensaje de exito
 }
  else {
    Serial.println(F("subscribe to responseTopic FAILED"));                  //Si no se logra la subcripcion imprimir un mensaje de error         
    Serial.println(F("subscribe to responseTopic:FAILED"));                  //Si no se logra la subcripcion imprimir un mensaje de error         
  }

  StaticJsonBuffer<1536> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  JsonObject& d = root.createNestedObject("d");
  JsonObject& metadata = d.createNestedObject("metadata");
  metadata["publishInterval"] = publishInterval;
  JsonObject& supports = d.createNestedObject("supports");
  supports["deviceActions"] = true;  
  char buff[1536];
  root.printTo(buff, sizeof(buff));
  Serial.println(F("publishing device metadata:"));
  Serial.println(buff);
  
  if (client.publish(manageTopic, buff)) {
    Serial.println(F("device Publish ok"));
    Serial.println(F("device Publish METADATA:OK"));
  }else {
    Serial.println(F("device Publish failed:"));
    Serial.println(F("device Publish METADATA:FAILED"));
 }
}

//--------  anager function. Configure the wifi connection if not connect put in mode AP--------//
void wifimanager() {
  WiFiManager wifiManager;
  Serial.println(F("empezando"));
  Serial.println(F("Wifi Manager: empezando"));
  if (!  wifiManager.autoConnect("flatwifi")) {
    if (!wifiManager.startConfigPortal("FlatWifi")) {
      //reset and try again, or maybe put it to deep sleep
      ESP.reset();
      delay(5000);
    }
  }
}
//------------------------------- setup  ---------------------------------------------//
//-------- Funcion Principal de inicializacion de rutina en modulo 
void setup() {
  pinMode(luzRoja, OUTPUT);
  pinMode(luzAzul, OUTPUT);
  pinMode(luzVerde, OUTPUT);

  Serial.begin(115200);
  Serial.println(F("initializing AUTH MQTT Setup"));
   //------------------------------------------------------ Funcion de Conexion a Wifi
  while (WiFi.status() != WL_CONNECTED) {//conectamos al wifi si no hay la rutina iniciara una pagina web de configuracion en la direccion 192.168.4.1 
    wifimanager();
    delay(1000);
  }
  Serial.print(F("nWiFi connected, IP address: "));
  Serial.println(WiFi.localIP());
  Serial.println();//dejamos una linea en blanco en la terminal 
 
  Serial.println(F("Connecting to mqtt sevrer"));
  //------------------------------------------------------ Funcion de Conexion a MQTT
  mqttConnect();//Conectamos al servicio de Mqtt con las credenciales provistas en el archivo "settings.h"
  Serial.println(F("Mqtt Connection Done!, sending Device Data"));
  //------------------------------------------------------ Funcion de Conexion a SERVICIO DE PUBSUB
  initManagedDevice();//inciamos la administracion remota desde Bluemix
  Serial.println(F("Finalizing Setup")); //enviamos un mensaje de depuracion 
  delay(100);
}

//-------- publishData function. Publish the data to MQTT server, the payload should not be bigger than 45 characters name field and data field counts. --------//
void publishManageData (String Sid0, int env, int fail){
  float vdd = ESP.getVcc()/1000 ;
  StaticJsonBuffer<1024> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  JsonObject& d = root.createNestedObject("d");
  JsonObject& metadata = d.createNestedObject("metadata");
  metadata["device_name"] = Sid0;
  metadata["Bateria"] = vdd;
  metadata["enviados"] = env;
  metadata["fallidos"] = fail;
  char buff[1024];
  root.printTo(buff, sizeof(buff));
  Serial.println("publishing device metadata:");
  Serial.println(buff);
  if (client.publish(manageTopic, buff)) {
    Serial.println("Manage Publish ok");
     published ++;
     failed = 0; 
  }else {
    Serial.print(F("Manage Publish failed:"));
    failed ++;
  }
}

void loop(void) {
 
    // VERIFICAMOS CUANTAS VECES NO SE HAN ENVIOADO PAQUETES (ERRORES)
   if (failed >= FAILTRESHOLD){
    failed =0;
    published =0;
    sent=0;    
    ESP.reset();
  }
  
   //si ha pasado el tiempo establecido entonces publicar los datos
  if(millis() - lastPublishMillis > UPDATESENDTIME) {
    String Msg = String ("MSGfailed" + failed); 
    Serial.println(Msg);
    publishManageData(NodeID, published, failed);
    UPDATESENDTIME = 30*60*1000UL; 
    lastPublishMillis = millis(); //Actulizar la ultima hora de envio
  }
  
  //verificar que el cliente de Conexion al servicio se encuentre conectado
  if (!client.loop()) {
    mqttConnect();// si el clinete al servicio MQTT se desconecto volver a conectarlo.
  }
}
