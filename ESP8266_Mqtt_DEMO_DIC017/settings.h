//-------- Customise these values-----------
//---------Bluemix IBM Settings-------------
#define ORG "faltbox"
#define DEVICE_TYPE "ESP:8266"
#define DEVICE_ID "ESPBCODE"
#define USER "demoMQTT"
#define PASS "demoMQTT"
//-------- Customise the above values --------

//-------- Customise these values-----------
//---------Blurmix Topics---------------------

const char publishTopic[] = "TestMQTT/IN";
const char responseTopic[] ="iotdm-1/screen/response";
const char manageTopic[] = "iotdevice-1/mgmt/screen/manage";
const char updateTopic[] = "iotdm-1/device/screen/update";
const char rebootTopic[] = "iotdm-1/mgmt/initiate/device/screen/reboot";
const char varTopic[] = "TestMQTT/OUT";


//Variables de Reloj para espera y envio de paquetes de MQTT
unsigned long publishInterval = 60*1000UL; //Variable configurable remotamente sobre el interbalo de publicacion
unsigned long UPDATESENDTIME = 60*1000UL; //Variable que define el tiempo a trancurrir despues de inicializado el reloj para enviar el primer mensake de MQTT en microsegundos (10*1000UL)= 10segundos
unsigned long UPDATEDISPLAYTIME = 3*60*1000UL; //Variable que define el tiempo a trancurrir despues de inicializado el reloj para enviar el primer mensake de MQTT en microsegundos (10*1000UL)= 10segundos


//-------- Variables de ERROR EN ENVIO de paquetes de MQTT ANTES DE REINICIO
#define FAILTRESHOLD 150
