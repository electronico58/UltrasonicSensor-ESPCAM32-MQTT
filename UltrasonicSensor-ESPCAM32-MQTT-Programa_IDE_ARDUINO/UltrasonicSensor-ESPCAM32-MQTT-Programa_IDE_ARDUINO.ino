/* ********************************************************************
 * 
 *      PROGRAMA PARA MANEJAR EL SENSOR ULTRASONICO "HC-SR04"
 *                CON LA TARJETA "ESP32-CAM"
 * 
 * ********************************************************************
 *       Diplomado IoT                     octubre - 2021
 * ********************************************************************
 * 
 * Descripción:
 * 1. Este programa es una mezcla de un ejemplo contenido
 *    en la biblioteca "Ultrasonic" de "Eric Simoes"
 *    y el programa "Conexión básica por MQTT del NodeMCU"
 *    realizado por Hugo Escalpelo (28-julio-2021), que se
 *    encuentra en el REPOSITORIO (GitHub) de Codigo IoT
 * 
 * 2. Este programa envía datos  por Internet a través del 
 *    protocolo MQTT. 
 *    Para poder comprobar el funcionamiento de este programa, 
 *    es necesario conectarse a un broker y usar NodeRed para 
 *    visualzar que la información se está recibiendo correctamente.
 *    Este programa no requiere componentes adicionales.
 * 
 *    Componente      PinESP32CAM       Estados lógicos
 *    ledStatus--------GPIO 33---------On=>LOW, Off=>HIGH
 *    ledFlash---------GPIO 4----------On=>HIGH, Off=>LOW
 *    
 *    HC-SR04-VCC------VCC-FTDI
 *    HC-SR04-GND------GND-FTDI
 *    
 *    ESPCAM32    ->  HC-SR04
 *    GPIO(15,14) ->  (Trig, Echo)
 *    GPIO15----------HC-SR04-Trig
 *    GPIO14----------HC-SR04-Echo
 * 
 * ******************************************************************/

 /* -----------------------------------------------------------------
  *  
  *  ESTRUCTURA DEL PROGRAMA:
  *  
  *  100 - void setup()
  *        110 - void callback()                  <-- RECEPCION msgs
  *  200 - void loop()
  *        210 - void verificar_conexion_broker()
  *             215 - void reconnect()
  *        220 - void envio_mensajes_mqtt()       <-- ENVIO msgs
  *        
  * ---------------------------------------------------------------*/

/**********************************
 *          BIBLIOTECAS
 * *******************************/
#include <WiFi.h>           // Control de WiFi
#include <PubSubClient.h>   // Conexion MQTT
#include <Ultrasonic.h>     // Sensor Ultrasonic

/**********************************
 *   DATOS para CONEXION A WiFi
 * *******************************/
const char* ssid = "INFINITUMBF13_2.4";  // Nombre de la RED
const char* password = "0007213411";     // Contraseña

/**********************************
 *    DATOS del BROKER MQTT
 * *******************************/

/*---------------------------------------------------*/
/*     PARA PROBAR EN RED LOCAL con BROKER local     */
/*          IP asignada ó IP publica                 */
/*            localhost -> 127.0.0.1                 */
/*       Equipo LOCAL corrindo "mosquitto"           */
/*---------------------------------------------------*/
//const char* mqtt_server = "127.0.0.1"; 
//IPAddress server(127,0,0,1);

/*---------------------------------------------------*/
/*           PARA PROBAR con BROKER remoto           */
/*     IP obtenida con: $ nslookup boker.hivemq.com  */
/*---------------------------------------------------*/
const char* mqtt_server = "3.122.36.163"; // IP del 6 oct 2021
IPAddress server(3,122,36,163);
//const char* mqtt_server = "127.0.0.1"; // IP del 6 oct 2021
//IPAddress server(127,0,0,1);

/**********************************
 *    Declaracion de VARIABLES
 * *******************************/
int flashLedPin = 4;      // Controlado por MQTT
int statusLedPin = 33;    // Estatus de conexión
long timeNow, timeLast;   // Variables de control de tiempo no bloqueante
int Distancia = 0;         // Distancia del sensor ultrasonico
int wait = 5000;          // Espera cada 5 segundos para envío de mensajes MQTT
int pinTrigger = 15;      // GPIO15 <- Trigger del UltrasonicSensor
int pinEcho = 14;         // GPIO14 <- Echo del UltrasonicSensor

/* ********************************************************************
 *                Declaracion de OBJETOS:
 *    
 * espClient         <- Objeto para manejo de conexion WiFi
 * client(espClient) <- Objeto para manejo del BROKER
 * ultrasonic(args)  <- Objeto para manejo del sesnor Ultrasonic
 * *******************************************************************/
WiFiClient espClient; 
PubSubClient client(espClient); 
Ultrasonic ultrasonic(pinTrigger, pinEcho); 

/* ********************************************************************
 *  
 *                    100 - void setup()
 *                    
 * *******************************************************************/
void setup() {

  Serial.begin (115200);
  
  /* - 1 ----------------------------------------------------
   *               Inicializacion de LED's 
   * ------------------------------------------------------*/
  pinMode (flashLedPin, OUTPUT);
  pinMode (statusLedPin, OUTPUT);
  digitalWrite (flashLedPin, LOW);
  digitalWrite (statusLedPin, HIGH);

  Serial.println();
  Serial.println();
  Serial.print("Conectar a ");
  Serial.println(ssid);
  
  /* - 2 ----------------------------------------------------
   *               Iniciar Conexion a WiFi 
   * ------------------------------------------------------*/                 
  WiFi.begin(ssid, password); 
  /* --- Espera a que se establezca la conexion a WiFi --- */
  while (WiFi.status() != WL_CONNECTED) { 
    digitalWrite (statusLedPin, HIGH);
    /* --- dado que es de suma importancia esperar 
        a la conexión, debe usarse espera bloqueante --- */
    delay(500); 
    digitalWrite (statusLedPin, LOW);
    /* --- Indicador de progreso --- */
    Serial.print(".");  
    delay (5);
  }
  
  /* - 3 ----------------------------------------------------
   *             Conexion a WiFi Establecida
   * ------------------------------------------------------*/
  Serial.println();
  Serial.println("WiFi conectado");
  Serial.println("Direccion IP: ");
  Serial.println(WiFi.localIP());
  /* --- Si se logro la conexión, encender statusLED --- */
  if (WiFi.status () > 0){
  digitalWrite (statusLedPin, LOW);
  }

  /* --- Formalidad antes de iniciar la comunicación con el BROKER --- */
  delay (1000); 

  /* - 4 ----------------------------------------------------
   *               Iniciar Conexion al BROKER 
   *               
   *     OBJETO:         server
   *     IP del BROKER:  3,122,36,163
   *                     IP obtenida con: 
   *                     $ nslookup boker.hivemq.com
   *     TIPO de OBJETO: IPAdress
   *     PUERTO:         1883 -> tcp MQTT protocol
   *     
   *     COMANDO:        IPAddress server(3,122,36,163);
   * ------------------------------------------------------*/
   /* --- Conectarse a la IP del Broker en el puerto indicado --- */
  client.setServer(server, 1883); 
  /* --- Activar función de CallBack, permite recibir mensajes 
            MQTT y ejecutar funciones a partir de ellos       --- */
  client.setCallback(callback);
  /* --- Esta espera es preventiva, espera a la conexión 
                para no perder información                    --- */
  delay(1500);  // 

  /* - 5 ----------------------------------------------------
   *             Inicia el Control de Tiempo
   * ------------------------------------------------------*/
  timeLast = millis ();
}


/* ********************************************************************
 *  
 *                      200 - void loop()
 *                      
 * *******************************************************************/
void loop() {

  /* -------------------------------------------------- *
   *     Verificar siempre que haya conexión al broker  *
   * ---------------------------------------------------*/
  verificar_conexion_broker();

  /* -------------------------------------------------- *
   *     Envío de mensajes por MQTT cada 5 segundos     *
   *           (wait = 5000 -> 5 segundos)              *
   * ---------------------------------------------------*/
  envio_mensajes_mqtt();
}


/* ********************************************************************
 *  
 *               210 - void verificar_conexion_broker()
 *                      
 * *******************************************************************/
void verificar_conexion_broker() {
  if (!client.connected()) {
    /* En caso de que no haya conexión,   *
     * ejecutar la función de reconexión, *
     * definida despues del void setup () */
    reconnect();  // 
  }

  /* --------------------------------------------------------
   *     Ejecutar de manera no bloqueante las funciones 
   *     necesarias para la comunicación con el broker 
   * ------------------------------------------------------*/  
  client.loop(); 
}


/* ********************************************************************
 *  
 *                  220 - void envio_mensajes_mqtt()
 *                          ENVIO DE MENSAJES MQTT
 *                      
 * *******************************************************************/
 void envio_mensajes_mqtt() {
  /* --------------------------------------------------------
   *     Envío de mensajes por MQTT cada 5 segundos 
   *              (wait = 5000 -> 5 segundos)
   * ------------------------------------------------------*/
  timeNow = millis();
  if (timeNow - timeLast > wait) { 
    /* --- Se cumplieron los 5 segundos y se actualiza
        la variable "timeLast" para seguimiento del tiempo --- */
    timeLast = timeNow;

    /* --- Lectura del SENSOR ULTRASONICO --- */
    Distancia = ultrasonic.read();

    /* --- Define un arreglo de caracteres para enviarlos 
            por MQTT. Especifica la longitud del mensaje
                     en 8 caracteres                      --- */
    char dataString[8];
    /* --- Esta es una función nativa de leguaje AVR que 
             convierte un arreglo de caracteres en una
                     variable String                      --- */
    dtostrf(Distancia, 1, 2, dataString);
    /* --- Se imprime en monitor solo para poder
             visualizar que el evento sucede              --- */
    Serial.print("Distancia: ");
    Serial.println(dataString);
    /* ---  Esta es la función que envía los datos por MQTT 
                Especifica el tema y el valor             --- */
    client.publish("codigoiot/distancia/fernandoramirez", dataString);
  }
 }


/* ********************************************************************
 *  
 *                      110 - void callback 
 *                    RECEPCION DE MENSAJES MQTT      
 *                      
 * *******************************************************************/
/* -------------------------------------------------------------------
 *   Esta función permite tomar acciones en caso de que se reciba un 
 *  mensaje correspondiente a un tema al cual se hará una suscripción
 * ----------------------------------------------------------------- */
void callback(char* topic, byte* message, unsigned int length) {

  /* --- Indicar por serial que llegó un mensaje --- */
  Serial.print("Llegó un mensaje en el tema: ");
  Serial.print(topic);

  /* --- Concatenar los mensajes recibidos para 
          conformarlos como una variable String.
          Se declara la variable en la cual se
          guardará el mensaje completo recibido  --- */
  String messageTemp;
  /* --- Se imprime y construye el mensaje recibido --- */
  for (int i = 0; i < length; i++) {  // Se imprime y concatena el mensaje
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }

  /* --- Se comprueba que el mensaje se haya concatenado correctamente --- */
  Serial.println();
  Serial.print ("Mensaje RECIBIDO concatenado en una sola variable: ");
  Serial.println (messageTemp);

  /* --------------------------------------------------------
   *      En esta parte puedes agregar las funciones que 
   *   requieras para actuar segun lo necesites al recibir 
   *                 un mensaje MQTT
   * ------------------------------------------------------*/

  /* --------------------------------------------------------
   *  Ejemplo: 
   *    En caso de recibir el mensaje true - false, 
   *    se cambiará el estado del LED soldado en la placa. 
   *    int statusLedPin = 33 <- Controlado por MQTT
   *    
   *    El ESP323CAM está suscrito al tema:
   *          "codigoiot/respuesta/fernandoramirez"
   * ------------------------------------------------------*/
  /* --- En caso de recibirse mensaje en el tema: 
              "codigoiot/respuesta/fernandoramirez"
                 ENCENDER el LED "flashLedPin"         --- */
  if (String(topic) == "codigoiot/respuesta/fernandoramirez") {
    if(messageTemp == "true"){
      Serial.println("Led encendido");
      digitalWrite(flashLedPin, HIGH);
    }
  /* --- En caso de NO recibirse mensaje en el tema: 
              "codigoiot/respuesta/fernandoramirez"
                 APAGAR el LED "flashLedPin"          --- */
    else if(messageTemp == "false"){
      Serial.println("Led apagado");
      digitalWrite(flashLedPin, LOW);
    }
  }
}


/* ********************************************************************
 *  
 *                      215 - void reconnect()
 *                      
 * *******************************************************************/
void reconnect() {

  /* -----------------------------------------------------------------
   *    Bloque de ESPERA hasta lograr la CONEXION con el BROKER
   * -----------------------------------------------------------------*/
  /* --- Espera hasta lograr conexión --- */
  while (!client.connected()) { 
    /* --- Mientras no exista CONEXION con el BROKER --- */
    Serial.print("Tratando de conectarse con el BROKER...");

    /* --- Pregunta por el resultado del intento de conexión --- */
    if (client.connect("ESP32CAMClient")) { 
      /* --- Se ESTABLECIO la conexion con el BROKER --- */
      Serial.println("Conectando con el BROKER...");
      /* --- El ESP32CAM se SUSCRIBE al tema: "codigoiot/respuesta/fernandoramirez" --- */
      client.subscribe("codigoiot/respuesta/fernandoramirez"); 
    }
    
    /* --- NO se logró establecer la conexion con el BROKER --- */
    else {  
      Serial.print("Conexion fallida con el BROKER, Error rc=");
      /* --- Se muestra el código de error --- */
      Serial.print(client.state()); 
      Serial.println(" Volviendo a intentar en 5 segundos");
      /* --- Espera de 5 segundos bloqueante --- */
      delay(5000);
      /* --- Muestra el estatus de conexion --- */
      Serial.println (client.connected ()); 
    }
    
  }
  
}
