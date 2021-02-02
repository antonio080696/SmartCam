#include <Arduino.h>
#include <FunctionalInterrupt.h>
#include "esp_camera.h"
#include <WiFi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <PubSubClient.h>

#define BUTTON1 12
#define BUTTON2 17


WiFiClient espClient;
PubSubClient client(espClient);
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

//Conexión con WiFi y servidores.

const char* ssid = "raspi-webgui";
const char* password = "ChangeMe";

String serverName = "10.3.141.1"; //Se utiliza para la conexion con el servidor HTTP
const char* mqtt_server = "10.3.141.1"; //Se utiliza para la conexion con el servidor MQTT

String serverPath = "/upload.php";     //Script encargado de recoger la imagen en el servidor 
const int serverPort = 80;


const int timerInterval = 2000;    // time between each HTTP POST image
unsigned long previousMillis = 0;   // last time image was 
const int Tinicio = 5000;

int capturaFoto = 0;
int primeraIteracion = 0;
int FotoRealizada = 0;
int recepcionLista = 0;
char vectorChar[3];//Para convertir un string en un array

//Creacion de la clase para la gestion de la interrupcion.
class Button
{
public:
	Button(uint8_t reqPin) : PIN(reqPin){
		pinMode(PIN, INPUT);
    // se activa cuando haya cambio de low a high
    attachInterrupt(PIN, std::bind(&Button::isr,this),  RISING);
		//attachInterrupt(PIN, std::bind(&Button::isr,this), FALLING);
	};
	~Button() {
		detachInterrupt(PIN);
	}

	void IRAM_ATTR isr() {
		 pressed = true;
	}

	void checkPressed() {
		if (pressed) {
      capturaFoto = 1;
			//Serial.printf("Button on pin %u has been pressed %u times\n", PIN, numberKeyPresses);
			pressed = false;
		}else{
      capturaFoto = 0;
		}
	}

private:
	  const uint8_t PIN;
    volatile bool pressed;
};

Button button1(BUTTON1);



long lastMsg = 0;
char msg[50];
int value = 0;

//Inicializamos todos los parametros necesarios
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);
    
  WiFi.mode(WIFI_STA);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);  
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  
  Serial.println();
  Serial.print("ESP32-CAM IP Address: ");
  Serial.println(WiFi.localIP());

  // Realizamos la configuracion de la camara
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // init with high specs to pre-allocate larger buffers
  if(psramFound()){
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 10;  //0-63 lower number means higher quality
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_CIF;
    config.jpeg_quality = 12;  //0-63 lower number means higher quality
    config.fb_count = 1;
  }
  
  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    delay(1000);
    ESP.restart();
  }
  delay(6000);
  //sendPhoto();
  primeraIteracion = 1;
  recepcionLista = 1;
  
}

//Funcion de callback para leer os mensajes enviados por el topic
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  if ((char)payload[0] == '2'){
    //Aviso de reccepcion lista
    recepcionLista = 1;
  }
  Serial.println();
}

// Función de reconectado al servidor MQTT
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("prueba", "hello world");
      // ... and resubscribe
      client.subscribe("prueba");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
 



void loop() {
  //Comprobamos la conexion con el servidor MQTT
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  //Si recibimos la confirmacion de que la imagen está en la Raspberry Pi
  if (FotoRealizada == 1) {
    snprintf (msg, 75, "%ld", FotoRealizada);
    Serial.print("Publish message: ");
    Serial.println(msg);
    client.publish("prueba", msg);
    FotoRealizada = 0;
  }
  
  //En la primera iteracion simpre se da el caso de que hay picos con los pines
  delay(1000);
  if(primeraIteracion == 1) {
    button1.checkPressed();
    primeraIteracion = 0;
  }else{
    //Comprobamos si el boton a sido pulsado
    button1.checkPressed();
    if (capturaFoto == 1 && recepcionLista == 1){
      Serial.printf("se ha pulsado el boton");
      
      unsigned long currentMillis = millis();
      //Comprobamos si el tiempo entre capturas consecutivas supera el minimo permitido
      if (currentMillis - previousMillis >= timerInterval) {
        sendPhoto();
        //Ponemos la recepcion de fotos a 0 para hasta que no nos llegue un mensaje desde la raspberry
        recepcionLista = 0;
        previousMillis = currentMillis;
        
      }
      //Reseteamos el boton que indica que se quiere hacer una captura
      capturaFoto = 0;  
    }
  }
}

//Funcion necearia para enviar la foto al servidor HTTP.
String sendPhoto() {
  String getAll;
  String getBody;
  Se toma una imagen y se almacena en el Buffer
  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();
  if(!fb) {
    Serial.println("Camera capture failed");
    delay(1000);
    ESP.restart();
  }
  
  Serial.println("Connecting to server: " + serverName);
  //Se comprueba si hay conexión con el servidor HTTP
  if (espClient.connect(serverName.c_str(), serverPort)) {
    Serial.println("Connection successful!");    
    String head = "--AntonioPerez\r\nContent-Disposition: form-data; name=\"imageFile\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--AntonioPerez--\r\n";

    uint16_t imageLen = fb->len;
    uint16_t extraLen = head.length() + tail.length();
    uint16_t totalLen = imageLen + extraLen;
  
    espClient.println("POST " + serverPath + " HTTP/1.1");
    espClient.println("Host: " + serverName);
    espClient.println("Content-Length: " + String(totalLen));
    espClient.println("Content-Type: multipart/form-data; boundary=AntonioPerez");
    espClient.println();
    espClient.print(head);

    //Se envia en fragmentos la imagen tomada
    uint8_t *fbBuf = fb->buf;
    size_t fbLen = fb->len;
    for (size_t n=0; n<fbLen; n=n+1024) {
      if (n+1024 < fbLen) {
        espClient.write(fbBuf, 1024);
        fbBuf += 1024;
      }
      else if (fbLen%1024>0) {
        size_t remainder = fbLen%1024;
        espClient.write(fbBuf, remainder);
      }
    }   
    espClient.print(tail);
    
    esp_camera_fb_return(fb);
    
    int timoutTimer = 10000;
    long startTimer = millis();
    boolean state = false;
    //Comenzamos con la lectura del mensaje recibido del servidor HTTP
    while ((startTimer + timoutTimer) > millis()) {
      Serial.print(".");
      delay(100);      
      while (espClient.available()) {
        char c = espClient.read();
        if (c == '\n') {
          if (getAll.length()==0) { state=true; }
          getAll = "";
        }
        else if (c != '\r') { getAll += String(c); }
        if (state==true) { getBody += String(c); }
        startTimer = millis();
      }
      if (getBody.length()>0) { break; }
    }
    //Serial.println();
    espClient.stop();
    getBody.toCharArray(vectorChar,3);
    int bodyInt = atoi(vectorChar);
    Serial.print(getBody);
    //Si la respuesta es 2 todo ha ido bien
    if (bodyInt == 2){
      FotoRealizada = 1;
      Serial.print("Se ha realizado la foto");
    }else{
      Serial.print("Noo se ha realizado la foto");
      FotoRealizada = 0;
      recepcionLista = 1;
    }
    //Serial.println(getBody); Dejar comentado para futuras depuraciones
  }
  else {
    getBody = "Connection to " + serverName +  " failed.";
    Serial.println(getBody);
    recepcionLista = 1;
  }
  return getBody; 
}
