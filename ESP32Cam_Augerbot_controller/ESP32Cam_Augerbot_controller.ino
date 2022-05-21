//created by ATLIN ANDERSON [ABA Industries]
//BOARD: AI THINKER ESP32-CAM
//ESP32 CAM board used

#include <WebServer.h>
#include <WiFi.h>
#include <esp32cam.h>
#include <WiFiUdp.h>


/////////////////////////////////////////////////////////////
//AUGERBOT SECTION/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////

//global variableS
const int dc_motor_max_rpm = 300;

int dc_motor_speed_rpm = 0;
int dc_motor_speed_rpm_old = 0;

int motor_direction_array[] = {1, -1, 1, -1};

//stores all commands to be sent to Motor controller
String serial_command_buffer = "";
String serial_command_buffer_old = "";


/////////////////////////////////////////////////////////////
//Network setup SECTION/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////

const char* WIFI_SSID = "ESP32 Augerbot V2";
const char* WIFI_PASS = "0123456789";

WiFiUDP Udp;
unsigned int localUdpPort = 4210;  // local port to listen on
char incomingPacket[1];  // buffer for incoming packets
char old_Packet[1];  // buffer for old packets


WebServer server(80);

TaskHandle_t Task1;


///////////////////////////////////////////////////////////////
////ESP32cam SECTION/////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
static auto loRes = esp32cam::Resolution::find(320, 240);
static auto hiRes = esp32cam::Resolution::find(800, 600);



void
setup()
{
  //enable the GPIO port for LED headlight
  pinMode(4, OUTPUT);
  Serial.begin(9600);

  //setup for ESP32cam stream
  {
    using namespace esp32cam;
    Config cfg;
    cfg.setPins(pins::AiThinker);
    cfg.setResolution(hiRes);
    cfg.setBufferCount(2);
    cfg.setJpeg(80);

    bool ok = Camera.begin(cfg);
    Serial.println(ok ? "CAMERA OK" : "CAMERA FAIL");
  }

  // Start the access point
  WiFi.softAP(WIFI_SSID, WIFI_PASS);
  Serial.print("Access Point \"");
  Serial.print(WIFI_SSID);
  Serial.println("\" started");

  Serial.print("IP address:\t");
  Serial.println(WiFi.softAPIP());

  server.on("/cam.bmp", handleBmp);
  server.on("/cam-lo.jpg", handleJpgLo);
  server.on("/cam-hi.jpg", handleJpgHi);
  server.on("/cam.jpg", handleJpg);
  server.on("/cam.mjpeg", handleMjpeg);

  server.begin();

  Udp.begin(localUdpPort);

  //Run the ESP32cam webserrver of core 1 (2nd core)
  xTaskCreatePinnedToCore(
    Task1code,   /* Task function. */
    "Task1",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    1,           /* priority of the task */
    &Task1,      /* Task handle to keep track of created task */
    0);          /* pin task to core 0 */
}


void
loop() {

  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    // receive incoming UDP packets;
    int len = Udp.read(incomingPacket, 255);
    if (len > 0)
    {
      incomingPacket[len] = 0;
    }

    //Functions to be preformed only when a packet is recievied
    ///////////////////////////////////////////////////////////////////
    if (incomingPacket[0] == 'F')
    {
      //toggle LED light
      digitalWrite(4, !digitalRead(4));
      incomingPacket[0] = old_Packet[0];
    }

    update_motor_speed ();
  }
  //Functions to be preformed repeatedly
  //////////////////////////////////////////////////////////////////////////////

  update_motor_direction();

  send_serial_command();
}
