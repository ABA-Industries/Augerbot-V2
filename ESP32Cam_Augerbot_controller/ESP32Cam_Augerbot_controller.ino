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
char  replyPacket[] = "";  // a reply string to send back

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

  //  Serial.print("http://");
  //  Serial.println(WiFi.localIP());
  //  Serial.println("  /cam.bmp");
  //  Serial.println("  /cam-lo.jpg");
  //  Serial.println("  /cam-hi.jpg");
  //  Serial.println("  /cam.mjpeg");

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

    /////////////////////////////////////////////
    //update servo speed
    if (incomingPacket[0] == '1')
    {
      dc_motor_speed_rpm = 1;
      dc_motor_speed_rpm_old = dc_motor_speed_rpm;
      incomingPacket[0] = old_Packet[0];
    }

    if (incomingPacket[0] == '2')
    {
      dc_motor_speed_rpm = 2;
      dc_motor_speed_rpm_old = dc_motor_speed_rpm;
      incomingPacket[0] = old_Packet[0];
    }

    if (incomingPacket[0] == '3')
    {
      dc_motor_speed_rpm = 3;
      dc_motor_speed_rpm_old = dc_motor_speed_rpm;
      incomingPacket[0] = old_Packet[0];
    }

    if (incomingPacket[0] == '4')
    {
      dc_motor_speed_rpm = 4;
      dc_motor_speed_rpm_old = dc_motor_speed_rpm;
      incomingPacket[0] = old_Packet[0];
    }

    if (incomingPacket[0] == '5')
    {
      dc_motor_speed_rpm = 5;
      dc_motor_speed_rpm_old = dc_motor_speed_rpm;
      incomingPacket[0] = old_Packet[0];
    }

    if (incomingPacket[0] == '6')
    {
      dc_motor_speed_rpm = 6;
      dc_motor_speed_rpm_old = dc_motor_speed_rpm;
      incomingPacket[0] = old_Packet[0];
    }

    if (incomingPacket[0] == '7')
    {
      dc_motor_speed_rpm = 7;
      dc_motor_speed_rpm_old = dc_motor_speed_rpm;
      incomingPacket[0] = old_Packet[0];
    }

    if (incomingPacket[0] == '8')
    {
      dc_motor_speed_rpm = 8;
      dc_motor_speed_rpm_old = dc_motor_speed_rpm;
      incomingPacket[0] = old_Packet[0];
    }

    if (incomingPacket[0] == '9')
    {
      dc_motor_speed_rpm = 9;
      dc_motor_speed_rpm_old = dc_motor_speed_rpm;
      incomingPacket[0] = old_Packet[0];
    }


  }
  //Functions to be preformed repeatedly
  //////////////////////////////////////////////////////////////////////////////

  // relative to front
  // 1 = right
  // -1 = left

  //{M1,  M2,
  // M3 , M4}

  //remember the two back motors are inverted

  if (incomingPacket[0] == 'S')
  {
    //Stop
    dc_motor_speed_rpm = 0;
  }



  if (incomingPacket[0] == 'W')
  {
    //forward
    motor_direction_array[0] = 1;
    motor_direction_array[1] = -1;
    motor_direction_array[2] = 1;
    motor_direction_array[3] = -1;

    dc_motor_speed_rpm = dc_motor_speed_rpm_old;
  }

  if (incomingPacket[0] == 'X')
  {
    //reverse
    motor_direction_array[0] = -1;
    motor_direction_array[1] = 1;
    motor_direction_array[2] = 1;
    motor_direction_array[3] = -1;

    dc_motor_speed_rpm = dc_motor_speed_rpm_old;
  }

  if (incomingPacket[0] == 'A')
  {
    //strafe left
    motor_direction_array[0] = -1;
    motor_direction_array[1] = -1;
    motor_direction_array[2] = 1;
    motor_direction_array[3] = 1;

    dc_motor_speed_rpm = dc_motor_speed_rpm_old;
  }

  if (incomingPacket[0] == 'D')
  {
    //strafe right
    motor_direction_array[0] = 1;
    motor_direction_array[1] = 1;
    motor_direction_array[2] = -1;
    motor_direction_array[3] = -1;

    dc_motor_speed_rpm = dc_motor_speed_rpm_old;
  }

  if (incomingPacket[0] == 'Q')
  {
    //strafe left
    motor_direction_array[0] = -1;
    motor_direction_array[1] = -1;
    motor_direction_array[2] = -1;
    motor_direction_array[3] = -1;

    dc_motor_speed_rpm = dc_motor_speed_rpm_old;
  }

  if (incomingPacket[0] == 'E')
  {
    //strafe right
    motor_direction_array[0] = 1;
    motor_direction_array[1] = 1;
    motor_direction_array[2] = 1;
    motor_direction_array[3] = 1;

    dc_motor_speed_rpm = dc_motor_speed_rpm_old;
  }

  serial_command_buffer.concat("D");
  for (int i = 0; i < (sizeof(motor_direction_array) / sizeof(int)); i++)
  {
    int temp = motor_direction_array[i] * dc_motor_speed_rpm;
    serial_command_buffer.concat(temp);
  }

  //send the final serial command string appended with servo speed
  if (serial_command_buffer.length() > 0 && serial_command_buffer != serial_command_buffer_old )
  {
    Serial.println(serial_command_buffer);
  }

  //then reset the string for new command
  serial_command_buffer_old = serial_command_buffer;
  serial_command_buffer = "";

  old_Packet[0] = incomingPacket[0];
}
