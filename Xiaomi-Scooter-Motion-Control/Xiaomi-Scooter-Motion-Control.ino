/*
  Refference used:
  https://github.com/maksoltane/esp32_xiaomi_m365/
*/

//current version working on esp32 & esp8266

#ifdef ESP32
  #include <WiFi.h>
  #include <WiFiUdp.h>
  #include <Update.h> //needed for arduinoOTA on esp32
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
  #include <WiFiUdp.h>
  WiFiEventHandler stationConnectedHandler;
  WiFiEventHandler stationDisconnectedHandler;
#else
  #error Platform not supported
#endif


#include <ArduinoOTA.h>
#include <arduino-timer.h>

auto timer = timer_create_default();


#define swversion "1.0"

// PINS

#ifdef ESP32
  const int throttlepin = D4;
#elif defined(ESP8266)
  const int throttlepin = D4;
#else
  #error Platform not supported
#endif


//functional modules

  //#define usewlanclientmode //NOT IMPLEMENTED comment out to disable Wifi Client functionality
  //#define usewlanapmode //NOT IMPLEMENTED  comment out to disable Wifi Access Poiint functionality
  #define usetelnetserver //comment out to disable telnet status/telemetrie server, this also disables RAW Server (so only mqtt might be left for leaving wifi activated)
  //#define userawserver //comment out to disable RAW Serial Data BUS Stream on Port 36524, NOT VERIFIED against wired-data-stream
  #define usepacketserver //comment out to disable PACKET Decode on Port 36525
  //#define usemqtt //NOT IMPLEMENTED comment out to disable mqtt functionality

//DEBUG Settings
  //#define debug_dump_states //dump state machines state
  //#define debug_dump_rawpackets //dump raw packets to Serial/Telnet
  //#define debug_dump_packetdecode //dump infos from packet decoder

//Config Settings (should be adopted to personal desires)

  #ifdef usemqtt
    #include <PubSubClient.h>
    #define mqtt_clientID "xiaomi-motion-controller"
    #define mqtt_server "192.168.0.31"
    #define mqtt_port 1883
  #endif

  #define OTApwd "h5fj8ovbthrfd65b4"



//WLAN
  #define maxssids 1
  #define ssid1 "mi-scooter-DEV"
  #define password1 "h5fj8bvothrfd65b4"
  #define ssid2 "..."
  #define password2 "..."
  #define ssid3 "..."
  #define password3 "..."

  const char *apssid ="m365oled";
  const char *appassword="365";
  //#define staticip //use static IP in Client Mode, Comment out for DHCP
  #ifdef staticip //static IP for Client Mode, in AP Mode default is 192.168.4.1/24
    IPAddress ip(192,168,0,149);
    IPAddress gateway(192,168,0,1);
    IPAddress dns(192,168,0,1);
    IPAddress subnet(255,255,255,0);
  #endif
  
  uint8_t currentssidindex = 0;
  uint8_t apnumclientsconnected=0;
  uint8_t apnumclientsconnectedlast=0;

  uint8_t wlanstate = 0;
  uint8_t wlanstateold = 0;
  #define wlanoff 0
  #define wlanturnapon 1
  #define wlanturnstaon 2
  #define wlansearching 3
  #define wlanconnected 4
  #define wlanap 5
  #define wlanturnoff 6 

  #define wlanconnecttimeout 10000 //timeout for connecting to one of  the known ssids
  #define wlanapconnecttimeout 30000 //timeout in ap mode until client needs to connect / ap mode turns off
  unsigned long wlanconnecttimestamp = 0;

#ifdef usetelnetserver
//TELNET
  WiFiServer telnetserver(36523);
  WiFiClient telnetclient;
  #ifdef userawserver
    WiFiServer rawserver(36524);
    WiFiClient rawclient;
  #endif
  #ifdef usepacketserver
    WiFiServer packetserver(36525);
    WiFiClient packetclient;
  #endif
  uint8_t telnetstate = 0;
  uint8_t telnetstateold = 0;
  uint8_t telnetrawstate = 0;
  uint8_t telnetrawstateold = 0;
  
  #define telnetoff 0
  #define telnetturnon 1
  #define telnetlistening 2
  #define telnetclientconnected 3
  #define telnetturnoff 4
  #define telnetdisconnectclients 5

  #define userconnecttimeout 30000 //timeout for connecting to telnet/http after wlan connection has been established with ap or client
  #define telnetrefreshanyscreen 100 //refresh telnet screen every xx ms
  #define telnetrefreshrawarrayscreen 500 //refresh telnet screen every xx ms
  unsigned long userconnecttimestamp = 0;
  unsigned long telnetnextrefreshtimestamp = 0;
  
  //telnet screens
  #define ts_telemetrie 0
  #define ts_statistics 1
  #define ts_esc_raw 2
  #define ts_ble_raw 3
  #define ts_bms_raw 4
  #define ts_x1_raw 5
  #define ts_esc_array 6
  #define ts_ble_array 7
  #define ts_bms_array 8
  #define ts_x1_array 9
  uint8_t telnetscreen = ts_statistics;
  uint8_t telnetscreenold = telnetscreen;

  //ANSI
  String ansiPRE  = "\033"; // escape code
  String ansiHOME = "\033[H"; // cursor home
  String ansiESC  = "\033[2J"; // esc
  String ansiCLC  = "\033[?25l"; // invisible cursor
  String ansiEND  = "\033[0m";   // closing tag for styles
  String ansiBOLD = "\033[1m";
  String ansiRED  = "\033[41m"; // red background
  String ansiGRN  = "\033[42m"; // green background
  String ansiBLU  = "\033[44m"; // blue background
  String ansiREDF = "\033[31m"; // red foreground
  String ansiGRNF = "\033[34m"; // green foreground
  String ansiBLUF = "\033[32m"; // blue foreground
  String BELL     = "\a";
#endif


//M365 - serial
  /* ATTENZIONE - the Arduino Serial Library uses HardwareSerial from the arduino-esp32 core which uses "esp32-hal-uart.cpp" driver from esp-idf which runs on RTOS
  * ESP32 has 128Byte Hardware RX Buffer
  * esp32-hal-uart uses interrupts, but default config toggles the rx-event only after 112 received bytes
  * this leads to the problem -> on m365 we must listen to the packets sent by other nodes (bms/esc/ble modules) and send our stuff with the right timing
  * so long explanation, short solution:
  * in esp32-hal-uart.c, in void uartEnableInterrupt(uart_t* uart) change
  * uart->dev->conf1.rxfifo_full_thrhd = 112;
  * to
  *  uart->dev->conf1.rxfifo_full_thrhd = 1;
  */
  //#define UART2RX GPIO_NUM_5 //TTGO Test board
  //#define UART2TX GPIO_NUM_17 //TTGO Test board
  //#define UART2RXunused GPIO_NUM_23 //TTGO Test board; ESP32 does not support RX or TX only modes - so we remap the rx pin to a unused gpio during sending

  //custom pins see here: http://www.iotsharing.com/2017/05/how-to-use-serial-arduino-esp32-print-debug.html?m=1
  //enable rx/tx only  modes see: https://github.com/esp8266/Arduino/blob/master/cores/esp8266/HardwareSerial.h
  #ifdef ESP32
    #define DebugSerial Serial //Debuguart = default Serial Port/UART0
    HardwareSerial M365Serial(2);  // UART2for M365
    #define UART2RX GPIO_NUM_23 //Wemos board
    #define UART2TX GPIO_NUM_5 //Wemos board
    #define UART2RXunused GPIO_NUM_19 //TTGO Test board; ESP32 does not support RX or TX only modes - so we remap the rx pin to a unused gpio during sending
    #define M365SerialFull M365Serial.begin(115200,SERIAL_8N1, UART2RX, UART2TX);
    //#define Serial1RX M365Serial.begin(115200,SERIAL_8N1, UART2RX, -1)
    #define M365SerialTX M365Serial.begin(115200,SERIAL_8N1, UART2RXunused, UART2TX);
  #elif defined(ESP8266)
    #define DebugSerial Serial1 //UART1 is TX only, will be mapped to GPIO2 (-> Debug output will be there)
    #define M365Serial Serial  // UART0 will be remapped to GPIO 13(RX) / 15 (TX) and used for M365 Communication
    #define UART2RX 2 //Wemos board
    #define UART2TX 4 //Wemos board

    #define M365SerialFull M365Serial.begin(115200,SERIAL_8N1, (SerialMode)UART_FULL); M365Serial.swap();
    //#define Serial1RX M365Serial.begin(115200,SERIAL_8N1, UART_RX_ONLY, UART2RX, UART2TX); M365Serial.swap();
    #define M365SerialTX M365Serial.begin(115200,SERIAL_8N1, (SerialMode)UART_TX_ONLY); M365Serial.swap();


    //TEST M365Serial.setRxBufferSize(1);
  #endif




//M365 - serial receiver
  #define maxlen 256 //serial buffer size in bytes
  uint8_t crc1=0;
  uint8_t crc2=0;
  uint16_t crccalc=0;
  uint8_t sbuf[maxlen];
  uint8_t len=0;
  uint8_t readindex=0;
  uint8_t m365receiverstate = 0;
  uint8_t m365receiverstateold = 0;
  #define m365receiveroff 0 //Serial1 will not be read...
  #define m365receiverready 1 //reading bytes until we find 0x55
  #define m365receiverpacket1 2 //preamble 0x55 received, waiting for 0xAA
  #define m365receiverpacket2 3 //len preamble received waiting for LEN
  #define m365receiverpacket3 4 //payload this state is kept until LEN bytes received
  #define m365receiverpacket4 5 //checksum receiving checksum
  #define m365receiverpacket5 6 //received a packet, test for checksum
  #define m365receiverstorepacket 7 //packet received, checksum valid, store in array, jump back to receiverready for next packet, set newpacket flag

//M365 - packets
  uint8_t m365packetstate = 0;
  uint8_t m365packetstateold = 0;
  #define m365packetidle 0
  #define m365newpacket 1
  //offsets in sbuf
  #define i_address 0
  #define i_hz 1
  #define i_offset 2
  #define i_payloadstart 3
  #define address_ble 0x20 //actively sent data by BLE Module with gas/brake & mode values
  #define address_x1 0x21 //actively sent data by ?BLE? with status like led on/off, normal/ecomode...
  #define address_esc 0x23 
  #define address_bms_request 0x22 //used by apps to read data from bms
  #define address_bms 0x25 //data from bms sent with this address (only passive if requested via address_esc_request)

/*
  #define aap2bms 0x22

  #define MastertoM365 0x20
#define M365toMaster 0x23

#define MastertoBATT 0x22 //request data from bms
#define BATTtoMaster 0x25
*/

//M365 - Statistics
  uint16_t packets_rec=0;
  uint16_t packets_crcok=0;
  uint16_t packets_crcfail=0;
  uint16_t packets_rec_bms=0;
  uint16_t packets_rec_esc=0;
  uint16_t packets_rec_x1=0;
  uint16_t packets_rec_ble=0;
  uint16_t packets_rec_unhandled=0;

  uint16_t packetsperaddress[256];
  uint16_t requests_sent_bms=0;
  uint16_t requests_sent_esc=0;

  int16_t speed_min = 0;
  int16_t speed_max = 0;
  int16_t oldspeed = 0;
  int16_t current_min = 0;
  int16_t current_max = 0;
  int32_t watt_min = 0;
  int32_t watt_max = 0;

  #define m365packettimeout  500 //timeout in ms after last received packet for showing connection-error
  unsigned long m365packettimestamp = 0;
  unsigned long m365packetlasttimestamp = 0;

  unsigned long duration_requestcycle=0;
  unsigned long timestamp_requeststart=0;


//M365 Device Buffers & Structs
  typedef struct {
    uint8_t u1[1];
    uint8_t throttle;
    uint8_t brake;
    //uint8_t u2[509];
  }__attribute__((packed))   blestruct;

  typedef struct {
    uint8_t mode; //offset 0x00 mode: 0-stall, 1-drive, 2-eco stall, 3-eco drive
    uint8_t battleds;  //offset 0x01 battery status 0 - min, 7(or 8...) - max
    uint8_t light;  //offset 0x02 0= off, 0x64 = on
    uint8_t beepaction;  //offset 0x03 "beepaction" ?
    //uint8_t u1[508];
  }__attribute__((packed))   x1struct;

  typedef struct {
    uint16_t u1[0x10]; //offset 0-0x1F
    char serial[14]; //offset 0x20-0x2D
    char pin[6]; //offset 0x2E-0x33
    uint8_t fwversion[2]; //offset 0x34-035,  e.g. 0x133 = 1.3.3
    uint16_t u2[10]; //offset 0x36-0x49
    uint16_t remainingdistance; //offset 0x4a-0x4B e.g. 123 = 1.23km
    uint16_t u3[20]; //offset 0x4C-0x73
    uint16_t ontime1; //offset 0x74-0x75 power on time in seconds
    uint16_t triptime; //offset 0x76-0x77 trip time in seconds
    uint16_t u4[2]; //offset 0x78-0x7C
    uint16_t frametemp1; //offset 0x7C-0x7D /10 = temp in °C
    uint16_t u5[54]; //offset 0x7e-0xe9
    uint16_t ecomode; //offset 0xEA-0xEB; on=1, off=0
    uint16_t u6[5]; //offset 0xec-0xf5
    uint16_t kers; //offset 0xf6-0xf7; 0 = weak, 1= medium, 2=strong
    uint16_t cruisemode; //offset 0xf8-0xf9, off 0, on 1
    uint16_t taillight; //offset 0xfa-0xfb, off 0, on 2
    uint16_t u7[50]; //offset  0xfc-0x15f
    uint16_t error; //offset 0x160-0x161
    uint16_t u8[3]; //offset 0x162-0x167
    uint16_t battpercent; //offset 0x168-0x169
    int16_t speed; //0x16A-0x16B /1000 in km/h, negative value = backwards...
    uint16_t averagespeed; //0x16C-0x16D /1000 in km/h?
    uint32_t totaldistance; //0x16e-0x171 /1000 in km
    uint16_t tripdistance; //0x172-0x173
    uint16_t ontime2; //offset 0x174-0x175 power on time 2 in seconds
    uint16_t frametemp2; //0x176-0x177 /10 = temp in °C
    uint16_t u10[68]; //offset 0x178-0x200 
  }__attribute__((packed))   escstruct;

  typedef struct {
    uint16_t u1[0x10]; //offset 0-0x1F
    char serial[14]; //offset 0x20-0x2D
    uint8_t fwversion[2]; //offset 0x2E-0x2f e.g. 0x133 = 1.3.3
    uint16_t totalcapacity; //offset 0x30-0x31 mAh
    uint16_t u2a[2]; //offset 0x32-0x35
    uint16_t cycles; //offset 0x36-0x37
    uint16_t chargingtimes; //offset 0x38-0x39
    uint16_t u2b[3]; //offset 0x3a-0x3f
    uint16_t proddate; //offset 0x40-0x41
        //fecha a la batt 7 MSB->año, siguientes 4bits->mes, 5 LSB ->dia ex:
      //b 0001_010=10, año 2010
      //        b 1_000= 8 agosto
      //            b  0_1111=15 
      //  0001_0101_0000_1111=0x150F 
    uint16_t u3[0x10]; //offset 0x42-0x61
    uint16_t remainingcapacity; //offset 0x62-0x63
    uint16_t remainingpercent; //offset 0x64-0x65
    int16_t current; //offset 0x66-67 - negative = charging; /100 = Ampere
    uint16_t voltage; //offset 0x68-69 /10 = Volt
    uint8_t temperature[2]; //offset 0x6A-0x6B -20 = °C
    uint16_t u4[5]; //offset 0x6C-0x75
    uint16_t health; //offset 0x76-0x77; 0-100, 60 schwellwert "kaputt"
    uint16_t u5[4]; //offset 0x78-0x7F
    uint16_t Cell1Voltage; //offset 0x80-0x81
    uint16_t Cell2Voltage; //offset 0x82-0x83
    uint16_t Cell3Voltage; //offset 0x84-0x85
    uint16_t Cell4Voltage; //offset 0x86-0x87
    uint16_t Cell5Voltage; //offset 0x88-0x89
    uint16_t Cell6Voltage; //offset 0x8A-0x8B
    uint16_t Cell7Voltage; //offset 0x8C-0x8D
    uint16_t Cell8Voltage; //offset 0x8E-0x8F
    uint16_t Cell9Voltage; //offset 0x90-0x91
    uint16_t Cell10Voltage; //offset 0x92-0x93
    uint16_t Cell11Voltage; //offset 0x94-0x95 not stock, custom bms with 12S battery, 0 if not connected
    uint16_t Cell12Voltage; //offset 0x96-0x97 not stock, custom bms with 12S battery, 0 if not connected
    //uint16_t u6[178]; //offset 0x98-0x
  }__attribute__((packed))  bmsstruct;

  uint8_t bledata[512];
  uint8_t x1data[512];
  uint8_t escdata[512];
  uint8_t bmsdata[512];

  blestruct* bleparsed = (blestruct*)bledata;
  x1struct* x1parsed = (x1struct*)x1data;
  escstruct* escparsed = (escstruct*)escdata;
  bmsstruct* bmsparsed = (bmsstruct*)bmsdata;

  bool newdata = false; //flag - we have update at least one byte in one of the data arrays
  bool senddata = false; //flag - we should send our data request _now_

//M365 - request stuff 
  /*
  request data from esc:   (Read 0x7x 2 words)
  PREAMBLE  LEN  Adr  HZ   Off  LEN  FIX1 FIX2 FIX30x
  0x55 0xaa 0x06 0x20 0x61 0x7c 0x02 0x02 0x28 0x27 CRC1 CRC2
  request data from bms: (Read Serial @ Offset 0x10, 0x10 words)
  PREAMBLE    LEN   Adr   HZ    Off   Len   CRC1  CRC2
  0x55  0xAA  0x03  0x22  0x01  0x10  0x12  0xB7  0xFF
  */

  //packet for bms requests and offsets
    uint8_t request_bms[9] = { 0x55,0xAA,0x03,0x22,0x01,0x10,0x3A,0xB7,0xFF};
    //uint8_t request_bms_serial[9] = { 0x55,0xAA,0x03,0x22,0x01,0x10,0x12,0xB7,0xFF};
    #define bms_request_offset 5
    #define bms_request_len 6
    #define bms_request_crcstart 2
    #define bms_request_crc1 7
    #define bms_request_crc2 8
  //packets for esc requests and offsets
    uint8_t request_esc[12] = { 0x55,0xAA,0x06,0x20,0x61,0x10,0xAC,0x02,0x28,0x27,0xB7,0xFF};
    //uint8_t request_esc_speed[12] = { 0x55,0xAA,0x06,0x20,0x61,0x7c,0x02,0x02,0x28,0x27,0xB7,0xFF};
    #define esc_request_offset 5 
    #define esc_request_len 6
    #define esc_request_throttle 8
    #define esc_request_brake 9
    #define esc_request_crcstart 2
    #define esc_request_crc1 10
    #define esc_request_crc2 11

  //request arrays  
    uint8_t requestindex = 0;
    #define requestmax 8
    uint8_t requests[requestmax][3]= {
        { address_esc, 0xB0, 0x20}, //error, battpercent,speed,averagespeed,totaldistance,tripdistance,ontime2,frametemp2
        { address_esc, 0x25, 0x02}, //remaining distance
        { address_esc, 0x3A, 0x0A}, //ontime, triptime, frametemp1
        { address_bms, 0x31, 0x0A}, //remaining cap & percent, current, voltage, temperature
        { address_bms, 0x40, 0x18}, //cell voltages (Cells 1-10 & 11-12 for 12S Batt/Custom BMS)
        { address_bms, 0x3B, 0x02}, //Battery Health
        { address_bms, 0x10, 0x22}, //serial,fwversion,totalcapacity,cycles,chargingtimes,productiondate
        { address_esc, 0x10, 0x16} //serial,pin,fwversion
    };

  //friendly names
    #define rq_esc_essentials 0x01
    #define rq_esc_remdist 0x02
    #define rq_esc_essentials2 0x04
    #define rq_bms_essentials 0x08
    #define rq_bms_cells 0x10
    #define rq_bms_health 0x20
    #define rq_bms_versioninfos 0x40
    #define rq_esc_versioninfos 0x80

  //mapping oled screens / requeired data
    #define numscreens 5
    uint8_t rqsarray[numscreens] = {
      rq_esc_essentials|rq_esc_remdist|rq_esc_essentials2|rq_bms_essentials|rq_bms_cells|rq_bms_health|rq_bms_versioninfos|rq_esc_versioninfos, //rq_esc_essentials|rq_bms_essentials|rq_esc_essentials2, //oledstop
      //0xff, //oledstop, TODO: Request infos like Serial/FW Version only _once_ (when entering subscreen)
      rq_esc_essentials|rq_bms_essentials, //oleddrive
      rq_esc_essentials, //oledm365error
      rq_esc_essentials, //oledtimeout
      rq_esc_essentials|rq_bms_essentials|rq_bms_cells //charging
    };

  uint8_t subscribedrequests = rq_esc_essentials|rq_bms_essentials;

//motionmodes
  uint8_t motionstate = 0;
  #define motionready 0
  #define motionbusy 1
  #define motionbreaking 2


  uint8_t stopsubscreen = 0;
  uint8_t chargesubscreen = 0;
  uint8_t windowsubpos=0;
  #define stopsubscreens 8
  #define chargesubscreens 2
  #define gasmin 50
  #define gasmax 190
  #define stopwindowsize (uint8_t)((gasmax-gasmin)/stopsubscreens)
  #define chargewindowsize (uint8_t)((gasmax-gasmin)/chargesubscreens)

//Misc
  #ifdef ESP32
    #define led GPIO_NUM_2
  #endif
  #ifdef ESP8266
    #define led 2
  #endif
  int ledontime = 200;
  int ledofftime = 10000;
  unsigned long ledcurrenttime = 100;
  uint8_t i;
  char tmp1[200];
  char tmp2[200];
  uint16_t tmpi;
  char chipid[7];
  char mac[12];
  unsigned int lastprogress=0;

  unsigned long duration_mainloop=0;
  unsigned long timestamp_mainloopstart=0;
  unsigned long duration_oled=0;
  unsigned long timestamp_oledstart=0;
  unsigned long duration_oled1draw=0;
  unsigned long timestamp_oled1draw=0;
  unsigned long duration_oled1i2c=0;
  unsigned long timestamp_oled1i2c=0;
  unsigned long duration_oled2draw=0;
  unsigned long timestamp_oled2draw=0;
  unsigned long duration_oled2i2c=0;
  unsigned long timestamp_oled2i2c=0;
  unsigned long duration_telnet=0;
  unsigned long timestamp_telnetstart=0;


  #define _max(a,b) ((a)>(b)?(a):(b))
  #define _min(a,b) ((a)<(b)?(a):(b))

void reset_statistics() {
  packets_rec=0;
  packets_crcok=0;
  packets_crcfail=0;
  for(uint16_t j = 0; j<=255; j++) {
    packetsperaddress[j]=0;  
  }
  for(uint16_t j = 0; j<=511; j++) {
    escdata[j]=0;  
    bmsdata[j]=0;  
    bledata[j]=0;  
    x1data[j]=0;  
  }
  speed_min = 0;
  speed_max = 0;
  current_min = 0;
  current_max = 0;
  watt_min = 0;
  watt_max = 0;
}

void start_m365() {
  M365SerialFull
  m365receiverstate = m365receiverready;
  m365packetstate=m365packetidle;
  reset_statistics();
}  //startm365




void m365_updatestats() {
  if (speed_min>escparsed->speed) { speed_min = escparsed->speed; }
  if (speed_max<escparsed->speed) { speed_max = escparsed->speed; }
  if (current_min>bmsparsed->current) { current_min = bmsparsed->current; }
  if (current_max<bmsparsed->current) { current_max = bmsparsed->current; }
  int32_t currentwatt = (int32_t)((float)(bmsparsed->voltage/100.0f)*(float)bmsparsed->current/100.0f);
  if (watt_min>currentwatt) { watt_min = currentwatt; }
  if (watt_max<currentwatt) { watt_max = currentwatt; }
}

void m365_handlerequests() {
  //find next subscribed index
    uint8_t startindex = requestindex;
    while (!(subscribedrequests&(1<<requestindex)))  {
      //DebugSerial.printf("skipping RQ index %d\r\n",requestindex+1);
      requestindex++;
      if (requestindex==requestmax) {
          //DebugSerial.println("rollover in rqloop1");
          requestindex=0; 
          duration_requestcycle=millis()-timestamp_requeststart;
          timestamp_requeststart=millis(); 
      }
      if (requestindex==startindex) { break; }
    }

  //request data for current index if we are interested in
  if (subscribedrequests&(1<<requestindex)) {
    //DebugSerial.printf("requesting RQ index %d\r\n",requestindex+1);
    if (requests[requestindex][0]==address_bms) {
      request_bms[bms_request_offset] = requests[requestindex][1];
      request_bms[bms_request_len] = requests[requestindex][2];
      crccalc = 0x00;
      for(uint8_t i=bms_request_crcstart;i<bms_request_crc1;i++) {
        crccalc=crccalc + request_bms[i];
      }
      crccalc = crccalc ^ 0xffff;
      request_bms[bms_request_crc1]=(uint8_t)(crccalc&0xff);
      request_bms[bms_request_crc2]=(uint8_t)((crccalc&0xff00)>>8);
      M365Serial.write((unsigned char*)&request_bms,9);
      requests_sent_bms++;
    } //if address address_bms
    if (requests[requestindex][0]==address_esc) {
      request_esc[esc_request_offset] = requests[requestindex][1];
      request_esc[esc_request_len] = requests[requestindex][2];
      request_esc[esc_request_throttle] = bleparsed->throttle;
      request_esc[esc_request_brake] = bleparsed->brake;
      crccalc = 0x00;
      for(uint8_t i=esc_request_crcstart;i<esc_request_crc1;i++) {
        crccalc=crccalc + request_esc[i];
      }
      crccalc = crccalc ^ 0xffff;
      request_esc[esc_request_crc1]=(uint8_t)(crccalc&0xff);
      request_esc[esc_request_crc2]=(uint8_t)((crccalc&0xff00)>>8);
      M365Serial.write((unsigned char*)&request_esc,12);
      requests_sent_esc++;
    } //if address address_esc
  } else {
    //DebugSerial.printf("skipping RQ index %d\r\n",requestindex+1);
  }
  //prepare next requestindex for next call
  requestindex++;
  if (requestindex==requestmax) { 
    //DebugSerial.println("rollover in rqloop2");
    requestindex=0; 
    duration_requestcycle=millis()-timestamp_requeststart;
    timestamp_requeststart=millis();
  }
  //DebugSerial.printf("---REQUEST-- %d\r\n",millis());
} //m365_handlerequests

void m365_handlepacket() {
 if (m365packetstate==m365newpacket) {
    m365packetlasttimestamp = millis()-m365packettimestamp;
    m365packettimestamp=millis();
    packetsperaddress[sbuf[i_address]]++;  
    

    #ifdef debug_dump_packetdecode
      sprintf(tmp1,"[%03d] PACKET Len %02X Addr %02X HZ %02X Offset %02X CRC %04X Payload: ",m365packetlasttimestamp,len,sbuf[i_address],sbuf[i_hz],sbuf[i_offset], crccalc);
      DebugSerial.print(tmp1);
      for(i = 0; i < len-3; i++){
        DebugSerial.printf("%02X ",sbuf[i_payloadstart+i]);
      }
      DebugSerial.println("");
      /*for(i = 0; i < MAX_SRV_CLIENTS; i++){
        if (serverClients[i] && serverClients[i].connected()){
          serverClients[i].write(tmp1,37);
          yield();
       }
      }*/
    #endif
    switch (sbuf[i_address]) {
      case address_bms:
          packets_rec_bms++;
          memcpy((void*)& bmsdata[sbuf[i_offset]<<1], (void*)& sbuf[i_payloadstart], len-3);
        break;
      case address_esc:
          packets_rec_esc++;
          memcpy((void*)& escdata[sbuf[i_offset]<<1], (void*)& sbuf[i_payloadstart], len-3);
        break;
      case address_ble:
          packets_rec_ble++;
          memcpy((void*)& bledata[sbuf[i_offset]<<1], (void*)& sbuf[i_payloadstart], len-3);
        break;
      case address_x1:
          packets_rec_x1++;
          memcpy((void*)& x1data[sbuf[i_offset]<<1], (void*)& sbuf[i_payloadstart], len-3);
        break;
      default:
          packets_rec_unhandled++;
        break;
    }
    newdata=true;
    m365packetstate=m365packetidle;
 } //if (m365packetstate==m365newpacket)
 else {
  if ((m365packettimestamp+m365packettimeout)>millis()) {
    //packet timeout
  }
 }
} //m365_handlepacket
  
void m365_receiver() { //recieves data until packet is complete
  uint8_t newbyte;
  if (M365Serial.available()) {
    newbyte = M365Serial.read();
#ifdef userawserver
     if (rawclient && rawclient.connected()) { rawclient.write(newbyte); }
#endif    
    switch(m365receiverstate) {
      case m365receiverready: //we are waiting for 1st byte of packet header 0x55
          if (newbyte==0x55) {
            m365receiverstate = m365receiverpacket1;
          }
        break;
      case m365receiverpacket1: //we are waiting for 2nd byte of packet header 0xAA
          if (newbyte==0xAA) {
            m365receiverstate = m365receiverpacket2;
            }
        break;
      case m365receiverpacket2: //we are waiting for packet length
          len = newbyte+1; //next byte will be 1st in sbuf, this is the packet-type1, len counted from 2nd byte in sbuf
          crccalc=newbyte;
          readindex=0;
          m365receiverstate = m365receiverpacket3;
        break;
      case m365receiverpacket3: //we are receiving the payload
          sbuf[readindex]=newbyte;
          readindex++;
          crccalc=crccalc+newbyte;
          if (readindex==len) {
            m365receiverstate = m365receiverpacket4;
          }
        break;
      case m365receiverpacket4: //we are waiting for 1st CRC byte
          crc1 = newbyte;
          m365receiverstate = m365receiverpacket5;
        break;
      case m365receiverpacket5: //we are waiting for 2nd CRC byte
          crc2 = newbyte;
          crccalc = crccalc ^ 0xffff;
          #ifdef usepacketserver
            if (packetclient && packetclient.connected()) { 
              sprintf(tmp1,"55 AA %02X ", len-1);
              sprintf(tmp2,"%02X %02X [CRCCalc:%04X]\r\n", crc1,crc2,crccalc);
              switch (sbuf[i_address]) {
                case address_bms:
                    packetclient.print(ansiRED);
                    packetclient.print("BMS: ");
                  break;
                case address_esc:
                    packetclient.print(ansiGRN);
                    packetclient.print("ESC: ");
                  break;
                case address_ble:
                    packetclient.print(ansiBLU);
                    packetclient.print("BLE: ");
                  break;
                case address_x1:
                    packetclient.print(ansiBLUF);
                    packetclient.print("X1 : ");
                  break;
                default:
                    packetclient.print(ansiREDF);
                    packetclient.print("???: ");
                  break;
                } //switch                else {
              packetclient.print(ansiEND);
              packetclient.print(tmp1);
              for(i = 0; i < len; i++){
                packetclient.printf("%02X ",sbuf[i]);
              } //for i...
              packetclient.print(tmp2);
            }
          #endif
          #ifdef debug_dump_rawpackets
            sprintf(tmp1,"Packet received: 55 AA %02X ", len-1);
            sprintf(tmp2,"CRC %02X %02X %04X\r\n", crc1,crc2,crccalc);
            DebugSerial.print(tmp1);
            for(i = 0; i < len; i++){
             DebugSerial.printf("%02X ",sbuf[i]);
            }
            DebugSerial.print(tmp2);
            /*
            for(i = 0; i < MAX_SRV_CLIENTS; i++){
              if (serverClients[i] && serverClients[i].connected()){
                serverClients[i].write(tmp1,26);
                for(i = 0; i < (len); i++){
                  telnetclient.printf("%02X ",sbuf[i]);
                  //serverClients[i].write(tmp1,3);
                }
                serverClients[i].write(tmp2, 12);
                yield();
             }
            }
            */
          #endif
          packets_rec++;
          if (crccalc==((uint16_t)(crc2<<8)+(uint16_t)crc1)) {
            m365packetstate = m365newpacket;
            packets_crcok++;
          } else {
            packets_crcfail++;
          }
        m365receiverstate = m365receiverready; //reset and wait for next packet
        if (sbuf[i_address]==0x20 && sbuf[i_hz]==0x65 && sbuf[i_offset]==0x00 ) {
          //senddata = true;
          m365_handlerequests();
          DebugSerial.printf("---REQUEST-- %d\r\n",millis());
        }
        break;
    } //switch
    //DebugSerial.printf("#S# %d\r\n",M365Serial.available());
  }//serial available
  
} //m365_receiver
 

#ifdef usetelnetserver
void telnet_refreshscreen() {
  uint8_t k=0;
  //for(i = 0; i < MAX_SRV_CLIENTS; i++){
    if (telnetclient && telnetclient.connected()){
        if (telnetscreenold!=telnetscreen) { telnetclient.print(ansiESC); telnetscreenold=telnetscreen; }
        telnetclient.print(ansiHOME);
        telnetclient.print(ansiCLC); 
        switch (telnetscreen) {
          case ts_telemetrie:
              telnetclient.print("M365 Telemetrie Screen"); 
            break;
          case ts_esc_raw:
              telnetclient.print("M365 ESC RAW Screen"); 
            break;
          case ts_ble_raw:
              telnetclient.print("M365 BLE RAW Screen"); 
            break;
          case ts_bms_raw:
              telnetclient.print("M365 BMS RAW Screen"); 
            break;
          case ts_x1_raw:
              telnetclient.print("M365 X1 RAW Screen"); 
            break;
          case ts_esc_array:
              telnetclient.print("M365 ESC Array Screen"); 
            break;
          case ts_ble_array:
              telnetclient.print("M365 BLE ARRAY Screen"); 
            break;
          case ts_bms_array:
              telnetclient.print("M365 BMS ARRAY Screen"); 
            break;
          case ts_x1_array:
              telnetclient.print("M365 X1 ARRAY Screen"); 
            break;
          case ts_statistics:
              telnetclient.print("M365 Statistics Screen");
            break;
        }
        tmpi++;
        if (tmpi % 2) {
          telnetclient.println(" .\r\n");
        } else {
          telnetclient.println("  \r\n");
        }
        switch (telnetscreen) {
          case ts_telemetrie:
              //if (newdata) {
                telnetclient.printf("\r\nBLE\r\n Throttle: %03d Brake %03d\r\n",bleparsed->throttle,bleparsed->brake);
                sprintf(tmp1,"%c%c%c%c%c%c%c%c%c%c%c%c%c%c",bmsparsed->serial[0],bmsparsed->serial[1],bmsparsed->serial[2],bmsparsed->serial[3],bmsparsed->serial[4],bmsparsed->serial[5],bmsparsed->serial[6],bmsparsed->serial[7],bmsparsed->serial[8],bmsparsed->serial[9],bmsparsed->serial[10],bmsparsed->serial[11],bmsparsed->serial[12],bmsparsed->serial[13]);
                telnetclient.printf("\r\n\r\nBMS Serial: %s Version: %x.%x.%x\r\n", tmp1,bmsparsed->fwversion[1],(bmsparsed->fwversion[0]&0xf0)>>4,bmsparsed->fwversion[0]&0x0f);
                telnetclient.printf(" Capacity Total: %5d mAh Remaining %5d mAh Percent %03d%%\r\n",bmsparsed->totalcapacity, bmsparsed->remainingcapacity, bmsparsed->remainingpercent);
                telnetclient.printf(" Temperature1 %3d°C Temperature2 %3d°C Health %05d\r\n",bmsparsed->temperature[1]-20, bmsparsed->temperature[0]-20, bmsparsed->health);
                telnetclient.printf(" Production Date: %d-%d-%d ",((bmsparsed->proddate)&0xFE00)>>9,((bmsparsed->proddate)&0x1E0)>>5,(bmsparsed->proddate)&0x1f);
                telnetclient.printf(" Charging Cycles: %d Charging Times: %d\r\n",bmsparsed->cycles,bmsparsed->chargingtimes);
                telnetclient.printf(" Voltage: %2.2f V Current %05d mA\r\n",(float)bmsparsed->voltage/100.0f, bmsparsed->current);
                telnetclient.printf(" C1: %1.3f C2: %1.3f C3: %1.3f C4: %1.3f C5: %1.3f C6: %1.3f C7: %1.3f C8: %1.3f C9: %1.3f C10: %1.3f\r\n",(float)bmsparsed->Cell1Voltage/1000.0f,(float)bmsparsed->Cell2Voltage/1000.0f,(float)bmsparsed->Cell3Voltage/1000.0f,(float)bmsparsed->Cell4Voltage/1000.0f,(float)bmsparsed->Cell5Voltage/1000.0f,(float)bmsparsed->Cell6Voltage/1000.0f,(float)bmsparsed->Cell7Voltage/1000.0f,(float)bmsparsed->Cell8Voltage/1000.0f,(float)bmsparsed->Cell9Voltage/1000.0f,(float)bmsparsed->Cell10Voltage/1000.0f);
                sprintf(tmp1,"%c%c%c%c%c%c%c%c%c%c%c%c%c%c",escparsed->serial[0],escparsed->serial[1],escparsed->serial[2],escparsed->serial[3],escparsed->serial[4],escparsed->serial[5],escparsed->serial[6],escparsed->serial[7],escparsed->serial[8],escparsed->serial[9],escparsed->serial[10],escparsed->serial[11],escparsed->serial[12],escparsed->serial[13]);
                telnetclient.printf("\r\n\r\nESC Serial: %s Version: %x.%x.%x\r\n", tmp1,escparsed->fwversion[1],(escparsed->fwversion[0]&0xf0)>>4,escparsed->fwversion[0]&0x0f);
                sprintf(tmp1,"%c%c%c%c%c%c",escparsed->pin[0],escparsed->pin[1],escparsed->pin[2],escparsed->pin[3],escparsed->pin[4],escparsed->pin[5]);
                telnetclient.printf(" Pin: %s Error %05d\r\n",tmp1,escparsed->error);
                telnetclient.printf(" Distance Total: %.2f km Trip: %.2f km Remaining %.2f km\r\n", (float)escparsed->totaldistance/1000.0f,(float)escparsed->tripdistance/100.0f,(float)escparsed->remainingdistance/100.0f);
                telnetclient.printf(" Power On Time1: %d s, Power On Time1: %d s, Trip Time: %d s\r\n",escparsed->ontime1,escparsed->ontime2,escparsed->triptime);
                telnetclient.printf(" FrameTemp1: %05d FrameTemp2: %.1f °C\r\n", (float)escparsed->frametemp1/10.0f, (float)escparsed->frametemp2/10.0f);
                telnetclient.printf(" Speed: %.2f km/h Avg: %.2f km/h\r\n", (float)escparsed->speed/1000.0f, (float)escparsed->averagespeed/1000.0f);
                telnetclient.printf(" Batt Percent: %3d%%\r\n",escparsed->battpercent);
                telnetclient.printf(" Ecomode: %05d Kers: %05d Cruisemode: %05d Taillight: %05d\r\n", escparsed->ecomode, escparsed->kers, escparsed->cruisemode, escparsed->taillight);
                telnetclient.printf("\r\n\r\nX1 Mode %03d LEDs %03d Light ",x1parsed->mode, x1parsed->battleds);
                if (x1parsed->light==0) { telnetclient.print("OFF "); } else {
                  if (x1parsed->light==100) { telnetclient.print("ON "); } else {
                   telnetclient.print("    ");
                  }
                }
                telnetclient.printf("%03d BeepAction %03d\r\n", x1parsed->light, x1parsed->beepaction);

              //}
              telnetnextrefreshtimestamp=millis()+telnetrefreshanyscreen;
            break;
          case ts_statistics:
              telnetclient.printf("Requests Sent:\r\n  ESC: %05d   BMS: %05d   Total: %05d\r\n", requests_sent_esc, requests_sent_bms, requests_sent_bms+requests_sent_esc);
              telnetclient.printf("Packets Received:\r\n  ESC: %05d   BMS: %05d   BLE: %05d   X1: %05d   unhandled: %05d\r\n", packets_rec_esc,packets_rec_bms,packets_rec_ble,packets_rec_x1,packets_rec_unhandled);
              telnetclient.printf("  CRC OK: %05d   CRC FAIL: %05d\r\n   Total: %05d\r\n",packets_crcok,packets_crcfail,packets_rec);
              telnetclient.printf("\r\nTimers:\r\n  Main:   %04.3f ms\r\n  Telnet: %04.3f ms\r\n",(float)duration_mainloop/1000.0f, (float)duration_telnet/1000.0f);
              telnetclient.printf("  OLED Main: %04.3f ms\r\n",(float)duration_oled/1000.0f);
              telnetclient.printf("  OLED1 Draw: %04.3f ms\r\n  OLED1 I2C:  %04.3f ms\r\n",(float)duration_oled1draw/1000.0f,(float)duration_oled1i2c/1000.0f);
              telnetclient.printf("  OLED2 Draw: %04.3f ms\r\n  OLED2 I2C:  %04.3f ms\r\n",(float)duration_oled2draw/1000.0f,(float)duration_oled2i2c/1000.0f);
              telnetclient.printf("  Request Cycle Loop: %04d ms\r\n  Request last Index: %03d\r\n", duration_requestcycle,requestindex);
              telnetclient.printf("  Time since last Packet: %05d ms\r\n",m365packetlasttimestamp);
     
              telnetclient.printf("\r\nPackets per device Address:\r\n");
              //for(i = 0; i < MAX_SRV_CLIENTS; i++){
              k = 0;
              for(uint16_t j = 0; j <=255; j++) {
                if (packetsperaddress[j]>=1) {
                  k++;
                  telnetclient.printf("%02X -> %05d  ", j, packetsperaddress[j]);
                  if ((k % 5)==0) { telnetclient.println(""); }
                }
              }
              //ServerClients[i].print 
              telnetnextrefreshtimestamp=millis()+telnetrefreshanyscreen;           
            break;
          case ts_esc_raw:
              telnetclient.println("     00 01 02 03   04 05 06 07   08 09 0A 0B   0C 0D 0E 0F   10 11 12 13   14 15 16 17   18 19 1A 1B   1C 1D 1E 1F");
              for(uint16_t j = 0; j<=511; j++) {
                  if ((j % 32)==0) { 
                    telnetclient.printf("\r\n%03X: ",j); 
                  } else {
                    if ((j % 4)==0) { 
                      telnetclient.print("- "); 
                    }
                  } //mod32
                  telnetclient.printf("%02X ", escdata[j]);
              }
              telnetnextrefreshtimestamp=millis()+telnetrefreshrawarrayscreen;
            break;
          case ts_ble_raw:
              telnetclient.println("     00 01 02 03   04 05 06 07   08 09 0A 0B   0C 0D 0E 0F   10 11 12 13   14 15 16 17   18 19 1A 1B   1C 1D 1E 1F");
              for(uint16_t j = 0; j<=511; j++) {
                  if ((j % 32)==0) { 
                    telnetclient.printf("\r\n%03X: ",j); 
                  } else {
                    if ((j % 4)==0) { 
                      telnetclient.print("- "); 
                    }
                  } //mod32
                  telnetclient.printf("%02X ", bledata[j]);
              }
              telnetnextrefreshtimestamp=millis()+telnetrefreshrawarrayscreen;
            break;
          case ts_bms_raw:
              telnetclient.println("     00 01 02 03   04 05 06 07   08 09 0A 0B   0C 0D 0E 0F   10 11 12 13   14 15 16 17   18 19 1A 1B   1C 1D 1E 1F");
              for(uint16_t j = 0; j<=511; j++) {
                  if ((j % 32)==0) { 
                    telnetclient.printf("\r\n%03X: ",j); 
                  } else {
                    if ((j % 4)==0) { 
                      telnetclient.print("- "); 
                    }
                  } //mod32
                  telnetclient.printf("%02X ", bmsdata[j]);
              }
              telnetnextrefreshtimestamp=millis()+telnetrefreshrawarrayscreen;
            break;
          case ts_x1_raw:
              telnetclient.println("     00 01 02 03   04 05 06 07   08 09 0A 0B   0C 0D 0E 0F   10 11 12 13   14 15 16 17   18 19 1A 1B   1C 1D 1E 1F");
              for(uint16_t j = 0; j<=511; j++) {
                  if ((j % 32)==0) { 
                    telnetclient.printf("\r\n%03X: ",j); 
                  } else {
                    if ((j % 4)==0) { 
                      telnetclient.print("- "); 
                    }
                  } //mod32
                  telnetclient.printf("%02X ", x1data[j]);
              }
              telnetnextrefreshtimestamp=millis()+telnetrefreshrawarrayscreen;
            break;
          case ts_esc_array:
          case ts_ble_array:
          case ts_bms_array:
          case ts_x1_array:
              telnetclient.print("empty screen - not implemented");
              telnetnextrefreshtimestamp=millis()+telnetrefreshanyscreen;
              
            break;
        }
        yield();
    }
  //} //i 0 to maxclients
  //DebugSerial.printf("---TELNET--- %d\r\n",millis());
}

void handle_telnet() {
  timestamp_telnetstart=micros();
  boolean hc = false;
   switch(telnetstate) {
    case telnetoff:
      break;
    case telnetturnon:
        telnetserver.begin();
        telnetserver.setNoDelay(true);
        DebugSerial.print("### Telnet Debug Server @ ");
        if (wlanstate==wlanap) {
          DebugSerial.print(WiFi.softAPIP());  
        } else {
          DebugSerial.print(WiFi.localIP());  
        }
        DebugSerial.println(":36523");
#ifdef userawserver
        rawserver.begin();
        rawserver.setNoDelay(true);
        DebugSerial.print("### Telnet RAW Server @ ");
        if (wlanstate==wlanap) {
          DebugSerial.print(WiFi.softAPIP());  
        } else {
          DebugSerial.print(WiFi.localIP());  
        }
        DebugSerial.println(":36524");
#endif        
#ifdef usepacketserver
        packetserver.begin();
        packetserver.setNoDelay(true);
        DebugSerial.print("### Telnet PACKET Server @ ");
        if (wlanstate==wlanap) {
          DebugSerial.print(WiFi.softAPIP());  
        } else {
          DebugSerial.print(WiFi.localIP());  
        }
        DebugSerial.println(":36525");
#endif        

        

        telnetstate = telnetlistening;
        userconnecttimestamp = millis()+userconnecttimeout;  
      break;
    case telnetlistening: 
            if (telnetserver.hasClient()) {
              //for(i = 0; i < MAX_SRV_CLIENTS; i++){
                //find free/disconnected spot
                if (!telnetclient || !telnetclient.connected()){
                  if(telnetclient) telnetclient.stop();
                  telnetclient = telnetserver.available();
                  if (!telnetclient) DebugSerial.println("available broken");
                  DebugSerial.print("### Telnet New client: ");
                  DebugSerial.print(i); DebugSerial.print(' ');
                  DebugSerial.println(telnetclient.remoteIP());
                  telnetstate = telnetclientconnected;
                  ledontime=250; ledofftime=250; ledcurrenttime = millis();
                  telnetnextrefreshtimestamp=millis()+telnetrefreshanyscreen;
                  break;
                }
            //}
            /*if (i >= MAX_SRV_CLIENTS) {
              //no free/disconnected spot so reject
              telnetserver.available().stop();
              DebugSerial.println("### Telnet rejected connection (max Clients)");
            }*/
          }
#ifdef userawserver
          if (rawserver.hasClient()) {
            //for(i = 0; i < MAX_SRV_CLIENTS; i++){
              //find free/disconnected spot
              if (!rawclient || !rawclient.connected()){
                if(rawclient) rawclient.stop();
                rawclient = rawserver.available();
                if (!rawclient) DebugSerial.println("available broken");
                DebugSerial.print("### Telnet RAW New client: ");
                DebugSerial.println(rawclient.remoteIP());
                telnetstate = telnetclientconnected;
                ledontime=250; ledofftime=250; ledcurrenttime = millis();
                //telnetnextrefreshtimestamp=millis()+telnetrefreshanyscreen;
                //break;
              }
            //}
          }
#endif
#ifdef usepacketserver
          if (packetserver.hasClient()) {
            //for(i = 0; i < MAX_SRV_CLIENTS; i++){
              //find free/disconnected spot
              if (!packetclient || !packetclient.connected()){
                if(packetclient) packetclient.stop();
                packetclient = packetserver.available();
                if (!packetclient) DebugSerial.println("available broken");
                DebugSerial.print("### Telnet PACKET New client: ");
                DebugSerial.println(packetclient.remoteIP());
                telnetstate = telnetclientconnected;
                ledontime=250; ledofftime=250; ledcurrenttime = millis();
                //telnetnextrefreshtimestamp=millis()+telnetrefreshanyscreen;
                //break;
              }
            //}
          }
#endif
          if (userconnecttimestamp<millis()) {
            telnetstate = telnetturnoff;
          }
      break;
    case telnetclientconnected: 
        //TODO check if clients are still connected... else start client connect timer and fall back to listening mode
        hc = false;
        //for(i = 0; i < MAX_SRV_CLIENTS; i++){
          if (telnetclient && telnetclient.connected()) { hc=true; }
        //} //i 0 to maxclients
#ifdef userawserver
        if (rawclient && rawclient.connected()) { hc=true; }
#endif
#ifdef usepacketserver
        if (packetclient && packetclient.connected()) { hc=true; }
#endif

        if (!hc) {
          //no more clients, restart listening timer....
          telnetstate = telnetturnon;
          DebugSerial.println("### Telnet lost all clients - restarting telnet");
        } else {
          //still has clients, update telnet stuff
            if (telnetnextrefreshtimestamp<millis()) {
                  telnet_refreshscreen();
            } //telnetrefreshtimer

        } //else !hc
        //check clients for data
        //for(i = 0; i < MAX_SRV_CLIENTS; i++){
          if (telnetclient && telnetclient.connected()){
            if(telnetclient.available()){
              //get data from the telnet client and push it to the UART
              uint8_t tcmd = telnetclient.read();
              DebugSerial.printf("Telnet Command: %02X\r\n",tcmd);
              switch (tcmd) {
                case 0x73: telnetscreen=ts_statistics; break; //s
                case 0x74: telnetscreen=ts_telemetrie; break; //t
                case 0x65: telnetscreen=ts_esc_raw; break; //e
                case 0x62: telnetscreen=ts_bms_raw; break; //b
                case 0x6E: telnetscreen=ts_ble_raw; break; //n
                case 0x78: telnetscreen=ts_x1_raw; break; //x
                case 0x45: telnetscreen=ts_esc_array; break; //E
                case 0x42: telnetscreen=ts_bms_array; break; //B
                case 0x4E: telnetscreen=ts_ble_array; break; //N
                case 0x58: telnetscreen=ts_x1_array; break;  //X
                case 0x72: reset_statistics(); break;  //r
              } //switch
              //while(serverClients[i].available()) M365DebugSerial.write(serverClients[i].read());
            } //serverclients available
          } //if connected
        //}  //for i
#ifdef userawserver        
        //TODO: Check RAW Client for Data and send to bus? immediately send or queue and send in next free timeslot?
#endif        
      break;
    case telnetdisconnectclients:
        if (telnetclient) telnetclient.stop();
        #ifdef userawserver
          if (rawclient) rawclient.stop();
        #endif        
        #ifdef userawserver
          if (packetclient) packetclient.stop();
        #endif
        telnetstate = telnetturnoff;
      break;
    case telnetturnoff:
        #ifdef ESP32
          telnetserver.end();
        #endif
        #if defined userawserver && defined ESP32
          rawserver.end();
        #endif      
        #if defined usepacketserver && defined ESP32
          packetserver.end();
        #endif
        wlanstate=wlanturnoff; 
        telnetstate = telnetoff;
      break;
   } //switch (telnetstate)
   duration_telnet = micros()-timestamp_telnetstart;
} //handle_telnet
#endif //usetelnetserver

#ifdef ESP32
void WiFiEvent(WiFiEvent_t event) {
    //DebugSerial.printf("[WiFi-event] event: %d\n", event);
    switch(event) {
      case SYSTEM_EVENT_WIFI_READY: DebugSerial.println("### WifiEvent: SYSTEM_EVENT_WIFI_READY"); break; // < ESP32 WiFi ready
      case SYSTEM_EVENT_SCAN_DONE: DebugSerial.println("### WifiEvent: SYSTEM_EVENT_SCAN_DONE"); break; // < ESP32 finish scanning AP
      case SYSTEM_EVENT_STA_START: DebugSerial.println("### WifiEvent: SYSTEM_EVENT_STA_START"); break; // < ESP32 station start
      case SYSTEM_EVENT_STA_STOP: DebugSerial.println("### WifiEvent: SYSTEM_EVENT_STA_STOP"); break; // < ESP32 station stop
      case SYSTEM_EVENT_STA_CONNECTED: DebugSerial.println("### WifiEvent: SYSTEM_EVENT_STA_CONNECTED"); break; // < ESP32 station connected to AP
      case SYSTEM_EVENT_STA_DISCONNECTED: DebugSerial.println("### WifiEvent: SYSTEM_EVENT_STA_DISCONNECTED"); break; // < ESP32 station disconnected from AP
      case SYSTEM_EVENT_STA_AUTHMODE_CHANGE: DebugSerial.println("### WifiEvent: SYSTEM_EVENT_STA_AUTHMODE_CHANGE"); break; // < the auth mode of AP connected by ESP32 station changed
      case SYSTEM_EVENT_STA_GOT_IP: DebugSerial.println("### WifiEvent: SYSTEM_EVENT_STA_GOT_IP"); break; // < ESP32 station got IP from connected AP
      case SYSTEM_EVENT_STA_LOST_IP: DebugSerial.println("### WifiEvent: SYSTEM_EVENT_STA_LOST_IP"); break; // < ESP32 station lost IP and the IP is reset to 0
      case SYSTEM_EVENT_STA_WPS_ER_SUCCESS: DebugSerial.println("### WifiEvent: SYSTEM_EVENT_STA_WPS_ER_SUCCESS"); break; // < ESP32 station wps succeeds in enrollee mode
      case SYSTEM_EVENT_STA_WPS_ER_FAILED: DebugSerial.println("### WifiEvent: SYSTEM_EVENT_STA_WPS_ER_FAILED"); break; // < ESP32 station wps fails in enrollee mode
      case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT: DebugSerial.println("### WifiEvent: SYSTEM_EVENT_STA_WPS_ER_TIMEOUT"); break; // < ESP32 station wps timeout in enrollee mode
      case SYSTEM_EVENT_STA_WPS_ER_PIN: DebugSerial.println("### WifiEvent: SYSTEM_EVENT_STA_WPS_ER_PIN"); break; // < ESP32 station wps pin code in enrollee mode
      case SYSTEM_EVENT_AP_START: DebugSerial.println("### WifiEvent: SYSTEM_EVENT_AP_START"); break; // < ESP32 soft-AP start
      case SYSTEM_EVENT_AP_STOP: DebugSerial.println("### WifiEvent: SYSTEM_EVENT_AP_STOP"); break; // < ESP32 soft-AP stop
      case SYSTEM_EVENT_AP_STACONNECTED: apnumclientsconnected++; DebugSerial.println("### WifiEvent: SYSTEM_EVENT_AP_STACONNECTED"); break; // < a station connected to ESP32 soft-AP
      case SYSTEM_EVENT_AP_STADISCONNECTED: apnumclientsconnected--; DebugSerial.println("### WifiEvent: SYSTEM_EVENT_AP_STADISCONNECTED"); break; // < a station disconnected from ESP32 soft-AP
      case SYSTEM_EVENT_AP_PROBEREQRECVED: DebugSerial.println("### WifiEvent: SYSTEM_EVENT_AP_PROBEREQRECVED"); break; // < Receive probe request packet in soft-AP interface
      case SYSTEM_EVENT_GOT_IP6: DebugSerial.println("### WifiEvent: SYSTEM_EVENT_GOT_IP6"); break; // < ESP32 station or ap or ethernet interface v6IP addr is preferred
      case SYSTEM_EVENT_ETH_START: DebugSerial.println("### WifiEvent: SYSTEM_EVENT_ETH_START"); break; // < ESP32 ethernet start
      case SYSTEM_EVENT_ETH_STOP: DebugSerial.println("### WifiEvent: SYSTEM_EVENT_ETH_STOP"); break; // < ESP32 ethernet stop
      case SYSTEM_EVENT_ETH_CONNECTED: DebugSerial.println("### WifiEvent: SYSTEM_EVENT_ETH_CONNECTED"); break; // < ESP32 ethernet phy link up
      case SYSTEM_EVENT_ETH_DISCONNECTED: DebugSerial.println("### WifiEvent: SYSTEM_EVENT_ETH_DISCONNECTED"); break; // < ESP32 ethernet phy link down
      case SYSTEM_EVENT_ETH_GOT_IP: DebugSerial.println("### WifiEvent: SYSTEM_EVENT_ETH_GOT_IP"); break; // < ESP32 ethernet got IP from connected AP
      case SYSTEM_EVENT_MAX: DebugSerial.println("### WifiEvent: SYSTEM_EVENT_MAX"); break; //
    }
} //WiFiEvent
#endif

#ifdef ESP8266
void onStationConnected(const WiFiEventSoftAPModeStationConnected& evt) {
  DebugSerial.println("### WifiEvent: Station connected");
  apnumclientsconnected++;
}

void onStationDisconnected(const WiFiEventSoftAPModeStationDisconnected& evt) {
  DebugSerial.println("### WifiEvent: Station disconnected");
  apnumclientsconnected--;
}
#endif

void handle_wlan() {
  switch(wlanstate) {
    case wlanoff: 
      break;
    case wlanturnapon:
        WiFi.disconnect(true);
        delay(500);
        WiFi.mode(WIFI_OFF);
        yield();
        WiFi.mode(WIFI_AP);
        yield();
        apnumclientsconnected=0;
        apnumclientsconnectedlast=0;
        #ifdef ESP32
          WiFi.onEvent(WiFiEvent);
        #endif
        #ifdef ESP8266
          stationConnectedHandler = WiFi.onSoftAPModeStationConnected(&onStationConnected);
          stationDisconnectedHandler = WiFi.onSoftAPModeStationDisconnected(&onStationDisconnected);
        #endif
        WiFi.softAP(apssid, appassword);
        yield();
        //IPAddress myIP = WiFi.softAPIP();
        DebugSerial.printf("### WLAN AP\r\nSSID: %s, AP IP address: ", apssid);  DebugSerial.println( WiFi.softAPIP());
#ifdef useoledxxx //POPUP
        display1.clearDisplay();
        display1.setFont();
        display1.println("AP Mode");
        display1.print( WiFi.softAPIP());
        display1.display();
#endif        
#ifdef usetelnetserver
        if (telnetstate==telnetoff) {
          telnetstate = telnetturnon;
        }
#endif        
        wlanconnecttimestamp = millis()+wlanapconnecttimeout;            
        wlanstate = wlanap;
        ArduinoOTA.begin();
        //start OTA
      break;
    case wlanturnstaon:
        DebugSerial.println("### Searching for known wlan");
        WiFi.persistent(false);
        WiFi.mode(WIFI_STA);
        #ifdef staticip
          if (!WiFi.config(ip, gateway, subnet, dns)) {
            DebugSerial.println("### STA Failed to configure");
          }
        #endif
        currentssidindex = 0;
        #ifdef ESP32
          WiFi.onEvent(WiFiEvent); //not really needed in STA mode
          WiFi.setHostname("M365OLEDESP32");
        #endif
        #ifdef ESP8266
          WiFi.hostname("M365OLEDESP8266");
        #endif

        WiFi.begin(ssid1, password1);
        wlanconnecttimestamp = millis()+wlanconnecttimeout;
#ifdef useoledxxx //POPUP
        display1.clearDisplay();
        display1.setFont();
        display1.printf("W %d",currentssidindex);
        display1.display();  
#endif        
        DebugSerial.printf("###  WLan searching for ssidindex %d\r\n",currentssidindex);
        wlanstate = wlansearching;
        ledontime=50; ledofftime=450; ledcurrenttime = millis();
        //start wlan connect timeout
      break;
    case wlansearching:
        if (WiFi.status() != WL_CONNECTED) {
          if (wlanconnecttimestamp<millis()) {
            //timeout, test next wlan
            currentssidindex++;
            if (currentssidindex<maxssids) {
              if (currentssidindex==1) { WiFi.begin(ssid2, password2); }
              if (currentssidindex==2) { WiFi.begin(ssid3, password3); }
              wlanconnecttimestamp = millis()+wlanconnecttimeout;
#ifdef useoledxxx //POPUP
              display1.clearDisplay();
              display1.setFont();
              display1.printf("W %d",currentssidindex);
              display1.display();
#endif              
              DebugSerial.printf("### WLan searching for ssidindex %d\r\n",currentssidindex);
            } else {
              DebugSerial.println("### WLAN search failed, starting Access Point");
              wlanstate = wlanturnapon;
            }
          }
        } else {
          wlanstate = wlanconnected;
          DebugSerial.print("### WLAN Connected to ");
          DebugSerial.print(WiFi.SSID());
          DebugSerial.print(", IP is ");
          DebugSerial.println(WiFi.localIP());
#ifdef useoledxxx //POPUP
          display1.clearDisplay();
          display1.setFont();
          display1.print("WC ");
          display1.println(WiFi.SSID());
          display1.print(WiFi.localIP());
          display1.display();
#endif          
#ifdef usetelnetserver
          if (telnetstate==telnetoff) {
                telnetstate = telnetturnon;
            }
#endif            
          ledontime=500; ledofftime=500; ledcurrenttime = millis();
          ArduinoOTA.begin();
        }
      break;
    case wlanconnected:
          if (WiFi.status() != WL_CONNECTED) {
              DebugSerial.println("### WiFi lost connection!");
#ifdef useoledxxx //POPUP
              display1.clearDisplay();
              display1.setFont();
              display1.println("W CONN LOST");
              display1.print(WiFi.SSID());
              display1.display();
#endif
#ifdef usetelnetserver
              if (telnetstate!=telnetoff) {
                  telnetstate = telnetdisconnectclients;
              }
#endif
             wlanstate = wlanturnoff;
          }
      break;
    case wlanap: //make 2 states - waiting for clients & has clients
        //without wifi events: apnumclientsconnected=WiFi.softAPgetStationNum();
        if (apnumclientsconnected!=apnumclientsconnectedlast) {
          //num of connect clients changed
          apnumclientsconnectedlast = apnumclientsconnected;
          DebugSerial.printf("### AP Clients connected changed: %d -> %d\r\n",apnumclientsconnectedlast, apnumclientsconnected);
#ifdef useoledxxx //POPUP
          display1.clearDisplay();
          display1.setFont();
          display1.printf("AP Clients %d",apnumclientsconnected);
          display1.display();
#endif          
          if (apnumclientsconnected==0) {
              //no one connnected, but there was someone connected.... restart timeout
              wlanconnecttimestamp = millis()+wlanapconnecttimeout;
          } //if (apnumclientsconnected==0) 
        } //if (apnumclientsconnected!=apnumclientsconnectedlast)
        yield();
        if (apnumclientsconnected==0) {
          if (wlanconnecttimestamp<millis()) {
            wlanstate=wlanturnoff; 
          } //if (wlanconnecttimestamp<millis())
        } //if (apnumclientsconnected==0)
      break;
    case wlanturnoff:
#ifdef usemqtt
        if (client.connected) {
          client.disconnect();
        }        
#endif
#ifdef usetelnetserver
        if (telnetclient) telnetclient.stop();
#endif
#ifdef userawserver
        if (rawclient) rawclient.stop();
#endif
#ifdef usepacketserver
        if (packetclient) packetclient.stop();
#endif


        #ifdef ESP32
          ArduinoOTA.end();
        #endif
        WiFi.softAPdisconnect();
        WiFi.disconnect();
        WiFi.mode(WIFI_OFF);
        DebugSerial.println("### WIFI OFF");
#ifdef useoledxxx //POPUP
        display1.clearDisplay();
        display1.setFont();
        display1.print("WLAN OFF");
        display1.display();
#endif
        wlanstate=wlanoff; 
        //STOP OTA    
      break;
  } //switch wlanstate
} //handle_wlan


#ifdef debug_dump_states //dump state machine states to Serial Port on change
  void print_states() {
    if (wlanstate!=wlanstateold) {
      DebugSerial.printf("### WLANSTATE %d -> %d\r\n",wlanstateold,wlanstate);
      wlanstateold=wlanstate;
    }
#ifdef usetelnetserver    
    if (telnetstate!=telnetstateold) {
      DebugSerial.printf("### TELNETSTATE %d -> %d\r\n",telnetstateold,telnetstate);
      telnetstateold=telnetstate;
    }
#endif    
    if (m365receiverstate!=m365receiverstateold) {
      DebugSerial.printf("### M365RecState: %d -> %d\r\n",m365receiverstateold,m365receiverstate);
      m365receiverstateold=m365receiverstate;
    }
    if (m365packetstate!=m365packetstateold) {
      DebugSerial.printf("M365PacketState: %d -> %d\r\n",m365packetstateold,m365packetstate);
      m365packetstateold=m365packetstate;
    }    
  }
#endif



void setup() {



  pinMode(led,OUTPUT);
  digitalWrite(led,LOW);
  #ifdef ESP32
    DebugSerial.begin(115200);
  #endif
  #ifdef ESP8266
    DebugSerial.begin(115200);
    DebugSerial.setDebugOutput(true);
  #endif
  DebugSerial.println(swversion);
  wlanstate=wlanturnstaon;
  uint64_t cit;
  #ifdef ESP32
    cit = ESP.getEfuseMac();
    sprintf(chipid,"%02X%02X%02X",(uint8_t)(cit>>24),(uint8_t)(cit>>32),(uint8_t)(cit>>40));
    DebugSerial.println(chipid);
    sprintf(mac,"%02X%02X%02X%02X%02X%02X",(uint8_t)(cit),(uint8_t)(cit>>8),(uint8_t)(cit>>16),(uint8_t)(cit>>24),(uint8_t)(cit>>32),(uint8_t)(cit>>40));
  #elif defined(ESP8266)
    cit = ESP.getChipId();
    sprintf(chipid,"%02X%02X%02X",(uint8_t)(cit>>24),(uint8_t)(cit>>32),(uint8_t)(cit>>40));
    byte mc[6];
    WiFi.macAddress(mc);
    sprintf(mac, "%02X%02X%02X%02X%02X%02X", mc[0], mc[1], mc[2], mc[3], mc[4], mc[5]);
  #endif
  start_m365();
  ArduinoOTA.onStart([]() {
    DebugSerial.println("OTA Start");
#ifdef usemqtt
    if (client.connected()) {
      sprintf(tmp1, "n/%d/OTAStart", mqtt_clientID);
      client.publish(tmp1, "OTA Starting");
    }
#endif
/*#ifdef useoled1
    display1.clearDisplay();
    display1.setTextSize(2);
    display1.setTextColor(WHITE);
    display1.setCursor(0,0);
    display1.setFont();
    display1.println("OTA Start");
    display1.display();
#endif*/
#ifdef useoled1
    display1.clearDisplay();
    display1.display();
#endif
#ifdef useoled2
    display2.clearDisplay();
    display2.display();
#endif
  });
  ArduinoOTA.onEnd([]() {
    DebugSerial.println("\nOTA End");
#ifdef usemqtt
    if (client.connected()) {
      sprintf(tmp1, "n/%d/OTAEND", mqtt_clientID);
      client.publish(tmp1, "OTA Done");
    }
#endif
/*#ifdef useoled1
    display1.clearDisplay();
    display1.setTextSize(2);
    display1.setTextColor(WHITE);
    display1.setCursor(0,0);
    display1.setFont();
    display1.println("OTA End");
    display1.display();
#endif*/
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    DebugSerial.printf("OTA %02u%%\r", (progress / (total / 100)));
#ifdef useoled1
    if ((progress / (total / 100))>lastprogress) {
      lastprogress = progress / (total / 100);
    }
    display1.clearDisplay();
    display1.setTextSize(1);
    display1.setTextColor(WHITE);
    display1.setCursor(0,10);
    display1.setFont();
    display1.print("  Updating  Firmware");
    display1.setCursor(54,50);
    display1.printf("%02u%%", lastprogress);
    display1.drawRect(14,25,100,10,WHITE);
    //display1.drawRect(15,33,14+(uint8_t)lastprogress,39,WHITE);
    display1.fillRect(14,26,(uint8_t)lastprogress,8,WHITE);
    display1.display();
#endif
  });
  ArduinoOTA.onError([](ota_error_t error) {
    if (error == OTA_AUTH_ERROR) sprintf(tmp1,"%s","Auth Failed");
    else if (error == OTA_BEGIN_ERROR) sprintf(tmp1,"%s","Begin Failed");
    else if (error == OTA_CONNECT_ERROR) sprintf(tmp1,"%s","Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) sprintf(tmp1,"%s","Receive Failed");
    else if (error == OTA_END_ERROR) sprintf(tmp1,"%s","End Failed");
    sprintf(tmp2, "OTA Error [%u]: %s", error, tmp1);
    DebugSerial.println(tmp2);
#ifdef usemqtt
    if (client.connected()) {
      sprintf(tmp1, "n/%d/OTAError", mqtt_clientID);
      client.publish(tmp1, tmp2);
      yield(); //give wifi/mqtt a chance to send before reboot
    }
#endif

  });
  ArduinoOTA.setPassword(OTApwd);
  sprintf(tmp2, "es-m36-%s",mac);
  ArduinoOTA.setHostname(tmp2);
} //setup

void handle_led() {
  if (millis()>ledcurrenttime) {
    if (digitalRead(led)) {
      digitalWrite(led,LOW);
      ledcurrenttime = millis()+ledontime;
    } else {
      digitalWrite(led,HIGH);
      ledcurrenttime = millis()+ledofftime;
    }
  }
}


bool release_throttle(void *) {

  analogWrite(throttlepin, 100); //Keep throttle open for 10% to disable KERS. 
  motionstate = motionready;
    return false; // false to stop
}


void motion_control() {

if (bleparsed->brake>1) {
  
  analogWrite(throttlepin, 0); //close throttle directly when break is touched
  
  motionstate = motionbreaking;
  timer.cancel();
  
}
  else {
    motionstate = motionready;
  }
  


  
// Check if auto throttle is off and speed is increasing
 if (motionstate==motionready) {
 
 if (escparsed->speed>oldspeed) {


// Check if speed is at least 5 km/h
    if (newdata & (abs((float)escparsed->speed/1000.0f)>5.1f)) {
  // Open throttle for 5 seconds
  analogWrite(throttlepin, 1023);
  timer.every(5000, release_throttle);
  motionstate = motionbusy;

        }    
      }
    }

  oldspeed = escparsed->speed;
 }




void loop() {
  timer.tick();
  timestamp_mainloopstart=micros();
  handle_wlan();
#ifdef usetelnetserver
  handle_telnet();
#endif  
  while (M365Serial.available()) { //decode anything we received as long as  there's data in the buffer
    m365_receiver(); 
    m365_handlepacket();
  }
  if (newdata) {
    m365_updatestats();
  }

  ArduinoOTA.handle();
  handle_led();
  motion_control();
#ifdef usemqtt
  client.loop();
#endif
  #ifdef debug_dump_states
    print_states();
  #endif
  duration_mainloop=micros()-timestamp_mainloopstart;
}
