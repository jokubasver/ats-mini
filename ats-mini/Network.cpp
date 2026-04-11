#include "Common.h"
#include "Storage.h"
#include "Themes.h"
#include "Utils.h"
#include "Menu.h"
#include "Draw.h"

#include <WiFi.h>
#include <WiFiMulti.h>
#include <WiFiUdp.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <NTPClient.h>
#include <ESPmDNS.h>
#include <esp_timer.h>

#define CONNECT_TIME  3000  // Time of inactivity to start connecting WiFi

// Audio streaming via WebSocket
#define AUDIO_PIN          11    // GPIO11 – ADC input from NS4160 pin 8 via RC filter
#define AUDIO_SAMPLE_RATE  8000  // 8 kHz sample rate
#define AUDIO_CHUNK_SIZE   512   // Samples per WebSocket message (64 ms at 8 kHz)

// GPIO11 on ESP32-S3: ADC2 channel 0.  We use analogRead (oneshot driver) rather
// than the ADC continuous DMA driver because the continuous driver bypasses the
// ADC2/WiFi arbitration layer and causes an immediate panic on ESP32-S3 when WiFi
// is active.  analogRead handles the coexistence lock internally.

static uint8_t             audioChunk[2][AUDIO_CHUNK_SIZE]; // double buffer
static volatile int        audioChunkIdx = 0;               // active write buffer
static volatile int        audioChunkPos = 0;
static esp_timer_handle_t  audioTimer    = nullptr;
static TaskHandle_t        audioSampleH  = nullptr;
static volatile int32_t    audioDcEst    = 128 << 8;        // Q8 DC estimate for IIR HP filter
static volatile bool       audioRunning  = false;
static AsyncWebSocket      audioWS("/audiows");
static QueueHandle_t       audioSendQ    = nullptr;         // completed chunk indices → send task
static TaskHandle_t        audioSendH    = nullptr;

WiFiMulti wifiMulti;

//
// Access Point (AP) mode settings
//
static const char *apSSID    = RECEIVER_NAME;
static const char *apPWD     = 0;       // No password
static const int   apChannel = 10;      // WiFi channel number (1..13)
static const bool  apHideMe  = false;   // TRUE: disable SSID broadcast
static const int   apClients = 3;       // Maximum simultaneous connected clients

static uint16_t ajaxInterval = 2500;

static bool itIsTimeToWiFi = false; // TRUE: Need to connect to WiFi
static uint32_t connectTime = millis();

// Settings
String loginUsername = "";
String loginPassword = "";

// AsyncWebServer object on port 80
AsyncWebServer server(80);

// NTP Client to get time
WiFiUDP ntpUDP;
NTPClient ntpClient(ntpUDP, "pool.ntp.org");

static bool wifiInitAP();
static bool wifiConnect();
static void webInit();

static void webSetConfig(AsyncWebServerRequest *request);

static void startAudioSampling();
static void stopAudioSampling();

static const String webInputField(const String &name, const String &value, bool pass = false);
static const String webStyleSheet();
static const String webPage(const String &body);
static const String webUtcOffsetSelector();
static const String webThemeSelector();
static const String webAudioPage();
static const String webRadioPage();
static const String webMemoryPage();
static const String webConfigPage();

//
// Delayed WiFi connection
//
void netRequestConnect()
{
  connectTime = millis();
  itIsTimeToWiFi = true;
}

void netTickTime()
{
  // Connect to WiFi if requested
  if(itIsTimeToWiFi && ((millis() - connectTime) > CONNECT_TIME))
  {
    netInit(wifiModeIdx);
    connectTime = millis();
    itIsTimeToWiFi = false;
  }

  // Periodically clean up stale WebSocket connections
  audioWS.cleanupClients();
}

//
// Get current connection status
// (-1 - not connected, 0 - disabled, 1 - connected, 2 - connected to network)
//
int8_t getWiFiStatus()
{
  wifi_mode_t mode = WiFi.getMode();

  switch(mode)
  {
    case WIFI_MODE_NULL:
      return(0);
    case WIFI_AP:
      return(WiFi.softAPgetStationNum()? 1 : -1);
    case WIFI_STA:
      return(WiFi.status()==WL_CONNECTED? 2 : -1);
    case WIFI_AP_STA:
      return((WiFi.status()==WL_CONNECTED)? 2 : WiFi.softAPgetStationNum()? 1 : -1);
    default:
      return(-1);
  }
}

char *getWiFiIPAddress()
{
  static char ip[16];
  return strcpy(ip, WiFi.status()==WL_CONNECTED ? WiFi.localIP().toString().c_str() : "");
}

//
// Stop WiFi hardware
//
void netStop()
{
  wifi_mode_t mode = WiFi.getMode();

  MDNS.end();

  // If network connection up, shut it down
  if((mode==WIFI_STA) || (mode==WIFI_AP_STA))
    WiFi.disconnect(true);

  // If access point up, shut it down
  if((mode==WIFI_AP) || (mode==WIFI_AP_STA))
    WiFi.softAPdisconnect(true);

  WiFi.mode(WIFI_MODE_NULL);
}

//
// Initialize WiFi network and services
//
void netInit(uint8_t netMode, bool showStatus)
{
  // Always disable WiFi first
  netStop();

  switch(netMode)
  {
    case NET_OFF:
      // Do not initialize WiFi if disabled
      return;
    case NET_AP_ONLY:
      // Start WiFi access point if requested
      WiFi.mode(WIFI_AP);
      // Let user see connection status if successful
      if(wifiInitAP() && showStatus) delay(2000);
      break;
    case NET_AP_CONNECT:
      // Start WiFi access point if requested
      WiFi.mode(WIFI_AP_STA);
      // Let user see connection status if successful
      if(wifiInitAP() && showStatus) delay(2000);
      break;
    default:
      // No access point
      WiFi.mode(WIFI_STA);
      break;
  }

  // Initialize WiFi and try connecting to a network
  if(netMode>NET_AP_ONLY && wifiConnect())
  {
    // Let user see connection status if successful
    if(netMode!=NET_SYNC && showStatus) delay(2000);

    // NTP time updates will happen every 5 minutes
    ntpClient.setUpdateInterval(5*60*1000);

    // Get NTP time from the network
    clockReset();
    for(int j=0 ; j<10 ; j++)
      if(ntpSyncTime()) break; else delay(500);
  }

  // If only connected to sync...
  if(netMode==NET_SYNC)
  {
    // Drop network connection
    WiFi.disconnect(true);
    WiFi.mode(WIFI_MODE_NULL);
  }
  else
  {
    // Initialize web server for remote configuration
    webInit();

    // Initialize mDNS
    MDNS.begin("atsmini"); // Set the hostname to "atsmini.local"
    MDNS.addService("http", "tcp", 80);
  }
}

//
// Returns TRUE if NTP time is available
//
bool ntpIsAvailable()
{
  return(ntpClient.isTimeSet());
}

//
// Update NTP time and synchronize clock with NTP time
//
bool ntpSyncTime()
{
  if(WiFi.status()==WL_CONNECTED)
  {
    ntpClient.update();

    if(ntpClient.isTimeSet())
      return(clockSet(
        ntpClient.getHours(),
        ntpClient.getMinutes(),
        ntpClient.getSeconds()
      ));
  }
  return(false);
}

//
// Initialize WiFi access point (AP)
//
static bool wifiInitAP()
{
  // These are our own access point (AP) addresses
  IPAddress ip(10, 1, 1, 1);
  IPAddress gateway(10, 1, 1, 1);
  IPAddress subnet(255, 255, 255, 0);

  // Start as access point (AP)
  WiFi.softAP(apSSID, apPWD, apChannel, apHideMe, apClients);
  WiFi.softAPConfig(ip, gateway, subnet);

  drawScreen(
    ("Use Access Point " + String(apSSID)).c_str(),
    ("IP : " + WiFi.softAPIP().toString() + " or atsmini.local").c_str()
  );

  ajaxInterval = 2500;
  return(true);
}

//
// Connect to a WiFi network
//
static bool wifiConnect()
{
  String status = "Connecting to WiFi network...";

  // Clean credentials
  wifiMulti.APlistClean();

  // Get the preferences
  prefs.begin("network", true, STORAGE_PARTITION);
  loginUsername = prefs.getString("loginusername", "");
  loginPassword = prefs.getString("loginpassword", "");

  // Try connecting to known WiFi networks
  for(int j=0 ; (j<3) ; j++)
  {
    char nameSSID[16], namePASS[16];
    sprintf(nameSSID, "wifissid%d", j+1);
    sprintf(namePASS, "wifipass%d", j+1);

    String ssid = prefs.getString(nameSSID, "");
    String password = prefs.getString(namePASS, "");

    if(ssid != "")
      wifiMulti.addAP(ssid.c_str(), password.c_str());
  }

  // Done with preferences
  prefs.end();

  drawScreen(status.c_str());

  // If failed connecting to WiFi network...
  if (wifiMulti.run() != WL_CONNECTED)
  {
    // WiFi connection failed
    drawScreen(status.c_str(), "No WiFi connection");
    // Done
    return(false);
  }
  else
  {
    // WiFi connection succeeded
    drawScreen(
      ("Connected to WiFi network (" + WiFi.SSID() + ")").c_str(),
      ("IP : " + WiFi.localIP().toString() + " or atsmini.local").c_str()
    );
    // Done
    ajaxInterval = 1000;
    return(true);
  }
}

//
// Audio send task – runs on core 1 at FreeRTOS priority 2 (same as sampler, above loop).
// Receives completed chunk indices from the sampling task via a queue and
// sends them over the WebSocket.  Keeping the potentially-slow binaryAll() here
// prevents any TCP-stack stall from blocking the sampling task.
//
static void audioSendTask(void *)
{
  int idx;
  while(xQueueReceive(audioSendQ, &idx, portMAX_DELAY) == pdTRUE)
  {
    if(idx < 0) break;  // poison pill – stop signal
    if(audioWS.count() > 0)
      audioWS.binaryAll(audioChunk[idx], AUDIO_CHUNK_SIZE);
  }
  audioSendH = nullptr;
  vTaskDelete(nullptr);
}

//
// esp_timer callback – fires at AUDIO_SAMPLE_RATE Hz from the esp_timer task.
// Sends a task notification to audioSampleTask so the actual ADC read happens
// in a regular task context (required because analogRead uses a mutex internally
// to arbitrate ADC2 access with the WiFi driver).
//
static void audioTimerCB(void *)
{
  if(audioSampleH)
    xTaskNotifyGive(audioSampleH);
}

//
// Sampling task – woken by audioTimerCB at 8 kHz.
// Calls analogRead, applies the IIR DC-blocking filter, and fills the double-
// buffer.  When a chunk is full it queues the index to audioSendTask.
//
static void audioSampleTask(void *)
{
  analogSetPinAttenuation(AUDIO_PIN, ADC_11db);  // 0–3.3 V range

  while(ulTaskNotifyTake(pdTRUE, portMAX_DELAY) && audioRunning)
  {
    uint8_t raw = (uint8_t)(analogRead(AUDIO_PIN) >> 4);  // 12-bit → 8-bit unsigned

    // IIR DC-blocking high-pass (Q8 fixed-point, α = 255/256, cutoff ≈ 5 Hz).
    audioDcEst += (int32_t)raw - (audioDcEst >> 8);
    int s = (int)raw - (audioDcEst >> 8) + 128;
    if(s < 0)   s = 0;
    if(s > 255) s = 255;

    audioChunk[audioChunkIdx][audioChunkPos++] = (uint8_t)s;
    if(audioChunkPos >= AUDIO_CHUNK_SIZE)
    {
      int sendIdx    = audioChunkIdx;
      audioChunkIdx ^= 1;
      audioChunkPos  = 0;
      xQueueSend(audioSendQ, &sendIdx, 0);
    }
  }

  audioSampleH = nullptr;
  vTaskDelete(nullptr);
}

static void startAudioSampling()
{
  if(audioTimer) return;
  audioChunkIdx = 0;
  audioChunkPos = 0;
  audioDcEst    = 128 << 8;
  audioRunning  = true;

  // Depth 2: one chunk queued for the send task while the next is filling.
  audioSendQ = xQueueCreate(2, sizeof(int));

  // Send task: priority 2 (above loop) so display-loop SPI activity cannot
  // delay chunk delivery to the WebSocket client.
  xTaskCreatePinnedToCore(audioSendTask, "audioSend", 8192, nullptr, 2, &audioSendH, 1);

  // Sampling task: priority 3 so it preempts the send task and the main loop.
  // Pinned to core 0 to keep the WiFi/TCP stack on core 1 unimpeded.
  xTaskCreatePinnedToCore(audioSampleTask, "audioSample", 4096, nullptr, 3, &audioSampleH, 0);

  // Periodic timer at exactly AUDIO_SAMPLE_RATE Hz to trigger each ADC read.
  esp_timer_create_args_t timerArgs = {};
  timerArgs.callback = audioTimerCB;
  timerArgs.name     = "audioSample";
  esp_timer_create(&timerArgs, &audioTimer);
  esp_timer_start_periodic(audioTimer, 1000000ULL / AUDIO_SAMPLE_RATE);
}

static void stopAudioSampling()
{
  if(!audioTimer) return;

  // Stop the timer first so no new notifications arrive after we clear the flag.
  audioRunning = false;
  esp_timer_stop(audioTimer);
  esp_timer_delete(audioTimer);
  audioTimer = nullptr;

  // Wake the sampling task one last time so it can see audioRunning == false.
  if(audioSampleH) xTaskNotifyGive(audioSampleH);
  for(int i = 0; i < 200 && audioSampleH; i++) vTaskDelay(1);

  // Send a poison pill to wake and stop the send task.
  if(audioSendQ)
  {
    int stop = -1;
    xQueueSend(audioSendQ, &stop, pdMS_TO_TICKS(100));
  }
  for(int i = 0; i < 200 && audioSendH; i++) vTaskDelay(1);

  if(audioSendQ){ vQueueDelete(audioSendQ); audioSendQ = nullptr; }
  audioChunkIdx = 0;
  audioChunkPos = 0;
}

static void audioWSEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                         AwsEventType type, void *arg, uint8_t *data, size_t len)
{
  if(type == WS_EVT_CONNECT)
    startAudioSampling();
  else if(type == WS_EVT_DISCONNECT && audioWS.count() <= 1)
    stopAudioSampling();
}

//
// Initialize internal web server
//
static void webInit()
{
  server.on("/", HTTP_ANY, [] (AsyncWebServerRequest *request) {
    request->send(200, "text/html", webRadioPage());
  });

  server.on("/memory", HTTP_ANY, [] (AsyncWebServerRequest *request) {
    request->send(200, "text/html", webMemoryPage());
  });

  server.on("/config", HTTP_ANY, [] (AsyncWebServerRequest *request) {
    if(loginUsername != "" && loginPassword != "")
      if(!request->authenticate(loginUsername.c_str(), loginPassword.c_str()))
        return request->requestAuthentication();
    request->send(200, "text/html", webConfigPage());
  });

  server.on("/audio", HTTP_ANY, [] (AsyncWebServerRequest *request) {
    request->send(200, "text/html", webAudioPage());
  });

  server.onNotFound([] (AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not found");
  });

  // This method saves configuration form contents
  server.on("/setconfig", HTTP_ANY, webSetConfig);

  // WebSocket endpoint for audio streaming
  audioWS.onEvent(audioWSEvent);
  server.addHandler(&audioWS);

  // Start web server
  server.begin();
}

void webSetConfig(AsyncWebServerRequest *request)
{
  uint32_t prefsSave = 0;

  // Start modifying preferences
  prefs.begin("network", false, STORAGE_PARTITION);

  // Save user name and password
  if(request->hasParam("username", true) && request->hasParam("password", true))
  {
    loginUsername = request->getParam("username", true)->value();
    loginPassword = request->getParam("password", true)->value();

    prefs.putString("loginusername", loginUsername);
    prefs.putString("loginpassword", loginPassword);
  }

  // Save SSIDs and their passwords
  bool haveSSID = false;
  for(int j=0 ; j<3 ; j++)
  {
    char nameSSID[16], namePASS[16];

    sprintf(nameSSID, "wifissid%d", j+1);
    sprintf(namePASS, "wifipass%d", j+1);

    if(request->hasParam(nameSSID, true) && request->hasParam(namePASS, true))
    {
      String ssid = request->getParam(nameSSID, true)->value();
      String pass = request->getParam(namePASS, true)->value();
      prefs.putString(nameSSID, ssid);
      prefs.putString(namePASS, pass);
      haveSSID |= ssid != "" && pass != "";
    }
  }

  // Save time zone
  if(request->hasParam("utcoffset", true))
  {
    String utcOffset = request->getParam("utcoffset", true)->value();
    utcOffsetIdx = utcOffset.toInt();
    clockRefreshTime();
    prefsSave |= SAVE_SETTINGS;
  }

  // Save theme
  if(request->hasParam("theme", true))
  {
    String theme = request->getParam("theme", true)->value();
    themeIdx = theme.toInt();
    prefsSave |= SAVE_SETTINGS;
  }

  // Save scroll direction and menu zoom
  scrollDirection = request->hasParam("scroll", true)? -1 : 1;
  zoomMenu        = request->hasParam("zoom", true);
  prefsSave |= SAVE_SETTINGS;

  // Done with the preferences
  prefs.end();

  // Save preferences immediately
  prefsRequestSave(prefsSave, true);

  // Show config page again
  request->redirect("/config");

  // If we are currently in AP mode, and infrastructure mode requested,
  // and there is at least one SSID / PASS pair, request network connection
  if(haveSSID && (wifiModeIdx>NET_AP_ONLY) && (WiFi.status()!=WL_CONNECTED))
    netRequestConnect();
}

static const String webInputField(const String &name, const String &value, bool pass)
{
  String newValue(value);

  newValue.replace("\"", "&quot;");
  newValue.replace("'", "&apos;");

  return(
    "<INPUT TYPE='" + String(pass? "PASSWORD":"TEXT") + "' NAME='" +
    name + "' VALUE='" + newValue + "'>"
  );
}

static const String webStyleSheet()
{
  return
"BODY"
"{"
  "margin: 0;"
  "padding: 0;"
"}"
"H1"
"{"
  "text-align: center;"
"}"
"TABLE"
"{"
  "width: 100%;"
  "max-width: 768px;"
  "border: 0px;"
  "margin-left: auto;"
  "margin-right: auto;"
"}"
"TH, TD"
"{"
  "padding: 0.5em;"
"}"
"TH.HEADING"
"{"
  "background-color: #80A0FF;"
  "column-span: all;"
  "text-align: center;"
"}"
"TD.LABEL"
"{"
  "text-align: right;"
"}"
"INPUT[type=text], INPUT[type=password], SELECT"
"{"
  "width: 95%;"
  "padding: 0.5em;"
"}"
"INPUT[type=submit]"
"{"
  "width: 50%;"
  "padding: 0.5em 0;"
"}"
".CENTER"
"{"
  "text-align: center;"
"}"
;
}

static const String webPage(const String &body)
{
  return
"<!DOCTYPE HTML>"
"<HTML>"
"<HEAD>"
  "<META CHARSET='UTF-8'>"
  "<META NAME='viewport' CONTENT='width=device-width, initial-scale=1.0'>"
  "<TITLE>ATS-Mini Config</TITLE>"
  "<STYLE>" + webStyleSheet() + "</STYLE>"
"</HEAD>"
"<BODY STYLE='font-family: sans-serif;'>" + body + "</BODY>"
"</HTML>"
;
}

static const String webUtcOffsetSelector()
{
  String result = "";

  for(int i=0 ; i<getTotalUTCOffsets(); i++)
  {
    char text[64];

    sprintf(text,
      "<OPTION VALUE='%d'%s>%s</OPTION>",
      i, utcOffsetIdx==i? " SELECTED":"",
      utcOffsets[i].desc
    );

    result += text;
  }

  return(result);
}

static const String webThemeSelector()
{
  String result = "";

  for(int i=0 ; i<getTotalThemes(); i++)
  {
    char text[64];

    sprintf(text,
      "<OPTION VALUE='%d'%s>%s</OPTION>",
       i, themeIdx==i? " SELECTED":"", theme[i].name
    );

    result += text;
  }

  return(result);
}

static const String webAudioPage()
{
  return webPage(
"<H1>ATS-Mini Audio Stream</H1>"
"<P ALIGN='CENTER'>"
  "<A HREF='/'>Status</A>&nbsp;|&nbsp;"
  "<A HREF='/memory'>Memory</A>&nbsp;|&nbsp;"
  "<A HREF='/config'>Config</A>"
"</P>"
"<TABLE>"
"<TR><TD CLASS='CENTER'>"
  "<BUTTON ID='sta' ONCLICK='startAudio()'>&#9654; Start</BUTTON>"
  "&nbsp;"
  "<BUTTON ID='sto' ONCLICK='stopAudio()' DISABLED>&#9632; Stop</BUTTON>"
"</TD></TR>"
"<TR><TD CLASS='CENTER' ID='st'>Press Start to listen</TD></TR>"
"</TABLE>"
"<SCRIPT>"
// AudioBufferSourceNode approach – each incoming chunk is decoded into a
// Float32 AudioBuffer and scheduled to start exactly where the previous one
// ended via the 'npt' (next-play-time) cursor.  Because AudioBufferSourceNode
// is driven by the browser's dedicated real-time audio thread (not the JS main
// thread), pitch and timing are always correct regardless of main-thread load.
// The DC-blocking filter on the firmware side keeps every chunk centred at
// mid-scale, so there are no level discontinuities at buffer boundaries and
// therefore no audible clicks between chunks.
// JITTER: initial pre-buffer (seconds).  The first chunk is scheduled 300 ms
// ahead of ctx.currentTime to absorb WiFi jitter.  The same value is used as
// a recovery gap if a chunk arrives late (npt has already passed).
"var ctx=null,ws=null,npt=0,JITTER=0.3;"
"function startAudio(){"
  "if(ws)return;"
  "if(!ctx||ctx.state==='closed')ctx=new(window.AudioContext||window.webkitAudioContext)({sampleRate:8000});"
  "ctx.resume();"
  "ws=new WebSocket((location.protocol==='https:'?'wss://':'ws://')+location.host+'/audiows');"
  "ws.binaryType='arraybuffer';"
  "ws.onopen=function(){"
    "if(ctx&&ctx.state==='suspended')ctx.resume();"
    "npt=0;"
    "document.getElementById('st').textContent='Streaming...';"
  "};"
  "ws.onmessage=function(e){"
    "if(!ctx||ctx.state==='closed')return;"
    "var b=new Uint8Array(e.data);"
    "var a=ctx.createBuffer(1,b.length,8000);"
    "var d=a.getChannelData(0);"
    "for(var i=0;i<b.length;i++)d[i]=b[i]/128.0-1.0;"
    "var s=ctx.createBufferSource();"
    "s.buffer=a;s.connect(ctx.destination);"
    // Schedule: if npt is in the past (late chunk or first chunk) add JITTER
    // to give the audio thread time to prepare; otherwise chain immediately.
    "var n=ctx.currentTime;if(npt<n+0.005)npt=n+JITTER;"
    "s.start(npt);npt+=b.length/8000;"
  "};"
  "ws.onclose=function(){"
    "ws=null;npt=0;"
    "document.getElementById('sta').disabled=false;"
    "document.getElementById('sto').disabled=true;"
    "document.getElementById('st').textContent='Disconnected - click Start to retry';"
  "};"
  "document.getElementById('sta').disabled=true;"
  "document.getElementById('sto').disabled=false;"
"}"
"function stopAudio(){"
  "if(ws){ws.onclose=null;ws.close();ws=null;}"
  "if(ctx){ctx.close();ctx=null;}"
  "npt=0;"
  "document.getElementById('sta').disabled=false;"
  "document.getElementById('sto').disabled=true;"
  "document.getElementById('st').textContent='Stopped';"
"}"
"</SCRIPT>"
  );
}

static const String webRadioPage()
{
  String ip = "";
  String ssid = "";
  String freq = currentMode == FM?
    String(currentFrequency / 100.0) + "MHz "
  : String(currentFrequency + currentBFO / 1000.0) + "kHz ";

  if(WiFi.status()==WL_CONNECTED)
  {
    ip = WiFi.localIP().toString();
    ssid = WiFi.SSID();
  }
  else
  {
    ip = WiFi.softAPIP().toString();
    ssid = String(apSSID);
  }

  return webPage(
"<H1>ATS-Mini Pocket Receiver</H1>"
"<P ALIGN='CENTER'>"
  "<A HREF='/memory'>Memory</A>&nbsp;|&nbsp;<A HREF='/config'>Config</A>&nbsp;|&nbsp;<A HREF='/audio'>Audio</A>"
"</P>"
"<TABLE COLUMNS=2>"
"<TR>"
  "<TD CLASS='LABEL'>IP Address</TD>"
  "<TD><A HREF='http://" + ip + "'>" + ip + "</A> (" + ssid + ")</TD>"
"</TR>"
"<TR>"
  "<TD CLASS='LABEL'>MAC Address</TD>"
  "<TD>" + String(getMACAddress()) + "</TD>"
"</TR>"
"<TR>"
  "<TD CLASS='LABEL'>Firmware</TD>"
  "<TD>" + String(getVersion(true)) + "</TD>"
"</TR>"
"<TR>"
  "<TD CLASS='LABEL'>Band</TD>"
  "<TD>" + String(getCurrentBand()->bandName) + "</TD>"
"</TR>"
"<TR>"
  "<TD CLASS='LABEL'>Frequency</TD>"
  "<TD>" + freq + String(bandModeDesc[currentMode]) + "</TD>"
"</TR>"
"<TR>"
  "<TD CLASS='LABEL'>Signal Strength</TD>"
  "<TD>" + String(rssi) + "dBuV</TD>"
"</TR>"
"<TR>"
  "<TD CLASS='LABEL'>Signal to Noise</TD>"
  "<TD>" + String(snr) + "dB</TD>"
"</TR>"
"<TR>"
  "<TD CLASS='LABEL'>Battery Voltage</TD>"
  "<TD>" + String(batteryMonitor()) + "V</TD>"
"</TR>"
"</TABLE>"
);
}

static const String webMemoryPage()
{
  String items = "";

  for(int j=0 ; j<MEMORY_COUNT ; j++)
  {
    char text[64];
    sprintf(text, "<TR><TD CLASS='LABEL' WIDTH='10%%'>%02d</TD><TD>", j+1);
    items += text;

    if(!memories[j].freq)
      items += "&nbsp;---&nbsp;</TD></TR>";
    else
    {
      String freq = memories[j].mode == FM?
        String(memories[j].freq / 1000000.0) + "MHz "
      : String(memories[j].freq / 1000.0) + "kHz ";
      items += freq + bandModeDesc[memories[j].mode] + "</TD></TR>";
    }
  }

  return webPage(
"<H1>ATS-Mini Pocket Receiver Memory</H1>"
"<P ALIGN='CENTER'>"
  "<A HREF='/'>Status</A>&nbsp;|&nbsp;<A HREF='/config'>Config</A>&nbsp;|&nbsp;<A HREF='/audio'>Audio</A>"
"</P>"
"<TABLE COLUMNS=2>" + items + "</TABLE>"
);
}

const String webConfigPage()
{
  prefs.begin("network", true, STORAGE_PARTITION);
  String ssid1 = prefs.getString("wifissid1", "");
  String pass1 = prefs.getString("wifipass1", "");
  String ssid2 = prefs.getString("wifissid2", "");
  String pass2 = prefs.getString("wifipass2", "");
  String ssid3 = prefs.getString("wifissid3", "");
  String pass3 = prefs.getString("wifipass3", "");
  prefs.end();

  return webPage(
"<H1>ATS-Mini Config</H1>"
"<P ALIGN='CENTER'>"
  "<A HREF='/'>Status</A>"
  "&nbsp;|&nbsp;<A HREF='/memory'>Memory</A>"
  "&nbsp;|&nbsp;<A HREF='/audio'>Audio</A>"
"</P>"
"<FORM ACTION='/setconfig' METHOD='POST'>"
  "<TABLE COLUMNS=2>"
  "<TR><TH COLSPAN=2 CLASS='HEADING'>WiFi Network 1</TH></TR>"
  "<TR>"
    "<TD CLASS='LABEL'>SSID</TD>"
    "<TD>" + webInputField("wifissid1", ssid1) + "</TD>"
  "</TR>"
  "<TR>"
    "<TD CLASS='LABEL'>Password</TD>"
    "<TD>" + webInputField("wifipass1", pass1, true) + "</TD>"
  "</TR>"
  "<TR><TH COLSPAN=2 CLASS='HEADING'>WiFi Network 2</TH></TR>"
  "<TR>"
    "<TD CLASS='LABEL'>SSID</TD>"
    "<TD>" + webInputField("wifissid2", ssid2) + "</TD>"
  "</TR>"
  "<TR>"
    "<TD CLASS='LABEL'>Password</TD>"
    "<TD>" + webInputField("wifipass2", pass2, true) + "</TD>"
  "</TR>"
  "<TR><TH COLSPAN=2 CLASS='HEADING'>WiFi Network 3</TH></TR>"
  "<TR>"
    "<TD CLASS='LABEL'>SSID</TD>"
    "<TD>" + webInputField("wifissid3", ssid3) + "</TD>"
  "</TR>"
  "<TR>"
    "<TD CLASS='LABEL'>Password</TD>"
    "<TD>" + webInputField("wifipass3", pass3, true) + "</TD>"
  "</TR>"
  "<TR><TH COLSPAN=2 CLASS='HEADING'>This Web UI Login Credentials</TH></TR>"
  "<TR>"
    "<TD CLASS='LABEL'>Username</TD>"
    "<TD>" + webInputField("username", loginUsername) + "</TD>"
  "</TR>"
  "<TR>"
    "<TD CLASS='LABEL'>Password</TD>"
    "<TD>" + webInputField("password", loginPassword, true) + "</TD>"
  "</TR>"
  "<TR><TH COLSPAN=2 CLASS='HEADING'>Settings</TH></TR>"
  "<TR>"
    "<TD CLASS='LABEL'>Time Zone</TD>"
    "<TD>"
      "<SELECT NAME='utcoffset'>" + webUtcOffsetSelector() + "</SELECT>"
    "</TD>"
  "</TR>"
  "<TR>"
    "<TD CLASS='LABEL'>Theme</TD>"
    "<TD>"
      "<SELECT NAME='theme'>" + webThemeSelector() + "</SELECT>"
    "</TD>"
  "</TR>"
  "<TR>"
    "<TD CLASS='LABEL'>Reverse Scrolling</TD>"
    "<TD><INPUT TYPE='CHECKBOX' NAME='scroll' VALUE='on'" +
    (scrollDirection<0? " CHECKED ":"") + "></TD>"
  "</TR>"
   "<TR>"
    "<TD CLASS='LABEL'>Zoomed Menu</TD>"
    "<TD><INPUT TYPE='CHECKBOX' NAME='zoom' VALUE='on'" +
    (zoomMenu? " CHECKED ":"") + "></TD>"
  "</TR>"
  "<TR><TH COLSPAN=2 CLASS='HEADING'>"
    "<INPUT TYPE='SUBMIT' VALUE='Save'>"
  "</TH></TR>"
  "</TABLE>"
"</FORM>"
);
}
