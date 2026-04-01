#include <WiFi.h>
#include <Wire.h>
#include <SparkFun_ADS122C04_ADC_Arduino_Library.h> 
#include <SdFat.h>
#include <RTClib.h>
#include <Adafruit_MCP4725.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_BME280.h>
#include <TaskScheduler.h>
#include <Adafruit_Sensor.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ESPmDNS.h> 
#include <Preferences.h>

#define _TASK_SLEEP_ON_IDLE_RUN //light sleep when not running tasks (Not working currently)

//#define DEVICE_NAME "OECTLogger19" // change for each logger
#define PIN PIN_NEOPIXEL
#define NUMPIXELS 1
#define BUTTON_PIN 38 
#define VBATPIN A13

Preferences preferences;

const uint8_t SD_CS_PIN = A5;

SdFs sd;
FsFile file;

char filename[40];
char datetime[] = "YYMMDD-hhmmss.csv"; 
char DEVICE_NAME[20] = "OECTLogger??";

struct DeviceMapping {
  const char* mac;
  uint8_t number;
};

// List of known MAC address -> logger number mappings
DeviceMapping devices[] = {
  {"F4:65:0B:33:FE:04", 5},
  {"F4:65:0B:30:38:48", 6},
  {"F4:65:0B:33:21:DC", 7},
  {"F4:65:0B:33:3F:F4", 8},
  {"F4:65:0B:34:19:78", 9},
  {"F4:65:0B:30:33:F8", 10},
  {"F4:65:0B:34:6E:04", 11},
  {"F4:65:0B:33:82:BC", 12},
  {"14:2B:2F:AF:5F:CC", 13},
  {"F4:65:0B:34:6A:A8", 14},
  {"F4:65:0B:31:C8:9C", 15},
  {"F4:65:0B:33:FC:A0", 16},
  {"F4:65:0B:33:FE:00", 17},
  {"F4:65:0B:31:C8:B0", 18}
};



SFE_ADS122C04 ads;

Adafruit_MCP4725 dac1;
Adafruit_MCP4725 dac2;
Adafruit_BME280 bme;
RTC_DS3231 rtc;
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

class SimpleEMA {
  public:
    SimpleEMA(float alpha) : alpha(alpha), initialized(false), ema(0) {}

    float filter(float newValue) {
      if (!initialized) {
        ema = newValue; // Initialize the EMA with the first value
        initialized = true;
      } else {
        ema = (alpha * newValue) + ((1 - alpha) * ema); // Calculate the EMA
      }
      return ema;
    }

  private:
    float alpha;
    float ema;
    bool initialized;
};

SimpleEMA emaFilter_1(0.4);
SimpleEMA emaFilter_2(0.4);

volatile float vex2 = -0.4;      // Variable formally gate voltage
volatile float vex1 = -0.4;     // Variable formally source voltage
int32_t raw_ADC_data_1 = 0;
float filtered_ADC_data_1 = 0;
int32_t raw_ADC_data_2 = 0;
float filtered_ADC_data_2 = 0;
float voltage_1 = 0.0;
float current_1 = 0.0;
float voltage_2 = 0.0;
float current_2 = 0.0;
volatile float bVolts = 0.0;
volatile uint32_t statusColor = 0;
volatile uint32_t pastStatusColor = 0;
float temp, rh, press, vex1_s,vex2_s;



int ind;
bool userUpdate = false;

IPAddress apIP(192, 168, 4, 1);
IPAddress netMsk(255, 255, 255, 0);
AsyncWebServer server(80);
const int MAX_DAC_VALUE = 4095; 
const int MIN_DAC_VALUE = 1;
const int ADC_MAX_VALUE = 8388607; // (2^24)/2 - 1
const float V_REF = 5.0;           // Total voltage range
const float V_OFFSET = 2.5;        // Offset voltage
const int INCREMENT_STEP = 1; 
const float TOLERANCE  = 0.002; 

// --- New Global Variables for WiFi/WebServer Control ---
bool wifiActive = false;
unsigned long wifiActivationTime = 0;
const unsigned long WIFI_ACTIVE_DURATION = 10 * 60 * 1000; // 10 minutes in milliseconds
volatile bool buttonPressed = false; // Volatile for ISR

// --- Function Prototypes ---
void blinkNeopixel(int n,int interval, uint32_t color);
void blinkNeopixelForever(int interval, uint32_t color);
bool setupDACs();
void onValuesUpdated();
float mapRange(float value, float inMin, float inMax, float outMin, float outMax);
bool setupADC();
bool setupBME();
void setOutputVoltage(Adafruit_MCP4725 &dac, float targetVoltage);
void readADCCallback();
void sendSensorDataCallback();
void readTempRHCallBack();
void readUserInputCallBack();
void setDeviceName();
void serveFile(AsyncWebServerRequest *request, const char* path);
String getRtcTimeString();
void blinkStatusLEDCallback();
void readBatteryCallback();
void checkButtonTaskCallback();

// --- New Function Prototypes for WiFi/WebServer Control ---
void activateWiFiAndWebServer();
void deactivateWiFiAndWebServer();
void IRAM_ATTR buttonISR();
void deactivateWiFiAndWebServerCallback(); // Callback for the scheduler task

// --- Task Scheduler Setup ---
Task readADCTask(10000, TASK_FOREVER, &readADCCallback); 
Task sendSensorDataTask(60000, TASK_FOREVER, &sendSensorDataCallback);
//Task readTempRHTask(10000, TASK_FOREVER, &readTempRHCallBack);
Task readUserInputTask(10000, TASK_FOREVER, &readUserInputCallBack);
Task wifiTimerTask(WIFI_ACTIVE_DURATION, TASK_ONCE, &deactivateWiFiAndWebServerCallback);
//Task wifiDeactivationTask(WIFI_ACTIVE_DURATION, TASK_ONCE, &deactivateWiFiAndWebServerCallback); // New task for Wi-Fi timeout
Task readBatteryTask(600000, TASK_FOREVER, &readBatteryCallback);
Task blinkLEDTask(1000, TASK_FOREVER, &blinkStatusLEDCallback);
Task checkButtonTask(1000, TASK_FOREVER, &checkButtonTaskCallback);

Scheduler runner;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(5000);

  WiFi.mode(WIFI_MODE_STA);
  WiFi.disconnect();  // Ensure no connection
  delay(2000);

  setDeviceName();

  

  pixels.begin();
  blinkNeopixel(1,500,pixels.Color(50, 0, 50)); //purple 
  blinkNeopixel(1,500,pixels.Color(50, 20, 0)); //organge
  blinkNeopixel(1,500,pixels.Color(50, 0, 50)); //purple
  blinkNeopixel(1,500,pixels.Color(50, 20, 0)); //organge 


  //delay to allow serial connection to intialise
  delay(5000); 

  // --- Initialize Button for WiFi Activation ---
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);
  Serial.println("Button interrupt attached to GPIO 38.");
  Serial.println(WiFi.macAddress());

  WiFi.mode(WIFI_MODE_NULL);

  

  
 // --- Web server routes setup (always define them, but only start server on button press) ---
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    String html = "<html><head><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<title>ESP32 Web Interface</title></head><body>";
    html += "<h2>" + String(DEVICE_NAME) +"</h2>";
    html +=  "<h3>Input Voltages</h3>";
    html += "<label for='vex1'>Ch1 vEX (Excitation Voltage Ch1, up to +-2.5V):</label>";
    html += "<input type='number' id='vex1' step='0.001' min='-2.5' max='2.5' value='" + String(vex1, 3) + "'><br>";
    html += "<label for='vex2'>Ch2 vEX (Excitation Voltage Ch2, up to +-2.5V):</label>";
    html += "<input type='number' id='vex2' step='0.001' min='-2.5' max='2.5' value='" + String(vex2, 3) + "'><br>";
    html += "<button onclick='submitValues()'>Submit</button>";
    html += "<h3>ADC Data</h3>";
    html += "<p>Ch1 vEX: <span id='vex1_actual'>" + String(vex1_s) + "</span> V</p>";
    html += "<p>Ch2 vEX: <span id='vex2_actual'>" + String(vex2_s) + "</span> V</p>";
    html += "<p>Sensor Raw ADC Data Ch1: <span id='adc_1'>" + String(raw_ADC_data_1) + "</span></p>";
    html += "<p>Sensor Output Voltage Ch1: <span id='voltage_1'>" + String(voltage_1) + "</span> V</p>";
    html += "<p>Sensor Current Ch1: <span id='current_1'>" + String(current_1) + "</span> mA</p>";
    html += "<p>Sensor Raw ADC Data Ch2: <span id='adc_2'>" + String(raw_ADC_data_2) + "</span></p>";
    html += "<p>Sensor Output Voltage Ch2: <span id='voltage_2'>" + String(voltage_2) + "</span> V</p>";
    html += "<p>Sensor Current Ch2: <span id='current_2'>" + String(current_2) + "</span> mA</p>";
    html += "<p>Battery Voltage: <span id='bVolts'>" + String(bVolts) + "</span> V</p>";
    html += "<hr>"; // Separator
    html += "<h3>Real-Time Clock (RTC) Status</h3>";
    html += "<p>Current RTC Time: <span id='rtc_time_display'>" + getRtcTimeString() + "</span></p>";
    html += "<p>Your Device's Time: <span id='client_time_display'>Waiting...</span></p>";
    html += "<button onclick='updateRTC()'>Update RTC to Device Time</button>";
    
    // Add this new button for file download
    html += "<hr>"; // Separator
    html += "<h3>Data Logging</h3>";
    html += "<a href='/list_files'><button>Download Log Files</button></a>";

    html += "<script>";
    html += "function submitValues() {";
    html += "   var vex2 = document.getElementById('vex2').value;";
    html += "   var vex1 = document.getElementById('vex1').value;";
    html += "   fetch('/setValues?vex2=' + vex2 + '&vex1=' + vex1).then(response => {";
    html += "     if (response.ok) {";
    html += "       fetch('/userUpdate');"; // Fetch to set the userUpdate flag
    html += "     }";
    html += "   });";
    html += "}";

    html += "function updateRTC() {";
    html += "  var now = new Date();";
    html += "  var epochSeconds = Math.floor(now.getTime() / 1000);"; // convert to seconds
    html += "  fetch('/setrtc?epoch=' + epochSeconds).then(response => {";
    html += "    if (response.ok) {";
    html += "      alert('RTC time update requested successfully!');";
    html += "      fetch('/userUpdate');"; // Signal the user update task 
    html += "    } else {";
    html += "      alert('Failed to update RTC time.');";
    html += "    }";
    html += "  });";
    html += "}";

    // html += "setInterval(function() {";
    // html += "   fetch('/updateData').then(response => response.json()).then(data => {";
    // html += "     document.getElementById('adc_1').innerText = data.adc_1;";
    // html += "     document.getElementById('voltage_1').innerText = data.voltage_1;";
    // html += "     document.getElementById('current_1').innerText = data.current_1;";
    // html += "     document.getElementById('adc_2').innerText = data.adc_2;";
    // html += "     document.getElementById('voltage_2').innerText = data.voltage_2;";
    // html += "     document.getElementById('current_2').innerText = data.current_2;";
    // html += "     document.getElementById('bVolts').innerText = data.bVolts;";
    // html += "     document.getElementById('vex1_actual').innerText = data.vex1_s;";
    // html += "     document.getElementById('vex2_actual').innerText = data.vex2_s;"; 
    // html += "   });";
    // html += "}, 3000);"; // Update more frequently for better responsiveness
    // html += "</script>";

  html += "setInterval(function() {";
  
  html += "   fetch('/updateData').then(response => response.json()).then(data => {";
  html += "     document.getElementById('adc_1').innerText = data.adc_1;";
  html += "     document.getElementById('voltage_1').innerText = data.voltage_1;";
  html += "     document.getElementById('current_1').innerText = data.current_1;";
  html += "     document.getElementById('adc_2').innerText = data.adc_2;";
  html += "     document.getElementById('voltage_2').innerText = data.voltage_2;";
  html += "     document.getElementById('current_2').innerText = data.current_2;";
  html += "     document.getElementById('bVolts').innerText = data.bVolts;";
  html += "     document.getElementById('vex1_actual').innerText = data.vex1_s;";
  html += "     document.getElementById('vex2_actual').innerText = data.vex2_s;"; 
  html += "   });";
  
  // --- 2. Update RTC and Client Time (New) ---
  // Get and display client's local time
  html += "  var clientNow = new Date();";
  html += "  var clientTimeStr = clientNow.getFullYear() + '-' + ('0' + (clientNow.getMonth() + 1)).slice(-2) + '-' + ('0' + clientNow.getDate()).slice(-2) + ' ' + ('0' + clientNow.getHours()).slice(-2) + ':' + ('0' + clientNow.getMinutes()).slice(-2) + ':' + ('0' + clientNow.getSeconds()).slice(-2);";
  html += "  document.getElementById('client_time_display').innerText = clientTimeStr;";

  // Fetch and display RTC time from the ESP32
  html += "  fetch('/rtctime').then(response => response.text()).then(data => {";
  html += "    document.getElementById('rtc_time_display').innerText = data;";
  html += "  });";

  html += "}, 3000);"; 
  html += "</script>";
    html += "</body></html>";
    request->send(200, "text/html", html);
  });


  server.on("/list_files", HTTP_GET, [](AsyncWebServerRequest *request){
    String html = "<html><head><meta name='viewport' content='width=device-width, initial-scale=1.0'><title>CSV Files</title></head><body>";
    html += "<h2>Available CSV Files</h2><ul>";

    FsFile root = sd.open("/");
    if (root) {
      FsFile file = root.openNextFile();
      char nameBuffer[64];
      while (file) {
        size_t len = file.getName(nameBuffer, sizeof(nameBuffer));
        String name;
        if (len > 0) {
            name = String(nameBuffer); // 3. Convert the buffer content to String
        }
        
        if (!file.isDir() && name.endsWith(".csv")) { 
          html += "<li><a href='/download?name=" + name + "'>" + name + "</a></li>";
        }
        file = root.openNextFile();
      }
      root.close();
    }
    

    html += "</ul></body></html>";
    request->send(200, "text/html", html);
  });

  server.on("/download", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("name")) {
      String fileName = "/" + request->getParam("name")->value();
      if (sd.exists(fileName.c_str())) {
        serveFile(request, fileName.c_str());
      } else {
        request->send(404, "text/plain", "File not found");
      }
    } else {
      request->send(400, "text/plain", "Bad Request: Missing 'name' parameter");
    }
  });

  server.on("/setValues", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("vex2") && request->hasParam("vex1")) {
      vex2 = request->getParam("vex2")->value().toFloat();
      vex1 = request->getParam("vex1")->value().toFloat();
    }
    request->send(200, "text/plain", "Values set");
  });

  server.on("/userUpdate", HTTP_GET, [](AsyncWebServerRequest *request) {
    userUpdate = true; // Set the flag to indicate a user update
    request->send(200, "text/plain", "User update received");
  });

  // Corrected server.on("/updateData", HTTP_GET, ...)
server.on("/updateData", HTTP_GET, [](AsyncWebServerRequest *request){
  String jsonResponse = "{\"adc_1\":" + String(raw_ADC_data_1) +
                        ",\"voltage_1\":" + String(voltage_1, 6) +
                        ",\"current_1\":" + String(current_1, 6) +
                        ",\"adc_2\":" + String(raw_ADC_data_2) +
                        ",\"voltage_2\":" + String(voltage_2, 6) +
                        ",\"current_2\":" + String(current_2, 6) +
                        ",\"bVolts\":" + String(bVolts, 2) + 
                        ",\"vex1_s\":" + String(vex1_s, 6) +  
                        ",\"vex2_s\":" + String(vex2_s, 6) +
                        "}";  
  request->send(200, "application/json", jsonResponse);
});

server.on("/setrtc", HTTP_GET, [](AsyncWebServerRequest *request){
  if (request->hasParam("epoch")) {
    // Get the epoch timestamp sent from the client
    long epoch = request->getParam("epoch")->value().toInt();
    
    // Create a DateTime object from the epoch time
    DateTime newTime(epoch);
    
    // Adjust the DS3231 RTC
    rtc.adjust(newTime);

    Serial.print("RTC Time updated to: ");
    Serial.println(getRtcTimeString());
    
    request->send(200, "text/plain", "RTC Time set successfully!");
  } else {
    request->send(400, "text/plain", "Bad Request: Missing 'epoch' parameter");
  }
});

server.on("/rtctime", HTTP_GET, [](AsyncWebServerRequest *request){
  String rtcTime = getRtcTimeString();
  request->send(200, "text/plain", rtcTime);
});

  

  pixels.setPixelColor(0, pixels.Color(0, 0, 50));//blue
  pixels.show();
  delay(5000);
  Serial.println("Server setup complete (not yet started)"); 

  preferences.begin("oect-config", true);

  vex1 = preferences.getFloat("vex1-target", vex1); 
  vex2 = preferences.getFloat("vex2-target", vex2);

  preferences.end();

  Serial.print("Loaded VEX1 target from memory: "); Serial.println(vex1);
  Serial.print("Loaded VEX2 target from memory: "); Serial.println(vex2);

  Serial.println("setup_adc");
  setupDACs();
  pixels.setPixelColor(0, pixels.Color(50, 0, 0));//red
  pixels.show();
  
  setupADC();
  
  pixels.setPixelColor(0, pixels.Color(50, 10, 0));//orange
  pixels.show();
  setOutputVoltage(dac1, 3, vex1);
  setOutputVoltage(dac2, 2, vex2);

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, using compile time as fallback");
    pixels.setPixelColor(0, pixels.Color(50, 0, 0)); //red
    pixels.show();
    // use complie time
    rtc.adjust(DateTime(__DATE__, __TIME__));
    DateTime sync = rtc.now();
    rtc.adjust(DateTime(sync.unixtime() + 25)); //offeset the time it takes to upload
  }

  DateTime start = rtc.now();
  start.toString(datetime);

  snprintf(filename, sizeof(filename), "%s-%s", DEVICE_NAME, datetime);

  setupBME();

  if (!sd.begin(SD_CS_PIN, SPI_HALF_SPEED)) {
    Serial.println("Card failed, or not present");
    blinkNeopixelForever(100, pixels.Color(50, 50, 0)); //yellow
  }
  Serial.println("card initialized.");

  

  readBatteryCallback(); // run once at the start. 

  runner.init();
  runner.addTask(readADCTask);
  runner.addTask(sendSensorDataTask);
  runner.addTask(readUserInputTask);
  runner.addTask(readBatteryTask);
  runner.addTask(blinkLEDTask);
  runner.addTask(checkButtonTask);
  
  runner.addTask(wifiTimerTask);  // Add the new deactivation task

  readADCTask.enable();
  sendSensorDataTask.enable();
  readUserInputTask.enable();
  readBatteryTask.enable();
  blinkLEDTask.enable();
  checkButtonTask.enable();

   wifiTimerTask.disable();

  // wifiDeactivationTask is initially disabled and enabled on button press

  Serial.println("begin sampling");

  pixels.setPixelColor(0, pixels.Color(0, 50, 0));//green
  pixels.show();
}

void loop() {
  runner.execute();
}

// --- New Functions for WiFi and Web Server Control ---
void activateWiFiAndWebServer() {
  if (!wifiActive) {
    Serial.println("Activating Wi-Fi in SoftAP mode...");
    WiFi.softAPConfig(apIP, apIP, netMsk);
    WiFi.softAP(DEVICE_NAME);
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());

    if (!MDNS.begin("logger")) {
      Serial.println("Error starting mDNS");
    } else {
      Serial.println("mDNS responder started");
      MDNS.addService("http", "tcp", 80);
      Serial.print("Access your server at: http://logger.local\n");
    }
    server.begin();
    wifiActive = true;
    wifiActivationTime = millis();
    pastStatusColor = statusColor;
    statusColor = pixels.Color(0, 50, 50); // Cyan for WiFi active
    
    Serial.println("Web server started.");
  }
}

void deactivateWiFiAndWebServer() {
  if (wifiActive) {
    Serial.println("Deactivating Wi-Fi and Web Server...");
    server.end();
    WiFi.softAPdisconnect(true);
    MDNS.end();
    wifiActive = false;
    statusColor = pastStatusColor; // back to what it was for WiFi inactive (back to normal)
    
    Serial.println("Wi-Fi and Web Server deactivated.");
  }
}

void deactivateWiFiAndWebServerCallback() {
  deactivateWiFiAndWebServer();
  wifiTimerTask.disable(); // Disable the task after it runs
}

// --- ISR for the button ---
void IRAM_ATTR buttonISR() {
  buttonPressed = true;
}

// This function performs the ADC conversion and returns the raw ADC value.
long getADCReading() {
  ads.start(); 
  unsigned long start_time = millis(); 
  boolean drdy = false; 

  while((drdy == false) && (millis() < (start_time + ADS122C04_CONVERSION_TIMEOUT))) {
    delay(5); 
    drdy = ads.checkDataReady(); 
  }

  if (drdy == false) {
    Serial.println(F("checkDataReady timed out"));
    return 0; // Return 0 or an error value on timeout
  }
  return ads.readADC();
}

// This is the main callback function that takes two measurements,
// converts them to voltage and current, and stores them in global variables.
void readADCCallback() {
  // First Measurement (AIN0)
  ads.setInputMultiplexer(ADS122C04_MUX_AIN0_AVSS);
  delay(10); // Small delay to allow MUX to settle
  raw_ADC_data_1 = getADCReading();
  filtered_ADC_data_1 = emaFilter_1.filter(raw_ADC_data_1);
  voltage_1 = (filtered_ADC_data_1 / ADC_MAX_VALUE) * V_REF - V_OFFSET;
  current_1 = ((-(voltage_1))/1000) * 1000; // in mA for 1k resistor

  // Second Measurement (AIN1)
  ads.setInputMultiplexer(ADS122C04_MUX_AIN1_AVSS);
  delay(10); // Small delay to allow MUX to settle
  raw_ADC_data_2 = getADCReading();
  filtered_ADC_data_2 = emaFilter_2.filter(raw_ADC_data_2);
  voltage_2 = (filtered_ADC_data_2 / ADC_MAX_VALUE) * V_REF - V_OFFSET;
  current_2 = ((-(voltage_2))/1000) * 1000; // in mA for 1k resistor
}

void readBatteryCallback(){
  uint32_t raw_value = analogRead(VBATPIN);
  bVolts = raw_value * 2.0 * 3.3 / 4095.0; 

  if (bVolts <= 3.4) {
    statusColor = pixels.Color(50, 10, 0); // Set color to orangeish
  } else if (bVolts <= 3.7) {
    statusColor = pixels.Color(50, 50, 0); // Set color to yellow
  } else {
    statusColor = pixels.Color(0, 50, 0); // Set color to green
  }
}


void sendSensorDataCallback(){

  int raw_vex1, raw_vex2;
  
  char sample_str[400];

  temp = bme.readTemperature();
  rh = bme.readHumidity();
  press = (bme.readPressure() / 100);

  ads.setInputMultiplexer(ADS122C04_MUX_AIN3_AVSS);
  delay(10); // Small delay to allow MUX to settle
   
  raw_vex1 = getADCReading();
  vex1_s = ((float)raw_vex1 / ADC_MAX_VALUE) * V_REF - 2.5; 

  ads.setInputMultiplexer(ADS122C04_MUX_AIN2_AVSS);
  delay(10); // Small delay to allow MUX to settle
  raw_vex2 = getADCReading();
  vex2_s = ((float)raw_vex2 / ADC_MAX_VALUE) * V_REF - 2.5; 

  DateTime now = rtc.now();

  Serial.print("RAW_ADC_OUT_1:");
  Serial.print(raw_ADC_data_1);
  Serial.print("  filtered_ADC_OUT_1:");
  Serial.print(filtered_ADC_data_1);
  Serial.print("  RAW_ADC_OUT_2:");
  Serial.print(raw_ADC_data_2);
  Serial.print("  filtered_ADC_OUT_2:");
  Serial.print(filtered_ADC_data_2);
  Serial.print("  EX_Volts_2:");
  Serial.print(vex2_s, 3);
  Serial.print("  EX_Volts_1:");
  Serial.print(vex1_s, 3);
  Serial.print("  TIA_Voltage_1:");
  Serial.print(voltage_1, 8);
  Serial.print("  Current_1:");
  Serial.print(current_1, 8);
  Serial.print("  TIA_Voltage_2:");
  Serial.print(voltage_2, 8);
  Serial.print("  Current_2:");
  Serial.print(current_2, 8);
  Serial.print("  Temp:");
  Serial.print(temp, 4);
  Serial.print("  RH:");
  Serial.print(rh, 4);
  Serial.print("  Pressure:");
  Serial.println(press, 4);

    sprintf(sample_str, "%d,%.3f,%.8f,%.8f,%d,%.3f,%.8f,%.8f,%.3f,%.3f,%.2f,%.2f,%.2f\n", 
            raw_ADC_data_1, filtered_ADC_data_1, voltage_1, current_1,
            raw_ADC_data_2, filtered_ADC_data_2, voltage_2, current_2,
            vex2_s, vex1_s, temp, rh, press);

    FsFile dataFile = sd.open(filename, O_WRITE | O_APPEND | O_CREAT);
    
    if (dataFile) {

        dataFile.print(now.timestamp());
        dataFile.print(",");
        dataFile.print(sample_str);
        dataFile.flush(); 
        
        dataFile.close();

    } else {
        Serial.print("failed to write data to file: ");
        Serial.println(filename); 
        statusColor = pixels.Color(50, 0, 0); 
        delay(200);
    }
}

void readUserInputCallBack(){
  if (userUpdate){
    vex1_s = setOutputVoltage(dac1, 3, vex1);
    vex2_s = setOutputVoltage(dac2, 2, vex2);

    preferences.begin("oect-config", false); // Use 'false' for read/write
    preferences.putFloat("vex1-target", vex1);
    preferences.putFloat("vex2-target", vex2);
    preferences.end();
    Serial.println("New VEX targets saved to NVS.");
    userUpdate = false;
  }
}

void checkButtonTaskCallback() {
    // Check if the button was pressed AND the WiFi is not already active
    if (buttonPressed && !wifiActive) {
        Serial.println("Button pressed! Activating Wi-Fi and Web Server.");
        
        // This is the key: reset the flag right away to prevent multiple activations
        buttonPressed = false; 

        activateWiFiAndWebServer();
        
        // Enable and start the deactivation timer
        wifiTimerTask.enableDelayed(WIFI_ACTIVE_DURATION);
    }
}

bool setupDACs() {
  uint16_t dac1Val = MAX_DAC_VALUE / 2;
  uint16_t dac2Val = MAX_DAC_VALUE / 2;

  if (dac1.begin(0x60, &Wire)) { 
    Serial.println("MCP4725 initialization successful!");
  } else {
    Serial.println("Failed to initialize dac1. Please check your connections.");
    blinkNeopixelForever(100,pixels.Color(0, 0, 50)); 
  }

  if (!dac1.setVoltage(dac1Val, false)) {
      Serial.println("Failed to update DAC.");
    }

  if (dac2.begin(0x61, &Wire)) { 
    Serial.println("AD5693 initialization successful!");
  } else {
    Serial.println("Failed to initialize dac2. Please check your connections.");
    blinkNeopixelForever(100,pixels.Color(0, 0, 50));
  }
  
  if (!dac2.setVoltage(dac2Val, false)) {
      Serial.println("Failed to update DAC.");
    }
  return true;  
}

float mapRange(float value, float inMin, float inMax, float outMin, float outMax) {
    return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

bool setupADC() {
  if (ads.begin(0x40) == false) {
    Serial.println(F("ads122 not detected at default I2C address. Please check wiring. Freezing."));
    blinkNeopixelForever(100, pixels.Color(50, 0, 0)); 
  }

  Serial.println("setting up adc");
  ads.setInputMultiplexer(ADS122C04_MUX_AIN0_AVSS); 
  ads.setGain(ADS122C04_GAIN_1); 
  ads.enablePGA(ADS122C04_PGA_DISABLED); 
  ads.setDataRate(ADS122C04_DATA_RATE_20SPS); 
  ads.setOperatingMode(ADS122C04_OP_MODE_NORMAL); 
  ads.setConversionMode(ADS122C04_CONVERSION_MODE_SINGLE_SHOT); 
  ads.setVoltageReference(ADS122C04_VREF_AVDD); 
  ads.enableInternalTempSensor(ADS122C04_TEMP_SENSOR_OFF); 
  ads.setDataCounter(ADS122C04_DCNT_DISABLE); 
  ads.setDataIntegrityCheck(ADS122C04_CRC_DISABLED); 
  ads.setBurnOutCurrent(ADS122C04_BURN_OUT_CURRENT_OFF); 
  ads.setIDACcurrent(ADS122C04_IDAC_CURRENT_OFF); 
  ads.setIDAC1mux(ADS122C04_IDAC1_DISABLED); 
  ads.setIDAC2mux(ADS122C04_IDAC2_DISABLED); 
  ads.enableDebugging(Serial); 
  ads.printADS122C04config();
  ads.disableDebugging();
  return true;
}   

bool setupBME() {
  unsigned status;
  status = bme.begin(0x77);  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
    Serial.print("    ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("  ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("    ID of 0x60 represents a BME 280.\n");
    Serial.print("    ID of 0x61 represents a BME 680.\n");
    while (1) {
      pixels.setPixelColor(0, pixels.Color(50, 10, 0));//orange
      pixels.show();
      delay(100);
      pixels.setPixelColor(0, pixels.Color(0, 0, 00));//off
      pixels.show();
      delay(100);
    }
  }
  return true;
}

float setOutputVoltage(Adafruit_MCP4725 &dac, int adcFbCh, float targetVoltage) {
  const int MAX_ATTEMPTS = 50;               // Prevent infinite loop
  const float KP = 500.0;                    // Proportional gain for DAC step size
  const float MIN_STEP = 1;                  // Smallest DAC change allowed
  const float MAX_STEP = 200;                // Largest DAC change allowed

  uint16_t dacValue = 1400;
  float prevError = 0;

  // Set ADC input channel
  if (adcFbCh == 2){
    ads.setInputMultiplexer(ADS122C04_MUX_AIN2_AVSS);
  } else if( adcFbCh == 3){
    ads.setInputMultiplexer(ADS122C04_MUX_AIN3_AVSS);
  }
  delay(10); // settle time after mux change

  for (int attempt = 0; attempt < MAX_ATTEMPTS; ++attempt) {
    dac.setVoltage(dacValue, false);
    delay(10); // Allow output to settle

    ads.start();
    unsigned long start_time = millis();
    bool drdy = false;

    while (!drdy && (millis() - start_time < ADS122C04_CONVERSION_TIMEOUT)) {
      delay(5);
      drdy = ads.checkDataReady();
    }

    if (!drdy) {
      Serial.println(F("checkDataReady timed out"));
      return 0.0;
    }

    uint32_t adcValue = ads.readADC();
    float adcVoltage = ((float)adcValue / ADC_MAX_VALUE) * V_REF - 2.5;
    float error = targetVoltage - adcVoltage;

    Serial.print("DAC: ");
    Serial.print(dacValue);
    Serial.print(" | ADC Voltage: ");
    Serial.print(adcVoltage, 6);
    Serial.print(" | Error: ");
    Serial.println(error, 6);

    if (abs(error) < TOLERANCE) {
      Serial.print("Target voltage reached: ");
      Serial.println(adcVoltage, 6);
      return adcVoltage;
    }

    // Calculate proportional step size (with limits)
    int step = constrain((int)(KP * error), -MAX_STEP, MAX_STEP);
    
    // Avoid getting stuck with 0 step
    if (step == 0) {
      step = (error > 0) ? MIN_STEP : -MIN_STEP;
    }

    // Update DAC value and clamp to range
    int32_t newDAC = (int32_t)dacValue + step;
    newDAC = constrain(newDAC, MIN_DAC_VALUE, MAX_DAC_VALUE);

    // If no change in DAC is possible, exit
    if (newDAC == dacValue) {
      Serial.println("DAC value cannot be changed further.");
      return adcVoltage;
    }

    dacValue = newDAC;
    prevError = error;
  }

  Serial.println("Max attempts reached without hitting target.");
}


void setDeviceName() {
   String macStr = WiFi.macAddress(); // e.g., "F4:65:0B:33:FE:04"

   for (const auto& device : devices) {
     if (macStr.equalsIgnoreCase(device.mac)) {
        snprintf(DEVICE_NAME, sizeof(DEVICE_NAME), "OECTLogger%d", device.number);
        return;
      }
   }
 

   const char* mac_c_str = macStr.c_str();

   if (macStr.length() >= 2) {

      const char* last_two_digits = mac_c_str + macStr.length() - 2;


 
     snprintf(DEVICE_NAME, sizeof(DEVICE_NAME), "OECTLogger%s", last_two_digits);
     Serial.print("Unmatched MAC. Using last two digits: ");
     Serial.println(DEVICE_NAME);

   } else {

     snprintf(DEVICE_NAME, sizeof(DEVICE_NAME), "OECTLoggerERR");
   }
}

void serveFile(AsyncWebServerRequest *request, const char* path) {
  FsFile file = sd.open(path);
  if (!file) {
    request->send(404, "text/plain", "File not found");
    return;
  }

  AsyncWebServerResponse *response = request->beginResponse(
    "text/csv", file.fileSize(),
    [file](uint8_t *buffer, size_t maxLen, size_t index) mutable -> size_t {
      file.seek(index);
      return file.read(buffer, maxLen);
    }
  );
  response->addHeader("Content-Disposition", String("attachment; filename=") + String(path).substring(1));
  request->send(response);
}

String getRtcTimeString() {
  DateTime now = rtc.now();
  char buffer[30];
  // Formats time as: YYYY-MM-DD HH:MM:SS
  sprintf(buffer, "%04d-%02d-%02d %02d:%02d:%02d", 
          now.year(), now.month(), now.day(), 
          now.hour(), now.minute(), now.second());
  return String(buffer);
}



void blinkStatusLEDCallback() {
  static bool ledOn = false; // Static variable to remember the state between calls

  // Toggle the state of the LED
  ledOn = !ledOn;

  if (ledOn) {
    // If the desired color is not black, turn the LED on
    if (statusColor != 0) {
      pixels.setPixelColor(0, statusColor);
    }
  } else {
    // Always turn the LED off on the next toggle
    pixels.setPixelColor(0, pixels.Color(0, 0, 0)); 
  }
  pixels.show();
}

void blinkNeopixel(int n,int interval, uint32_t color) {
  for (int i = 0; i < n; i++) {
    pixels.setPixelColor(0, color);
    pixels.show(); 
    delay(interval);
    pixels.setPixelColor(0, pixels.Color(0, 0, 0)); 
    pixels.show();
    delay(interval);
  }   
}

void blinkNeopixelForever(int interval, uint32_t color) {
  while(1){
    pixels.setPixelColor(0, color);
    pixels.show(); 
    delay(interval);
    pixels.setPixelColor(0, pixels.Color(0, 0, 0)); 
    pixels.show();
    delay(interval);
  }
}