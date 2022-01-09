#include "connectionDetails.h"

#include <WiFi.h>
#include <PubSubClient.h>
#include <driver/adc.h>
#include "esp_adc_cal.h"

#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include <WiFiClient.h>
#include <WebServer.h>
#include <merlinUpdateWebServer.h>

#include <NTPClient.h>
#include <TimeLib.h>

#include "EmonLib.h"

#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chip
#include <SPI.h>

#include <Button2.h>

#define MCMDVERSION 0.4
#define DEBUG 0

#ifdef DEBUG
#define DEBUG_PRINT(x) Serial.print(x) 
#define DEBUG_PRINTDEC(x, DEC) Serial.print(x, DEC)
#define DEBUG_PRINTLN(x) Serial.println(x)
#define DEBUG_PRINTLNDEC(x, DEC) Serial.println(x, DEC)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTDEC(x, DEC)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINTLNDEC(x, DEC)
#endif

#define LED_BUILTIN 21
#define TOTALPHASES 1

// The voltage in our apartment. Usually this is 230V for Europe, 110V for US.
// Ours is higher because the building has its own high voltage cabin.
#define HOME_VOLTAGE 245.6

// Force EmonLib to use 10bit ADC resolution
#define ADC_BITS 10
#define ADC_COUNTS (1 << ADC_BITS)

// Create instances
EnergyMonitor _eMonitorInstance[3];

WiFiClient _wifiClient;
PubSubClient _mqttClient(_wifiClient);

String _lastMQTTMessage = "";
String _lastPublishedMQTTMessage = "";

// Array to store 30 readings (and then transmit in one-go to MQTT)
//unsigned long  _wattMeasurementsHistory[3][30];
double _liveAmpMeasurement[3] = {0, 0, 0};
double _liveWattMeasurement[3] = {0, 0, 0};

int _minuteMeasureIndex = 0;
int _hourMeasureIndex = 0;
int _dayMeasureIndex = 1;
unsigned long _lastMeasurement = 0;
unsigned long _timeFinishedSetup = 0;

// You can specify the time server pool and the offset (in seconds, can be
// changed later with setTimeOffset() ). Additionaly you can specify the
// update interval (in milliseconds, can be changed using setUpdateInterval() ).
WiFiUDP _ntpUDP;
//const byte offset = 2;
#define OFFSET 2
NTPClient _timeClient(_ntpUDP, "pool.ntp.org", OFFSET, 60000);
unsigned long _epochTime;
String _ntpDate = "";

double _ampsCurrent = 0;
double _watt_minute = 0;
double _watt_minute_total = 0;
double _currentUsageWatt = 0;
double _currentUsageWattHour = 0;
double _liveUsageWattMin = 0;
double _kWhDailyTotalAccumulating = 0;

long _rssi = 0;					 // Measure Signal Strength (RSSI) of Wi-Fi connection
long _rssiQualityPercentage = 0; // Measure Signal Strength (RSSI) of Wi-Fi connection

#define TFT_BL 4 // Display backlight control pin
#define ADC_EN 14
#define ADC_PIN 34
#define BUTTON_1 35
#define BUTTON_2 0

Button2 btn1(BUTTON_1);
Button2 btn2(BUTTON_2);

char buff[512];
int vref = 1100;
int btnCick = false;

byte _frameIndex = 0;

unsigned long _runCurrent;
unsigned long _runClearPreviousValues;
unsigned long _runClearAverageValues;
unsigned long _runWiFiCheck;
unsigned long _runScreenUpdate;
unsigned long _runScreenSaver;
unsigned long _runDeviceInfo = 0;
unsigned long _runFooter = 0;

bool _initFinished = false;
#define CLEAR_AVERAGE_INTERVAL_MILLISECS 300000 //reset every 5 minutes of non-activity (i.e.: try group storm lightning counts together);
#define WIFI_TICKLED_INTERVAL_MILLISECS 30000	// every 30 seconds
#define SCREEN_UPDATE_INTERVAL_MILLISECS 500	// every 15 seconds
#define SCREEN_SAVER_INTERVAL_MILLISECS 15000	// every 5 seconds

#ifndef TFT_DISPOFF
#define TFT_DISPOFF 0x28
#endif

#ifndef TFT_SLPIN
#define TFT_SLPIN 0x10
#endif

#ifndef TFT_BL
#define TFT_BL 4
#endif

bool _ledBacklight = true;

// The GPIO pin were the CT sensor is connected to (should be an ADC input)

const adc1_channel_t ADC_CHANNELS[3] = {ADC1_GPIO39_CHANNEL, ADC1_GPIO38_CHANNEL, ADC1_GPIO37_CHANNEL};
const byte ADC_INPUT[3] = {39, 38, 37};
double ADC_calib[3] = {30,90.90,90.90}; // (2000 turns / 22 Ohm burden) = 90.90
											 // (1800 turns / 37.7 Ohm burden) = 47.745
											 //30A ct clamp seems best at calibration value of 30


TFT_eSPI _tft = TFT_eSPI(135, 240); // Invoke library, pins defined in User_Setup.h

String _mqttPostFix = "";
float _batteryVoltage = 0;

//*****************************************************************************************
// Setup variables
#define numberOfSamples 4000
#define ICAL 1.08

// CT: Voltage depends on current, burden resistor, and turns
#define CT_BURDEN_RESISTOR 21.9 //21.9
#define CT_TURNS 2000 //100A=0.05A

// Initial gueses for ratios, modified by VCAL / ICAL tweaks
double I_RATIO = (long double)CT_TURNS / CT_BURDEN_RESISTOR * 3.319 / 4096 * ICAL;
// Filter variables 1
double lastFilteredI[3], filteredI[3];
double sqI[3], sumI[3];
// Sample variables
int lastSampleI[3], sampleI[3];
double Irms1[3];
unsigned long timer[3];
//*****************************************************************************************

void tickLED()
{
	//toggle state
	int state = digitalRead(LED_BUILTIN); // get the current state of GPIO1 pin
	if (WiFi.status() == WL_CONNECTED)	  //only switch LED if Wifi is connected
	{
		digitalWrite(LED_BUILTIN, !state); // set pin to the opposite state
	}
	else
	{
		// Turn the LED on (Note that LOW is the voltage level
		// but actually the LED is on; this is because
		// it is acive low on the ESP-01)
		digitalWrite(LED_BUILTIN, HIGH); // Turn the LED off by making the voltage HIGH
	}
	return;
}

void setupTimeClient()
{
	DEBUG_PRINTLN("Fetching NTP time");
	_timeClient.setTimeOffset(OFFSET * 3600);
	_timeClient.begin();
}

void getDate_TimeUpdate()
{

	DEBUG_PRINTLN("Updating time & Date (NTP)");

	while (!_timeClient.forceUpdate())
	{
		DEBUG_PRINTLN("Fetching Time Update");
		delay(350);
	}
	DEBUG_PRINTLN(_timeClient.getFormattedTime());

	String __formattedDate = _timeClient.getFormattedDate();
	DEBUG_PRINTLN(__formattedDate);
	// The formattedDate comes with the following format:
	// 2018-05-28T16:00:13Z
	// We need to extract date and time
	// Extract date
	int __splitT = __formattedDate.indexOf("T");
	_ntpDate = __formattedDate.substring(0, __splitT);
	DEBUG_PRINTLN("Time & Date update complete");
}

String IpAddress2String(const IPAddress &ipAddress)
{

	return String(ipAddress[0]) + String(".") +
		   String(ipAddress[1]) + String(".") +
		   String(ipAddress[2]) + String(".") +
		   String(ipAddress[3]);
}

void mqttPublish(String topic, String value, bool retain)
{
	String __topic = ("stat/" + String(MQTT_CLIENTNAME) + "/" + topic);
	DEBUG_PRINTLN("    " + __topic + " " + value);
	_mqttClient.publish(__topic.c_str(), value.c_str(), retain);

	_lastPublishedMQTTMessage = __topic + " " + value + (retain ? "(retain)" : "");
}

void SendDailykW()
{
	mqttPublish("kWattDay", String(_kWhDailyTotalAccumulating, 2), false); //_dayMeasureIndex should be 1440
	_kWhDailyTotalAccumulating = 0;
	_dayMeasureIndex = 1;
}
void mqttPublish(String topic, String value)
{
	mqttPublish(topic, value, false);
}
void mqttPublishStat(String topic, String value, bool retain)
{
	mqttPublish(topic, value, retain);
}
void mqttTransmitInitStat()
{
	mqttPublishStat("init", "{\"value1\":\"" + IpAddress2String(WiFi.localIP()) + "\",\"value2\":\"" + WiFi.macAddress() + "\",\"value3\":\"" + MQTT_CLIENTNAME + "\",\"signalQuality\":\"" + _rssiQualityPercentage + "\",\"batteryVoltage\":" + String(_batteryVoltage) + "}", false);
}

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
	DEBUG_PRINT("Message arrived [");
	DEBUG_PRINT(topic);
	DEBUG_PRINT("] ");
	char message_buff[100];
	int i = 0;
	for (i = 0; i < length; i++)
	{
		message_buff[i] = payload[i];
	}
	message_buff[i] = '\0';
	String __payloadString = String(message_buff);

	DEBUG_PRINTLN(__payloadString);

	String __incomingTopic = String(topic);

	_lastMQTTMessage = __incomingTopic + " " + __payloadString;

	if (__incomingTopic == "cmnd/" + String(MQTT_CLIENTNAME) + "/reset")
	{
		DEBUG_PRINTLN("Resetting ESP");
		ESP.restart();
	}
	if (__incomingTopic == "cmnd/" + String(MQTT_CLIENTNAME) + "/info")
	{
		mqttTransmitInitStat();
	}

	if (__incomingTopic == "cmnd/" + String(MQTT_CLIENTNAME) + "/getDailykW")
	{
		SendDailykW();
	}
}

void mqttReconnect()
{

	if (_mqttClient.connected())
	{
		return;
	}

	String __mqttClientID = String(MQTT_CLIENTNAME) + _mqttPostFix;
	DEBUG_PRINT("MQTT reconnecting as: ");
	DEBUG_PRINTLN(__mqttClientID);
	int __retryCount = 0;
	while (!_mqttClient.connected() && __retryCount < 3)
	{
		if (_mqttClient.connect(__mqttClientID.c_str()))
		{
			DEBUG_PRINTLN("connected");

			// Once connected, publish an announcement...
			DEBUG_PRINTLN("MQTT: Setting callback");
			_mqttClient.setCallback(mqttCallback);

			// Once connected, publish an announcement...
			String __outMessage = "tele/" + String(MQTT_CLIENTNAME) + "/alive";
			_mqttClient.publish(__outMessage.c_str(), "1");
			__outMessage = "tele/" + String(MQTT_CLIENTNAME) + "/ip";
			_mqttClient.publish(__outMessage.c_str(), IpAddress2String(WiFi.localIP()).c_str());

			// ... and resubscribe (LIMIT TO SPECIFIC TOPICS********)
			delay(200);
			String __topic = "cmnd/" + String(MQTT_CLIENTNAME) + "/reset";
			_mqttClient.subscribe(__topic.c_str());
			delay(100);
			__topic = "cmnd/" + String(MQTT_CLIENTNAME) + "/info";
			_mqttClient.subscribe(__topic.c_str());
			delay(100);
			__topic = "cmnd/" + String(MQTT_CLIENTNAME) + "/getDailykW";
			_mqttClient.subscribe(__topic.c_str());

			mqttTransmitInitStat();

			return;
		}
		else
		{
			DEBUG_PRINT("failed, rc=");
			DEBUG_PRINT(_mqttClient.state());
			DEBUG_PRINTLN(" try again in 2 seconds");
			// Wait a few seconds before retrying
			delay(2000);
			__retryCount++;
		}
	}
}

void setupWebServer() 
{

	DEBUG_PRINTLN("Handling Web Request...");

	_httpServer.on("/", []()
				   {
					   String __infoStr = "";

					   __infoStr += "<div align=left><H1><i>" + String(MQTT_CLIENTNAME) + "</i></H1>";
					   __infoStr += "<br>Connected to: " + String(SSID) + " (" + _rssiQualityPercentage + "%)<br><hr>";

					   __infoStr += "Watt Minute: " + String(_liveUsageWattMin) + "Wmin\n<br>";
					   __infoStr += "Running W:  " + String(_currentUsageWatt) + "W\n<br>";
					   __infoStr += "Running daily usage: " + String(_kWhDailyTotalAccumulating) + "kWh\n<br>";
					   __infoStr += "# Hours measured: " + String(_dayMeasureIndex) + "\n<br>";
					   

					   __infoStr += "<br>Last Message Received:  <br><i>" + _lastMQTTMessage;
					   __infoStr += "<br>Last Message Published: <br><i>" + _lastPublishedMQTTMessage;

					   __infoStr += "<hr><br>IP Address: " + IpAddress2String(WiFi.localIP());
					   __infoStr += "<br>MAC Address: " + WiFi.macAddress();
					   __infoStr += "<br>" + String(MQTT_CLIENTNAME) + " - Firmware version: " + String(MCMDVERSION,1);
					   
					   __infoStr += "</div>";

					   String __retStr = loginIndex + __infoStr + loginIndex2;

					   _httpServer.sendHeader("Connection", "close");
					   _httpServer.send(200, "text/html", __retStr);
				   });

	_httpServer.on("/serverIndex", HTTP_GET, []()
				   {
					   _httpServer.sendHeader("Connection", "close");
					   _httpServer.send(200, "text/html", serverIndex);
				   });

	_httpServer.on("/reset", []()
				   {
					   String _webClientReturnString = "Resetting device";
					   _httpServer.send(200, "text/plain", _webClientReturnString);
					   ESP.restart();
					   delay(1000);
				   });
	_httpServer.on("/resetSettings", []()
				   {
					   String _webClientReturnString = "Resetting Settings";
					   _httpServer.send(200, "text/plain", _webClientReturnString);

					   if (SPIFFS.exists("/config.json"))
					   {
						   DEBUG_PRINTLN("Removing Configuration files from SPIFFS");
						   SPIFFS.remove("/config.json");
					   }

					   //saveConfigValuesSPIFFS();
					   //writeConfigValuesToAS3935();
				   });

	_httpServer.on("/defaults", []()
				   {
					   String _webClientReturnString = "Resetting device to defaults";
					   _httpServer.send(200, "text/plain", _webClientReturnString);
				   });

	/*handling uploading firmware file */
	_httpServer.on(
		"/update", HTTP_POST, []()
		{
			_httpServer.sendHeader("Connection", "close");
			_httpServer.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
			ESP.restart();
		},
		[]()
		{
			HTTPUpload &upload = _httpServer.upload();
			if (upload.status == UPLOAD_FILE_START)
			{
				DEBUG_PRINT("Update: ");
				DEBUG_PRINTLN(upload.filename.c_str());
				if (!Update.begin(UPDATE_SIZE_UNKNOWN))
				{ //start with max available size
					Update.printError(Serial);
				}
			}
			else if (upload.status == UPLOAD_FILE_WRITE)
			{
				/* flashing firmware to ESP*/
				if (Update.write(upload.buf, upload.currentSize) != upload.currentSize)
				{
					Update.printError(Serial);
				}
			}
			else if (upload.status == UPLOAD_FILE_END)
			{
				if (Update.end(true))
				{ //true to set the size to the current progress
					DEBUG_PRINTLN("Update Success:" + String(upload.totalSize) + "\nRebooting...\n");
				}
				else
				{
					Update.printError(Serial);
				}
			}
		});

	_httpServer.onNotFound(handleSendToRoot);

	_httpServer.begin();

	DEBUG_PRINTLN("Web Request Completed...");
}

int WiFiConnectionCheck()
{

	DEBUG_PRINTLN("WiFiConnectionCheck");
	if (WiFi.status() == WL_CONNECTED)
	{
		return true;
	}

	DEBUG_PRINT("********** Free Heap: ");
	DEBUG_PRINTLN(ESP.getFreeHeap());

	DEBUG_PRINTLN("\r\nConnecting to: " + String(SSID));
	//delay(1000); //pause to prevent 'brown out' - to much current required during startup
	WiFi.begin(SSID, WIFIPASSWORD);

	byte _connAttempts = 0;
	//WiFi.setSleep(false);
	while (WiFi.status() != WL_CONNECTED)
	{

		DEBUG_PRINT(".");
		_connAttempts++;
		if (_connAttempts == 60)
		{ //22 seconds

			DEBUG_PRINTLN();
			DEBUG_PRINTLN("WiFi is NOT Connected [" + String(SSID) + "]");

			ESP.restart();
		}
		delay(500);
	}

	DEBUG_PRINTLN();
	DEBUG_PRINTLN("WiFi connected\r\n");
	DEBUG_PRINT("IP address: ");
	DEBUG_PRINTLN(WiFi.localIP());
	DEBUG_PRINT("********** Free Heap: ");
	DEBUG_PRINTLN(ESP.getFreeHeap());

	_rssi = WiFi.RSSI();

	float _RSSI_Reading = WiFi.RSSI();
	_RSSI_Reading = isnan(_RSSI_Reading) ? -100.0 : _RSSI_Reading;
	_rssiQualityPercentage = min(max(2 * (_RSSI_Reading + 100.0), 0.0), 100.0);

	DEBUG_PRINT("RSSI: ");
	DEBUG_PRINTLNDEC(_rssi, 1);
	DEBUG_PRINT("WiFi Quality: ");
	DEBUG_PRINTDEC(_rssiQualityPercentage, 1);
	DEBUG_PRINTLN("%");

	DEBUG_PRINTLN("Complete");

	return 1;
}

/*
TL_DATUM = Top left
TC_DATUM = Top centre
TR_DATUM = Top right
ML_DATUM = Middle left
MC_DATUM = Middle centre
MR_DATUM = Middle right
BL_DATUM = Bottom left
BC_DATUM = Bottom centre
BR_DATUM = Bottom right

*/

void drawFooter()
{
	if (!_ledBacklight)
	{
		return;
	}

	if (millis() - _runFooter > 29000) //only footer screen every 29 seconds
	{
		_runFooter = millis();
		_tft.fillRect(0, _tft.height() - 23, _tft.width(), 23, TFT_BLACK);
	}

	//DEBUG_PRINT("drawFooter..");

	_tft.drawLine(0, _tft.height() - 24, 239, _tft.height() - 24, TFT_DARKGREY);
	//wifi connection indicator
	_tft.fillCircle(_tft.width() - 4, _tft.height() - 4, 3, (WiFi.status() == WL_CONNECTED) ? TFT_GREEN : TFT_RED);

	_tft.setTextDatum(MC_DATUM);
	_tft.setTextColor(TFT_WHITE);
	String __t = _timeClient.getFormattedTime();
	__t = __t.substring(0, __t.length() - 3);
	_tft.drawString(_ntpDate + " " + __t, 120, _tft.height() - 10, 4);

	//DEBUG_PRINTLN(".. complete");
}

void showDeviceInfo()
{
	if (!_ledBacklight)
	{
		return;
	}

	if (millis() - _runDeviceInfo > 15000) //only update screen every 15 seconds
	{
		_runDeviceInfo = millis();
		DEBUG_PRINTLN("showDeviceInfo");

		_tft.setTextDatum(TL_DATUM);

		_runDeviceInfo = millis();
		uint16_t v = analogRead(ADC_PIN);
		_batteryVoltage = ((float)v / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
		_tft.fillScreen(TFT_BLACK);

		String voltage = "Battery: " + String(_batteryVoltage) + "V";
		DEBUG_PRINTLN(voltage);

		_tft.drawString(voltage, 0, 10, 2);

		_tft.setTextColor(TFT_GREEN, TFT_BLACK);
		_tft.drawString("" + IpAddress2String(WiFi.localIP()), 5, 40, 2);
		_tft.setTextColor(TFT_GREEN, TFT_BLACK);

		String __isConnected = (WiFi.status() == WL_CONNECTED) ? "Connected to " : "Not connected to ";
		_tft.drawString(__isConnected + String(SSID), 5, 60, 2);
		_tft.drawString("WiFi Quality: " + String(_rssiQualityPercentage) + "%", 5, 80, 2);
		DEBUG_PRINTLN(" completed");
	}
}

void writeInfo_EnergyToDisplay(double watts, double amps)
{

	if (!_ledBacklight)
	{
		return;
	}

	//DEBUG_PRINT("writeInfo_EnergyToDisplay");
	_tft.setTextSize(1);
	//_tft.fillScreen(TFT_BLACK);//inefficient

	_tft.setTextDatum(MC_DATUM);

	//for each phase:
	double __totalAmp = 0;
	double __totalWatt = 0;

	_tft.fillRect(0, 30, _tft.width(), _tft.height() - 25, TFT_BLACK);

	for (byte i = 0; i < 3; i++)
	{
		_tft.setTextColor(TFT_WHITE, TFT_BLACK);
		_tft.drawString("P" + String(i), 40 + 80 * i, 0, 2);

		__totalAmp += _liveAmpMeasurement[i];
		_tft.setTextColor(TFT_RED, TFT_BLACK);
		_tft.drawString(String(_liveAmpMeasurement[i], 1) + "A", 40 + 80 * i, 30, 2);

		__totalWatt += _liveWattMeasurement[i];
		_tft.setTextColor(TFT_YELLOW);
		_tft.drawString(String(_liveWattMeasurement[i], 1) + "W", 40 + 80 * i, 50, 2);
	}

	_tft.setTextDatum(TR_DATUM);
	//total amp/wattage
	_tft.drawLine(0, 70, 239, 70, TFT_DARKGREEN);
	_tft.setTextColor(TFT_RED);
	_tft.drawString(String(__totalAmp, 0) + "A", 100, 80, 4);

	_tft.setTextColor(TFT_YELLOW);
	_tft.drawString(String(__totalWatt / 1000, 0) + "kW", 200, 80, 4);

	//DEBUG_PRINTLN(" completed");
}

void espDelay(int ms)
{
	//! Long time delay, it is recommended to use shallow sleep, which can effectively reduce the current consumption
	esp_sleep_enable_timer_wakeup(ms * 1000);
	esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
	esp_light_sleep_start();
}

void button_init()
{
	//btn1.setLongClickHandler([](Button2& b) {
	//	btnCick = false;
	//	int r = digitalRead(TFT_BL);
	//	_tft.fillScreen(TFT_BLACK);
	//	_tft.setTextColor(TFT_GREEN, TFT_BLACK);
	//	_tft.setTextDatum(MC_DATUM);
	//	_tft.drawString("Press again to wake up", _tft.width() / 2, _tft.height() / 2);
	//	espDelay(6000);
	//	digitalWrite(TFT_BL, !r);

	//	_tft.writecommand(TFT_DISPOFF);
	//	_tft.writecommand(TFT_SLPIN);
	//	esp_sleep_enable_ext1_wakeup(GPIO_SEL_35, ESP_EXT1_WAKEUP_ALL_LOW);
	//	esp_deep_sleep_start();
	//	});

	btn1.setPressedHandler([](Button2 &b)
						   {
							   DEBUG_PRINTLN("Show system info");
							   btnCick = true;
							   _frameIndex = 1;
						   });

	btn2.setPressedHandler([](Button2 &b)
						   {
							   DEBUG_PRINTLN("Show current/wattage info");
							   btnCick = true;
							   _frameIndex = 0;
						   });

	//btn2.setPressedHandler([](Button2& b) {
	//	btnCick = false;
	//	DEBUG_PRINTLN("btn press wifi scan");
	//	WiFi.disconnect();
	//	delay(500);
	//	WiFiConnectionCheck();
	//	});
}

void voltageReadSetup()
{
	esp_adc_cal_characteristics_t adc_chars;
	esp_adc_cal_value_t val_type = esp_adc_cal_characterize((adc_unit_t)ADC_UNIT_1, (adc_atten_t)ADC1_CHANNEL_6, (adc_bits_width_t)ADC_WIDTH_BIT_12, 1100, &adc_chars);
	//Check type of calibration value used to characterize ADC
	if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF)
	{
		Serial.printf("eFuse Vref:%u mV", adc_chars.vref);
		vref = adc_chars.vref;
	}
	else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP)
	{
		Serial.printf("Two Point --> coeff_a:%umV coeff_b:%umV\n", adc_chars.coeff_a, adc_chars.coeff_b);
	}
	else
	{
		DEBUG_PRINTLN("Default Vref: 1100mV");
	}
}

void backlightOff()
{
	if (_ledBacklight)
	{
		digitalWrite(TFT_BL, LOW);
		_ledBacklight = false;
	}
}

void backlightOn()
{
	if (!_ledBacklight)
	{
		digitalWrite(TFT_BL, HIGH);
		_ledBacklight = true;
	}
}

void setup()
{

	_mqttPostFix = String(random(0xffff), HEX);

#ifdef DEBUG
	Serial.begin(115200);
#endif // DEBUG

	DEBUG_PRINTLN("   ");
	DEBUG_PRINTLN("Initialising");

	DEBUG_PRINTLN("Breathing LED setup");
	pinMode(LED_BUILTIN, OUTPUT);

	DEBUG_PRINTLN("Button Init");
	button_init();

	DEBUG_PRINTLN("Screen Setup");
	_tft.begin();
	_tft.setRotation(3);
	_tft.fillScreen(TFT_BLACK);

	DEBUG_PRINTLN("Battery ADC setup");
	//voltageReadSetup(); DEBUG_PRINTLN();

	DEBUG_PRINTLN("Sketch size: ");
	DEBUG_PRINTLN(ESP.getSketchSize());
	DEBUG_PRINT("Free size: ");
	DEBUG_PRINTLN(ESP.getFreeSketchSpace());

	WiFiConnectionCheck();
	MDNS.begin(MQTT_CLIENTNAME);
	setupWebServer();

	randomSeed(micros());

	DEBUG_PRINT("Attempting MQTT connection to: ");
	DEBUG_PRINT(MQTT_SERVERADDRESS);
	DEBUG_PRINTLN(":1883");
	_mqttClient.setServer(MQTT_SERVERADDRESS, 1883);
	_mqttClient.setCallback(mqttCallback);

	DEBUG_PRINT(F("********** Free Heap: "));
	DEBUG_PRINTLN(ESP.getFreeHeap());
	DEBUG_PRINTLN(F("ESP WiFi -> OTA Setup"));
	setupOTA();
	MDNS.addService("http", "tcp", 80);

	pinMode(TFT_BL, OUTPUT); // Set backlight pin to output mode

	for (byte i = 0; i < 3; i++)
	{
		adc1_config_channel_atten(ADC_CHANNELS[i], ADC_ATTEN_DB_11);
	}
	analogReadResolution(ADC_BITS);

	// Initialize emon library (100 = calibration number)
	for (byte i = 0; i < 3; i++)
	{
		_eMonitorInstance[i].current(ADC_INPUT[i], ADC_calib[i]);
	}

	setupTimeClient();
	getDate_TimeUpdate();

	//force first connection:
	mqttReconnect();

	_timeFinishedSetup = millis();
}

void loop()
{

	_runCurrent = millis(); //sets the counter

	if (btnCick)
	{
		DEBUG_PRINTLN("btnCick");
		_tft.fillScreen(TFT_BLACK);
		_runScreenSaver = _runCurrent;
		backlightOn();
		btnCick = false;
	}
	if (millis() - _timeFinishedSetup > 15000)
	{
		_initFinished = true;
	}
	if (_initFinished && (_runCurrent - _runScreenSaver >= SCREEN_SAVER_INTERVAL_MILLISECS))
	{
		backlightOff();
	}
	if (_runCurrent - _runWiFiCheck >= WIFI_TICKLED_INTERVAL_MILLISECS)
	{
		_runWiFiCheck = _runCurrent;
		tickLED();
		WiFiConnectionCheck();
		mqttReconnect();

		DEBUG_PRINTLN("Wifi check complete");
	}
	if (_runCurrent - _runScreenUpdate >= SCREEN_UPDATE_INTERVAL_MILLISECS)
	{

		_runScreenUpdate = _runCurrent;

		// Update the display
		if (_frameIndex)
		{
			showDeviceInfo();
		}
		else
		{
			writeInfo_EnergyToDisplay(_watt_minute, _ampsCurrent);
		}

		drawFooter();
	}

	if (_initFinished && (_runCurrent - _lastMeasurement > 1000))
	{
		_lastMeasurement = _runCurrent;
		//DEBUG_PRINTLN("Calculate usage /second");
		_minuteMeasureIndex++;

		//for (byte i = 0; i < 3; i++)
		//{
		//	// ************************************************ **************************
		//	// Phase1 ..3
		//	for (int n = 0; n < numberOfSamples; n++)
		//	{
		//		// Used for offset removal
		//		lastSampleI[i] = sampleI[i];
		//		// Read in voltage and current samples.
		//		sampleI[i] = analogRead(ADC_INPUT[i]);
		//		// Used for offset removal
		//		lastFilteredI[i] = filteredI[i];
		//		// Digital high pass filters to remove 1.6V DC offset.
		//		filteredI[i] = 0.9989 * (lastFilteredI[i] + sampleI[i] - lastSampleI[i]);
		//		// Root-mean-square method current
		//		// 1) square current values
		//		sqI[i] = filteredI[i] * filteredI[i];
		//		// 2) sum
		//		sumI[i] += sqI[i];
		//		delay(0.0002);
		//	}

		//	// Calculation of the root of the mean of the voltage and current squared (rms)
		//	// Calibration coeficients applied.
		//	Irms1[i] = (I_RATIO * sqrt(sumI[i] / numberOfSamples)) - 0.06;
		//	if (Irms1[i] < 0) { Irms1[i] = 0; };  // Set negative current to zero
		//	sumI[i] = 0;

		//	_liveAmpMeasurement[i] = Irms1[i];
		//	_liveWattMeasurement[i] = _liveAmpMeasurement[i] * HOME_VOLTAGE;

		//	_ampsCurrent += _liveAmpMeasurement[i];
		//	_watt_minute += _liveWattMeasurement[i];
		//}

		// CALCULATE FOR EVERY SECOND
		double __liveWatts = 0;
		
		DEBUG_PRINT("[" + String(_minuteMeasureIndex) + "] ");
		for (byte i = 0; i < TOTALPHASES; i++)
		{
			// Readings are unstable the first 10 seconds when the device powers on
			// so ignore them until they stabilise.
			_liveAmpMeasurement[i] = _eMonitorInstance[i].calcIrms(4000); // Calculate Irms only
			_liveWattMeasurement[i] = _liveAmpMeasurement[i] * HOME_VOLTAGE;

			_ampsCurrent += _liveAmpMeasurement[i];
			_watt_minute += _liveWattMeasurement[i];
			__liveWatts += _liveWattMeasurement[i];
			DEBUG_PRINT("[W: " + String(_liveWattMeasurement[i], 1) + "  ");
			//_wattMeasurementsHistory[i][_measureIndex] = _liveWattMeasurement[i];
		}
		DEBUG_PRINTLN("=Total: " + String(__liveWatts, 1) + "W");

		// When we have 60 measurements (1 Minute), send them to MQTT!
		if (_minuteMeasureIndex == 60)  
		{
			mqttPublish("amps",String(_ampsCurrent, 1));
			mqttPublish("volts",String(HOME_VOLTAGE,1));
			_liveUsageWattMin = _watt_minute / 60;
			_watt_minute_total += _watt_minute;
			_currentUsageWatt += _liveUsageWattMin;
			
			_currentUsageWattHour += (_liveUsageWattMin/60 );
			_hourMeasureIndex++;

	

			DEBUG_PRINTLN("Sending Watt minute");
			mqttPublish("Wmin", String(_watt_minute, 2));
			DEBUG_PRINTLN("Wmin: " + String(_watt_minute, 2));
			mqttPublish("kWatt", String(_liveUsageWattMin/1000, 1));
			DEBUG_PRINTLN("kWatt: " + String(_liveUsageWattMin/1000, 1));

			_watt_minute = 0;
			_minuteMeasureIndex = 0;

			DEBUG_PRINTLN(" completed");
		}

		_ampsCurrent = 0;
		// When we have 60 measurements (1 hour), send them to MQTT!
		if (_hourMeasureIndex == 60)
		{

			DEBUG_PRINTLN("Sending kW Hour");
			mqttPublish("kWh", String(_currentUsageWattHour / 1000, 1));
			DEBUG_PRINTLN("kWh: " + String(_currentUsageWattHour / 1000, 1));

			_kWhDailyTotalAccumulating += (_currentUsageWattHour / 1000);
			_dayMeasureIndex++;

			_watt_minute_total = 0;
			_hourMeasureIndex = 0;
			_currentUsageWatt = 0;
			_currentUsageWattHour = 0;
			DEBUG_PRINTLN(" completed");
		}
	}

	btn1.loop();
	btn2.loop();

	if (WiFi.status() == WL_CONNECTED)
	{
		_mqttClient.loop();
		ArduinoOTA.handle();		/* this function will handle incomming chunk of SW, flash and respond sender */
		_httpServer.handleClient(); //// Check if a client has connected
	}
}
