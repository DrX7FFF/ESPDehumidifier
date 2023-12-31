#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>
#include <mydebug.h>
#include <myfunction.h>
#include <DFRobot_SHT3x.h>
#include "SimpleKalmanFilter.h"
#include "ESP_ADC.h"
#include <AsyncTCP.h>

DFRobot_SHT3x   sht3x;

#define PORTPLOT 47269
#define PORTNR 8888
IPAddress broadcastIP;

#define LED				2
#define PELTIER_PIN		4
#define HOTFAN_PIN		16
#define COLDFAN_PIN		17
#define SHT_SDA			18
#define SHT_SCL			19
#define TEMPOUT_PIN		39
#define TEMPHOT_PIN		34   // GPIO 34 = A?, uses any valid Ax pin as you wish
#define TEMPCOLD_PIN	35  // GPIO 35 = A7, uses any valid Ax pin as you wish

#define R1 		100000 		// voltage divider resistor value
#define BETA 	3950.0		// Beta value
#define T0 		298.15 		// Temperature in Kelvin for 25 degree Celsius
#define R0 		100000		// Resistance of Thermistor at 25 degree Celsius 94kOhm
#define ADCMAX 	4095		// Résolution MAX
#define VSS 	3.3			// Tension MAX

#define COLDFAN_SPEEDMIN 2
#define COLDFAN_SPEEDMAX 0x1F

#define DELAY_REGUL		30000	//30 s
#define DELAY_STOP 		600000	//180000	// 3 minutes (1.5 min pour le démarrage)
#define DELAY_FLOW 		120000	// 5 minutes
#define DELAY_REFLOW	3600000	// 60 minutes

#define FILTER_TEMP_PRECISION	0.4
#define FILTER_TEMP_SPEED		0.01


//SimpleKalmanFilter simpleKalmanFilterHot(0.5, 0.5, 0.01);
SimpleKalmanFilter simpleKalmanFilterHot(FILTER_TEMP_PRECISION, FILTER_TEMP_PRECISION, FILTER_TEMP_SPEED);
SimpleKalmanFilter simpleKalmanFilterCold(FILTER_TEMP_PRECISION, FILTER_TEMP_PRECISION, FILTER_TEMP_SPEED);
SimpleKalmanFilter simpleKalmanFilterOut(FILTER_TEMP_PRECISION, FILTER_TEMP_PRECISION, 0.05);
// SimpleKalmanFilter simpleKalmanFilterHumidity(0.1, 0.1, 0.4);  //Très proche du réel et filtre les petites variations
SimpleKalmanFilter simpleKalmanFilterHumidity(0.2, 0.2, 0.4);  //Très proche du réel et filtre les petites variations
//SimpleKalmanFilter simpleKalmanFilterHumidity(0.4, 0.4, 0.1);
SimpleKalmanFilter simpleKalmanFilterDelta(0.2, 0.2, 0.05);

uint8_t coldFanSpeed = 0;
bool hotFanOn = false;
bool peltierOn = false;
bool pushJSON = false;
bool ledOn = false;

unsigned long memMillisRegul = 0;
uint32_t memMillisStop = 0;
uint32_t memMillisFlowMode = 0;
uint32_t memMillisIDLEMode = 0;

float temperature;
float humidity;
float tempCold;
float tempHot;
float tempOut;
float tempOutRaw;
float tempOutMax = -100;
float tempOutMin = 100;
float tempDewPoint;
float tempDelta;
int regul;

enum mode {
	IDLE,
	MISTINESS,
	FLOW,
	REFRESH,
	ERROR
};

mode activeMode = mode::IDLE;


float readTemp(uint8_t pin) {
	float adc = ADC_LUT[analogRead(pin)];
	float Vout = adc * VSS / ADCMAX;
	float Rt = R1 * Vout / (VSS - Vout);

	float T = 1 / (1 / T0 + log(Rt / R0) / BETA);  // Temperature in Kelvin
	float Tc = T - 273.15;                         // Celsius
	Tc = Tc + 1.5;                                 // Correction ancienne valeur 1.5
	return Tc;
}

void sendToNR(bool fanSpeedOnly = false) {
	static char buffer[300];
	static WiFiUDP udpNR;
	static uint32_t memMillis = 0;

	if (fanSpeedOnly){
		sprintf(buffer, "{\"coldFanSpeed\":%d}\0",
				coldFanSpeed);
		udpNR.beginPacket(broadcastIP, PORTNR);
		udpNR.print(buffer);
		udpNR.endPacket();
	}
	else
		if (millis() - memMillis > 300000 || pushJSON) {
			memMillis = millis();
			pushJSON = false;
			sprintf(buffer, "{\"temperature\":%3.1f,\"humidity\":%3.1f,\"absHumidity\":%3.1f,\"dewPoint\":%3.1f,\"tempCold\":%3.1f,\"tempHot\":%3.1f,\"tempOut\":%3.1f}\0",
					temperature,
					humidity,
					sht3x.computeAbsoluteHumidity(temperature, humidity),
					tempDewPoint,
					tempCold,
					tempHot,
					(tempOutMax+tempOutMin)/2);
			tempOutMax = tempOut;
			tempOutMin = tempOut;
			udpNR.beginPacket(broadcastIP, PORTNR);
			udpNR.print(buffer);
			udpNR.endPacket();
		}

}

void setColdFan(int fanSpeed) {
	if (fanSpeed > COLDFAN_SPEEDMAX)
		fanSpeed = COLDFAN_SPEEDMAX;
	if (fanSpeed < COLDFAN_SPEEDMIN)
		fanSpeed = 0;
	if (fanSpeed == coldFanSpeed)
		return;	

	coldFanSpeed = fanSpeed;
	ledcWrite(0, coldFanSpeed);
	DEBUGLOG("Cold speed : %d\r\n", coldFanSpeed);
	sendToNR(true);
}

void upColdFan() {
	if (coldFanSpeed == 0)
		setColdFan(COLDFAN_SPEEDMAX);
	else
		setColdFan(coldFanSpeed + 1);
}

void downColdFan() {
	if (coldFanSpeed > COLDFAN_SPEEDMIN)
		setColdFan(coldFanSpeed - 1);
}

void setPeltier(bool cmd) {
	if (cmd == peltierOn)
		return;
	peltierOn = cmd;
	digitalWrite(PELTIER_PIN, peltierOn);
	DEBUGLOG("Peltier : %s\r\n", peltierOn ? "On" : "Off");
}

void setHotFan(bool cmd) {
	if (cmd == hotFanOn)
		return;
	hotFanOn = cmd;
	digitalWrite(HOTFAN_PIN, hotFanOn);
	DEBUGLOG("Hot fan : %s\r\n", hotFanOn ? "On" : "Off");
}

void displayMode(){
	DEBUGLOG("Mode : ");
	switch (activeMode) {
		case mode::IDLE:
			DEBUGLOG("IDLE");
			break;
		case mode::FLOW:
			DEBUGLOG("FLOW");
			break;
		case mode::MISTINESS:
			DEBUGLOG("MISTINESS");
			break;
		case mode::REFRESH:
			DEBUGLOG("REFRESH");
			break;
		case mode::ERROR:
			DEBUGLOG("ERROR");
			break;
		default:
			DEBUGLOG("???");
			break;
	}
	DEBUGLOG("\n");
}

void setMode(mode newMode) {
	if ((newMode == mode::IDLE) && (activeMode == mode::MISTINESS) )
		newMode = mode::REFRESH;
	activeMode = newMode;
	displayMode();
	switch (activeMode) {
		case mode::IDLE:
			setPeltier(false);
			setHotFan(false);
			setColdFan(0);
			memMillisIDLEMode = millis();
			break;
		case mode::FLOW:
			setPeltier(false);
			setHotFan(true);
			setColdFan(0);
			memMillisFlowMode = millis();
			break;
		case mode::MISTINESS:
			setPeltier(true);
			setHotFan(true);
			setColdFan(COLDFAN_SPEEDMAX);
			memMillisStop = millis();  // Temporise l'arrêt si Out>DewPoint
			break;
		case mode::REFRESH:
			setPeltier(false);
			setHotFan(true);
			setColdFan(COLDFAN_SPEEDMAX);
			break;
		case mode::ERROR:
			setPeltier(false);
			setHotFan(false);
			setColdFan(0);
		default:
			break;
	}
}

void onReceiveDebug(void *arg, AsyncClient *client, void *data, size_t len) {
	// DEBUGLOG("Receive %d car %.*s\n", len, len, (char*)data);
	switch (((char *)data)[0]) {
		case '0':
			setMode(mode::IDLE);
			break;
		case '1':
			setMode(mode::MISTINESS);
			break;
		case 'r':
			setMode(mode::REFRESH);
			break;
		case 'f':
			setMode(mode::FLOW);
			break;
		case 'H':
			setHotFan(!hotFanOn);
			break;
		case 'P':
			setPeltier(!peltierOn);
			break;
		case 'J':
			pushJSON = true;
			break;
		case 'C':
			if (coldFanSpeed)
				setColdFan(0);
			else
				setColdFan(COLDFAN_SPEEDMAX);
			break;
		case '+':
			upColdFan();
			break;
		case '-':
			downColdFan();
			break;
		case '8':
			if(!sht3x.softReset())
				DEBUGLOG("Failed to Reset the chip....\n");
			if(!sht3x.startPeriodicMode(sht3x.eMeasureFreq_10Hz,sht3x.eRepeatability_High))
				DEBUGLOG("Failed to enter the periodic mode\n");
			break;
		case '9':
			ESP.restart();
			break;
		case '?':
			DEBUGLOG("?        Help\r\n");
			DEBUGLOG("9        Restart\r\n");
			DEBUGLOG("H        Hot fan swap\r\n");
			DEBUGLOG("P        Peltier swap\r\n");
			DEBUGLOG("J        Push JSON\r\n");
			DEBUGLOG("C        Cold fan swap\r\n");
			DEBUGLOG("+        Cold Up\r\n");
			DEBUGLOG("-        Cold Down\r\n");
			DEBUGLOG("0        IDLE Mode\r\n");
			DEBUGLOG("1        Mistiness Mode\r\n");
			DEBUGLOG("r        REFRESH Mode\r\n");
			DEBUGLOG("f        FLOW Mode\r\n");
			DEBUGLOG("Peltier : %s\r\n", peltierOn ? "On" : "Off");
			DEBUGLOG("Hot fan : %s\r\n", hotFanOn ? "On" : "Off");
			DEBUGLOG("Cold speed : %d\r\n", coldFanSpeed);
			displayMode();
	}
}

void sendToTeleplot() {
	static char buffer[300];
	static WiFiUDP udpPlot;

	sprintf(buffer, "temperature:%3.1f\nhumidity:%3.1f\ndewPoint:%3.1f\ntempCold:%3.1f\ntempHot:%3.1f\ntempOut:%3.1f\ntempOutRaw:%3.1f\nfanSpeed:%d\ndeltaRaw:%3.1f\nTimerStop:%d\ndelta:%3.1f\nregul:%d\nTimerRegul:%d\n",
			temperature,
			humidity,
			tempDewPoint,
			tempCold,
			tempHot,
			tempOut,
			tempOutRaw,
			coldFanSpeed,
			tempDewPoint - tempOut,
			(DELAY_STOP + memMillisStop - millis()) / 1000,
			tempDelta,
			regul,
			(DELAY_REGUL + memMillisRegul - millis()) /1000);
	udpPlot.beginPacket(broadcastIP, PORTPLOT);
	udpPlot.print(buffer);
	udpPlot.endPacket();
}

void setup() {
	DEBUGINIT();
	DEBUGSETDATAHANDLER(onReceiveDebug);
	loadIP();
	WiFi.begin();

	DEBUGLOG("Setup\nWait WiFi for 30s :");
	uint32_t mem = millis();
	while ((millis() - mem) < 30000 && !WiFi.isConnected()) {
		delay(1);
	}

	if (!WiFi.isConnected()) {
		DEBUGLOG("\nStart SmartConfig for 60s :");
		WiFi.beginSmartConfig();
		mem = millis();
		while ((millis() - mem) < 60000 && !WiFi.smartConfigDone()) {
			delay(1000);
			DEBUGLOG("/");
		}
		if (WiFi.smartConfigDone()) {
			DEBUGLOG("\nSmartConfig Done\n");
			WiFi.begin(WiFi.SSID().c_str(), WiFi.psk().c_str(), WiFi.channel(), WiFi.BSSID());
			delay(1000);
			ESP.restart();
		} else
			DEBUGLOG("\nSmartConfig Abort, continue without WiFi\n");
	}

	IPAddress myIP = WiFi.localIP();
	IPAddress myMask = WiFi.subnetMask();
	for (int i = 0; i < 4; i++)
		broadcastIP[i] = (myIP[i] & myMask[i]) | (~myMask[i]);
	DEBUGLOG("\nIPLocal %s\nMask %s\nBroadCast %s\n", myIP.toString().c_str(), myMask.toString().c_str(), broadcastIP.toString().c_str());

	ArduinoOTA.begin();

	// Init pin
	pinMode(TEMPCOLD_PIN,INPUT);
	pinMode(TEMPHOT_PIN,INPUT);
	pinMode(TEMPOUT_PIN,INPUT);
	pinMode(HOTFAN_PIN, OUTPUT);
	pinMode(PELTIER_PIN, OUTPUT);
	pinMode(COLDFAN_PIN, OUTPUT);
	pinMode(LED, OUTPUT);
	ledcSetup(0, 25000, 5);
	ledcAttachPin(COLDFAN_PIN, 0);

	Wire.setPins(SHT_SDA,SHT_SCL);
	DEBUGLOG("Init SHT3x\n");
	if (sht3x.begin() != 0)
		DEBUGLOG("Failed to Initialize the chip....\n");
	if(!sht3x.softReset())
		DEBUGLOG("Failed to Reset the chip....\n");
	if(!sht3x.startPeriodicMode(sht3x.eMeasureFreq_10Hz,sht3x.eRepeatability_High))
    	DEBUGLOG("Failed to enter the periodic mode\n");

	DEBUGLOG("Setup complet\n");
	setMode(mode::REFRESH);
}

// TODO : Attention si pas de lecture temperature alors mettre en IDLE
void loop() {
	ledOn = !ledOn;
	digitalWrite(LED, ledOn);

	// Read values
	tempCold = simpleKalmanFilterCold.updateEstimate(readTemp(TEMPCOLD_PIN));
	tempHot = simpleKalmanFilterHot.updateEstimate(readTemp(TEMPHOT_PIN));
	tempOutRaw = readTemp(TEMPOUT_PIN);
	tempOut = simpleKalmanFilterOut.updateEstimate(tempOutRaw);

	if (tempOut > tempOutMax)
		tempOutMax = tempOut;
	if (tempOut < tempOutMin)
		tempOutMin = tempOut;
	DFRobot_SHT3x::sRHAndTemp_t data = sht3x.readTemperatureAndHumidity();
	if(data.ERR == 0){
		humidity = simpleKalmanFilterHumidity.updateEstimate(data.Humidity);
		temperature = data.TemperatureC;
		tempDewPoint = sht3x.computeDewPoint(temperature, humidity);
		tempDelta = simpleKalmanFilterDelta.updateEstimate(tempDewPoint - tempOut);
	}
	else{
    	DEBUGLOG("Failed to read Data\n");
		// DEBUGLOG("Init SHT3x\n");
		// if (sht3x.begin() != 0)
		// 	DEBUGLOG("Failed to Initialize the chip....\n");
		// if(!sht3x.softReset())
		// 	DEBUGLOG("Failed to Reset the chip....\n");
		// if(!sht3x.startPeriodicMode(sht3x.eMeasureFreq_10Hz,sht3x.eRepeatability_High))
		// 	DEBUGLOG("Failed to enter the periodic mode\n");
		setMode(mode::ERROR);
	}

	switch (activeMode) {
		case mode::IDLE:
			if (millis() - memMillisIDLEMode>DELAY_REFLOW)
				setMode(mode::FLOW);
			break;
		case mode::FLOW:
			if (millis() - memMillisFlowMode>DELAY_FLOW)
				setMode(mode::IDLE);
			break;
		case mode::MISTINESS:
			regul = (tempDelta - 1.2) / 0.4;
			regul = (regul>0)?floor(regul):ceil(regul);
			if ((regul != 0) && (millis() - memMillisRegul > DELAY_REGUL )){
				memMillisRegul = millis();
				setColdFan(coldFanSpeed + regul>=COLDFAN_SPEEDMIN?coldFanSpeed + regul:COLDFAN_SPEEDMIN);
			}

			if ((tempDelta>0) || (coldFanSpeed > COLDFAN_SPEEDMIN))
				memMillisStop = millis();  // Si en dessous du DewPoint, Temporise l'arrêt
			if (millis() - memMillisStop > DELAY_STOP)  // si 2 min au dessus du DewPoint et vitesse au minimum alors arrêt
				setMode(mode::REFRESH);
			
			if (tempHot > 54)  // Si TempHot trop chaud alors arrêt
				setMode(mode::REFRESH);
			break;
		case mode::REFRESH:
			if (tempHot < temperature + 5)
				setMode(mode::IDLE);
			break;
		case mode::ERROR:
			// Ne rien faire
			break;
		default:
			setMode(mode::IDLE);
			break;
	}

	sendToTeleplot();
	sendToNR();
	ArduinoOTA.handle();

	delay(1000);
}
