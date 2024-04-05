#include <Arduino.h>
#include <ArduinoOTA.h>
#include <AsyncTCP.h>
#include <DFRobot_SHT3x.h>
#include <WiFi.h>
#include <mydebug.h>
#include <myfunction.h>

#include "ESP_ADC.h"
#include "PIDControler.h"
#include "SimpleKalmanFilter.h"

#define PORTPLOT 47269
#define PORTNR 8888

#define LIGHT_PIN 23
#define PELTIER_PIN 4
#define HOTFAN_PIN 16
#define COLDFAN_PIN 17
#define SHT_SDA 18
#define SHT_SCL 19
#define TEMPOUT_PIN 39
#define TEMPHOT_PIN 34   // GPIO 34 = A?, uses any valid Ax pin as you wish
#define TEMPCOLD_PIN 35  // GPIO 35 = A7, uses any valid Ax pin as you wish

#define ADCRES 10

#define R1 100000    // voltage divider resistor value
#define BETA 3950.0  // Beta value
#define T0 298.15    // Temperature in Kelvin for 25 degree Celsius
#define R0 100000    // Resistance of Thermistor at 25 degree Celsius 94kOhm
#define VSS 3300      // Tension MAX

#define COLDFAN_RESOLUTION 5                            // 6
#define COLDFAN_SPEEDMIN 2                              // 2 pour 9V 3 pour 8V  4 pour Resolution de 6
#define COLDFAN_SPEEDMAX (1 << COLDFAN_RESOLUTION) - 1  // 0x1F

// #define TEMPHOT_MAX 54
#define TEMPHOT_MAX 60

#define DELAY_STOP 600000     // 10 minutes (1.5 min pour le démarrage)
#define DELAY_FLOW 120000     // 2 minutes
// #define DELAY_REFLOW 3600000  // 60 minutes
#define DELAY_REFLOW 18000000  // 5H
#define DELAY_LIGHT	60000	// 2 minutes

#define DEWPOINT_OFFSET -1

DFRobot_SHT3x sht3x;
PIDController pid(6, 0.06, 0, COLDFAN_SPEEDMIN, COLDFAN_SPEEDMAX);

// SimpleKalmanFilter FilterOut(0.1, 0.01);
// SimpleKalmanFilter FilterDewPoint(0.1, 0.05);
SimpleKalmanFilter FilterHumidity(0.2, 0.4);  // Très proche du réel et filtre les petites variations

uint8_t coldFanSpeed = 0;
bool pushJSON = false;
bool manu = false;
bool teleplot = false;

uint32_t memMillisStop = 0;
uint32_t memMillisFlowMode = 0;
uint32_t memMillisIDLEMode = 0;
uint32_t memMillisLight = 0;

#define MEASUREERROR 30
#define PROCESSNOISE 0.22

// uint16_t mvoltHot = 0;
// uint16_t mvoltCold = 0;
// uint16_t mvoltOut = 0;
SimpleKalmanFilter filterVoltageHot(MEASUREERROR, PROCESSNOISE); //10 Amplitude du bruit pour une résolution sur 9 bits 512
SimpleKalmanFilter filterVoltageCold(MEASUREERROR, PROCESSNOISE); //10 Amplitude du bruit pour une résolution sur 9 bits 512
SimpleKalmanFilter filterVoltageOut(MEASUREERROR, PROCESSNOISE); //10 Amplitude du bruit pour une résolution sur 9 bits 512


float temperature;
float humidity;
float tempHot;
float tempCold;
float tempOut;
float tempDewPoint;
float tempDewPoint2;

enum mode {
	IDLE,
	MISTINESS,
	FLOW,
	REFRESH,
	ERROR
};

mode activeMode = mode::IDLE;

TaskHandle_t pADCRead;
void ADCRead( void * parameter){
	const TickType_t taskPeriod = 10; // 20ms <--> 50Hz
	TickType_t xLastWakeTime = xTaskGetTickCount();
	for (;;)  {
		// mvoltHot = (uint16_t)filterVoltageHot.updateEstimate(analogRead(TEMPHOT_PIN));
		// mvoltCold = (uint16_t)filterVoltageCold.updateEstimate(analogRead(TEMPCOLD_PIN));
		// mvoltOut = (uint16_t)filterVoltageOut.updateEstimate(analogRead(TEMPOUT_PIN));
		tempHot = ADC_LUT[(uint16_t)filterVoltageHot.updateEstimate(analogRead(TEMPHOT_PIN))]/10;
		tempCold = ADC_LUT[(uint16_t)filterVoltageCold.updateEstimate(analogRead(TEMPCOLD_PIN))]/10;
		tempOut = ADC_LUT[(uint16_t)filterVoltageOut.updateEstimate(analogRead(TEMPOUT_PIN))]/10;

		
		vTaskDelayUntil(&xLastWakeTime, taskPeriod);
	}
}

// long double readTemp(uint16_t mVolt) {
// 	float Vout = ADC_LUT[mVolt];
// 	float Rt = R1 * Vout / (VSS - Vout);
// 	float T = (1 / (1 / T0 + log(Rt / R0) / BETA))- 273.15;  // Temperature in Celsius
// 	return T;
// }

void sendToNR() {
	static char buffer[300];
	static WiFiUDP udpNR;
	static uint32_t memMillis = 0;

	if (millis() - memMillis > 300000 || pushJSON) {
		memMillis = millis();
		pushJSON = false;
		sprintf(buffer, "{\"temperature\":%.1f,\"humidity\":%.1f,\"absHumidity\":%.1f,\"dewPoint\":%.1f,\"tempCold\":%.1f,\"tempHot\":%.1f,\"tempOut\":%.1f,\"coldFanSpeed\":%.1f}\0",
				temperature,
				humidity,
				sht3x.computeAbsoluteHumidity(temperature, humidity),
				tempDewPoint,
				tempCold,
				tempHot,
				tempOut,
				activeMode==mode::MISTINESS ? pid.getI() : coldFanSpeed);
		udpNR.beginPacket(WiFi.broadcastIP(), PORTNR);
		udpNR.print(buffer);
		udpNR.endPacket();
	}
}

void setColdFan(uint8_t fanSpeed) {
	if (fanSpeed > COLDFAN_SPEEDMAX)
		fanSpeed = COLDFAN_SPEEDMAX;
	if (fanSpeed < COLDFAN_SPEEDMIN)
		fanSpeed = 0;
	if (fanSpeed == coldFanSpeed)
		return;

	coldFanSpeed = fanSpeed;
	// ledcWrite(0, coldFanSpeed);
	DEBUGLOG("Cold speed : %d\n", coldFanSpeed);
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

void setLight(bool cmd) {
	if (cmd == digitalRead(LIGHT_PIN))
		return;
	digitalWrite(LIGHT_PIN, cmd);
	DEBUGLOG("Light : %s\n", digitalRead(LIGHT_PIN) ? "On" : "Off");
}

void setPeltier(bool cmd) {
	if (cmd == digitalRead(PELTIER_PIN))
		return;
	digitalWrite(PELTIER_PIN, cmd);
	DEBUGLOG("Peltier : %s\n", digitalRead(PELTIER_PIN) ? "On" : "Off");
}

void setHotFan(bool cmd) {
	if (cmd == digitalRead(HOTFAN_PIN))
		return;
	digitalWrite(HOTFAN_PIN, cmd);
	DEBUGLOG("Hot fan : %s\n", digitalRead(HOTFAN_PIN) ? "On" : "Off");
}

void displayMode() {
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
	if ((newMode == mode::IDLE) && (activeMode == mode::MISTINESS))
		newMode = mode::REFRESH;
	switch (newMode) {
		case mode::IDLE:
			memMillisIDLEMode = millis();
			activeMode = newMode;
			setPeltier(false);
			setHotFan(false);
			setColdFan(0);
			break;
		case mode::FLOW:
			memMillisFlowMode = millis();
			activeMode = newMode;
			setPeltier(false);
			setHotFan(true);
			setColdFan(COLDFAN_SPEEDMAX);
			delay(1000);  // Attendre 1s que les fan démarrent
			break;
		case mode::MISTINESS:
			activeMode = newMode;
			setPeltier(true);
			setHotFan(true);
			setColdFan(COLDFAN_SPEEDMAX);
			delay(1000);  // Attendre 1s que les fan démarrent
			break;
		case mode::REFRESH:
			activeMode = newMode;
			setPeltier(false);
			setHotFan(true);
			setColdFan(COLDFAN_SPEEDMAX);
			delay(1000);  // Attendre 1s que les fan démarrent
			break;
		case mode::ERROR:
			activeMode = newMode;
			setPeltier(false);
			setHotFan(false);
			setColdFan(0);
		default:
			break;
	}
	displayMode();
	DEBUGLOG("Cold speed : %d\n", coldFanSpeed);
}

void onReceiveDebug(void *data, size_t len) {
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
			setHotFan(!digitalRead(HOTFAN_PIN));
			break;
		case 'P':
			setPeltier(!digitalRead(PELTIER_PIN));
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
		case 'I':
			if (!sht3x.softReset())
				DEBUGLOG("Failed to Reset the chip....\n");
			if (!sht3x.startPeriodicMode(sht3x.eMeasureFreq_10Hz, sht3x.eRepeatability_High))
				DEBUGLOG("Failed to enter the periodic mode\n");
			break;
		case 'O':
			ESP.restart();
			break;
		case 'L':
			setLight(!digitalRead(LIGHT_PIN));
			break;
		case '4':
			pid.kp -= 0.1;
			DEBUGLOG("PID kP : %.3f\n", pid.kp);
			break;
		case '7':
			pid.kp += 0.1;
			DEBUGLOG("PID kP : %.3f\n", pid.kp);
			break;
		case '5':
			pid.ki -= 0.01;
			DEBUGLOG("PID kI : %.3f\n", pid.ki);
			break;
		case '8':
			pid.ki += 0.01;
			DEBUGLOG("PID kI : %.3f\n", pid.ki);
			break;
		case '6':
			pid.kd -= 0.1;
			DEBUGLOG("PID kD : %.3f\n", pid.kd);
			break;
		case '9':
			pid.kd += 0.1;
			DEBUGLOG("PID kD : %.3f\n", pid.kd);
			break;
		case 'M':
			manu = !manu;
			DEBUGLOG("Manu : %s\n", manu ? "On" : "Off");
			break;
		case 't':
			teleplot = !teleplot;
			DEBUGLOG("Teleplot : %s\n", teleplot ? "On" : "Off");
			break;
		case '?':
			DEBUGLOG("?        Help\n");
			DEBUGLOG("0        IDLE Mode\n");
			DEBUGLOG("1        Mistiness Mode\n");
			DEBUGLOG("r        REFRESH Mode\n");
			DEBUGLOG("f        FLOW Mode\n");
			DEBUGLOG("J        Push JSON\n");
			DEBUGLOG("I        Init sht3x\n");
			DEBUGLOG("O        Restart ESP\n");
			DEBUGLOG("+        Cold Up\n");
			DEBUGLOG("-        Cold Down\n");
			DEBUGLOG("C        Cold fan swap\t\t[%d]\n", coldFanSpeed);
			DEBUGLOG("H        Hot fan swap\t\t[%s]\n", digitalRead(HOTFAN_PIN) ? "On" : "Off");
			DEBUGLOG("P        Peltier swap\t\t[%s]\n", digitalRead(PELTIER_PIN) ? "On" : "Off");
			DEBUGLOG("L        Light swap\t\t[%s]\n", digitalRead(LIGHT_PIN) ? "On" : "Off");
			DEBUGLOG("M        Manu, no PID\t\t[%s]\n", manu ? "On" : "Off");
			DEBUGLOG("t        Teleplot\t\t[%s]\n", teleplot ? "On" : "Off");
			DEBUGLOG("7-4      PID kP\t\t\t[%.3f]\n", pid.kp);
			DEBUGLOG("8-5      PID kI\t\t\t[%.3f]\n", pid.ki);
			DEBUGLOG("9-6      PID kD\t\t\t[%.3f]\n", pid.kd);
			displayMode();
	}
}

void sendToTeleplot() {
	static char buffer[300];
	static WiFiUDP udpPlot;

	sprintf(buffer, "temperature:%.2f\nhumidity:%.2f\ndewPoint:%.2f\ndewPoint2:%.2f\ntempCold:%.2f\ntempHot:%.2f\ntempOut:%.2f\nfanSpeed:%d\nTimerStop:%d\nPID_p:%.3f\nPID_i:%.3f\nPID_d:%.3f\nPID:%.3f\ndelta:%.3f\n",
			temperature,
			humidity,
			tempDewPoint,
			tempDewPoint2,
			tempCold,
			tempHot,
			tempOut,
			coldFanSpeed,
			(DELAY_STOP + memMillisStop - millis()) / 1000,
			pid.getP(),
			pid.getI(),
			pid.getD(),
			pid.getPID(),
			pid.getDelta());
	udpPlot.beginPacket(WiFi.broadcastIP(), PORTPLOT);
	udpPlot.print(buffer);
	udpPlot.endPacket();
}

void setup() {
	DEBUGINIT(onReceiveDebug);
	
	loadIP();
	myWifiBeginWPS();

	// Init pin
	pinMode(TEMPCOLD_PIN, INPUT);
	pinMode(TEMPHOT_PIN, INPUT);
	pinMode(TEMPOUT_PIN, INPUT);
	pinMode(HOTFAN_PIN, OUTPUT);
	pinMode(PELTIER_PIN, OUTPUT);
	pinMode(COLDFAN_PIN, OUTPUT);
	pinMode(LIGHT_PIN, OUTPUT);
	setLight(false);
	// ledcSetup(0, 25000, COLDFAN_RESOLUTION);
	// ledcAttachPin(COLDFAN_PIN, 0);
	digitalWrite(COLDFAN_PIN, false);

	Wire.setPins(SHT_SDA, SHT_SCL);
	if (sht3x.begin() != 0)
		DEBUGLOG("Failed to Initialize the chip....\n");
	if (!sht3x.softReset())
		DEBUGLOG("Failed to Reset the chip....\n");
	if (!sht3x.startPeriodicMode(sht3x.eMeasureFreq_10Hz, sht3x.eRepeatability_High))
		DEBUGLOG("Failed to enter the periodic mode\n");

	analogSetWidth(ADCRES);
	analogReadResolution(ADCRES);
	xTaskCreate(ADCRead,"ADCRead",2048,NULL,1,&pADCRead);

	ArduinoOTA.begin();

	DEBUGLOG("Setup complet\n");
	setMode(mode::REFRESH);
}

void loop() {
	uint8_t regul;
	// Read values

	// tempHot = readTemp(mvoltHot);
	// tempCold = readTemp(mvoltCold);
	// tempOut = readTemp(mvoltOut);
	DFRobot_SHT3x::sRHAndTemp_t data = sht3x.readTemperatureAndHumidity();
	if (data.ERR == 0) {
		humidity = FilterHumidity.updateEstimate(data.Humidity);
		temperature = data.TemperatureC;
		// tempDewPoint = FilterDewPoint.updateEstimate(sht3x.computeDewPoint(temperature, humidity));
		tempDewPoint = sht3x.computeDewPoint(temperature, humidity);
		tempDewPoint2 = sht3x.computeDewPoint(tempOut, humidity);
	} else {
		DEBUGLOG("Failed to read Data\n");
		setMode(mode::ERROR);
	}

	// Traitement du mode
	switch (activeMode) {
		case mode::IDLE:
			if (millis() - memMillisIDLEMode > DELAY_REFLOW)
				setMode(mode::FLOW);
			break;
		case mode::FLOW:
			if (millis() - memMillisFlowMode > DELAY_FLOW)
				setMode(mode::IDLE);
			break;
		case mode::MISTINESS:
			if (tempHot > TEMPHOT_MAX){  // Si TempHot trop chaud alors arrêt
				DEBUGLOG("TOO HOT !\n");
				setMode(mode::REFRESH);
			}

			// regul = pid.compute(tempDewPoint + DEWPOINT_OFFSET - tempOut);
			regul = pid.compute(tempDewPoint + DEWPOINT_OFFSET - tempCold);
			if (!manu)
				setColdFan(regul);

			if (!pid.getUnderCapacity() || manu)
				memMillisStop = millis();               // Si régul bien en dessous de la vitesse minimum
			if (millis() - memMillisStop > DELAY_STOP)  // si 2 min au dessus du DewPoint et vitesse au minimum alors arrêt
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
			DEBUGLOG("Pas de mode %u\n", activeMode);
			setMode(mode::IDLE);
			break;
	}
	if (activeMode != mode::MISTINESS)
		memMillisStop = millis();  // Reset Timer arrêt

	if (digitalRead(LIGHT_PIN)){
		if (millis() - memMillisLight > DELAY_LIGHT)
			setLight(false);
	}
	else
		memMillisLight = millis();


	if (teleplot)
		sendToTeleplot();
	sendToNR();
	ArduinoOTA.handle();

	delay(1000);
}
