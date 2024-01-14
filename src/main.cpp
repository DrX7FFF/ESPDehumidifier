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
IPAddress broadcastIP;

#define LIGHT_PIN 23
#define PELTIER_PIN 4
#define HOTFAN_PIN 16
#define COLDFAN_PIN 17
#define SHT_SDA 18
#define SHT_SCL 19
#define TEMPOUT_PIN 39
#define TEMPHOT_PIN 34   // GPIO 34 = A?, uses any valid Ax pin as you wish
#define TEMPCOLD_PIN 35  // GPIO 35 = A7, uses any valid Ax pin as you wish

#define R1 100000    // voltage divider resistor value
#define BETA 3950.0  // Beta value
#define T0 298.15    // Temperature in Kelvin for 25 degree Celsius
#define R0 100000    // Resistance of Thermistor at 25 degree Celsius 94kOhm
#define ADCMAX 4095  // Résolution MAX
#define VSS 3.3      // Tension MAX

#define COLDFAN_RESOLUTION 5                            // 6
#define COLDFAN_SPEEDMIN 2                              // 2 pour 9V 3 pour 8V  4 pour Resolution de 6
#define COLDFAN_SPEEDMAX (1 << COLDFAN_RESOLUTION) - 1  // 0x1F

#define TEMPHOT_MAX 54

#define DELAY_STOP 600000     // 180000	// 3 minutes (1.5 min pour le démarrage)
#define DELAY_FLOW 120000     // 5 minutes
#define DELAY_REFLOW 3600000  // 60 minutes

#define DEWPOINT_OFFSET -1

DFRobot_SHT3x sht3x;
PIDController pid(15, 0.1, 0, COLDFAN_SPEEDMIN, COLDFAN_SPEEDMAX);
SimpleKalmanFilter FilterHot(0.1, 0.01);
SimpleKalmanFilter FilterCold(0.1, 0.01);
SimpleKalmanFilter FilterOut(0.1, 0.05);
SimpleKalmanFilter FilterDewPoint(0.1, 0.05);
SimpleKalmanFilter FilterHumidity(0.2, 0.4);  // Très proche du réel et filtre les petites variations

uint8_t coldFanSpeed = 0;
bool pushJSON = false;
bool manu = false;

uint32_t memMillisStop = 0;
uint32_t memMillisFlowMode = 0;
uint32_t memMillisIDLEMode = 0;

float temperature;
float humidity;
float tempCold;
float tempHot;
float tempOut;
float tempDewPoint;

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

void sendToNR() {
	static char buffer[300];
	static WiFiUDP udpNR;
	static uint32_t memMillis = 0;

	if (millis() - memMillis > 300000 || pushJSON) {
		memMillis = millis();
		pushJSON = false;
		sprintf(buffer, "{\"temperature\":%.1f,\"humidity\":%.1f,\"absHumidity\":%.1f,\"dewPoint\":%.1f,\"tempCold\":%.1f,\"tempHot\":%.1f,\"tempOut\":%.1f,\"coldFanSpeed\":%d}\0",
				temperature,
				humidity,
				sht3x.computeAbsoluteHumidity(temperature, humidity),
				tempDewPoint,
				tempCold,
				tempHot,
				tempOut,
				coldFanSpeed);
		udpNR.beginPacket(broadcastIP, PORTNR);
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
	ledcWrite(0, coldFanSpeed);
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
			delay(1000);  // Attendre 1s que les fan démarrent

			memMillisFlowMode = millis();
			break;
		case mode::MISTINESS:
			setPeltier(true);
			setHotFan(true);
			setColdFan(COLDFAN_SPEEDMAX);
			delay(1000);  // Attendre 1s que les fan démarrent
			break;
		case mode::REFRESH:
			setPeltier(false);
			setHotFan(true);
			setColdFan(COLDFAN_SPEEDMAX);
			delay(1000);  // Attendre 1s que les fan démarrent
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
			pid.ki -= 0.1;
			DEBUGLOG("PID kI : %.3f\n", pid.ki);
			break;
		case '8':
			pid.ki += 0.1;
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
		case '?':
			DEBUGLOG("?        Help\n");
			DEBUGLOG("H        Hot fan swap\n");
			DEBUGLOG("P        Peltier swap\n");
			DEBUGLOG("J        Push JSON\n");
			DEBUGLOG("C        Cold fan swap\n");
			DEBUGLOG("+        Cold Up\n");
			DEBUGLOG("-        Cold Down\n");
			DEBUGLOG("L        Light switch\n");
			DEBUGLOG("0        IDLE Mode\n");
			DEBUGLOG("1        Mistiness Mode\n");
			DEBUGLOG("r        REFRESH Mode\n");
			DEBUGLOG("f        FLOW Mode\n");
			DEBUGLOG("I        Init sht3x\n");
			DEBUGLOG("O        Restart ESP\n");
			DEBUGLOG("M        Manu, no PID\n");
			DEBUGLOG("7-4      PID kP\n");
			DEBUGLOG("8-5      PID kI\n");
			DEBUGLOG("9-6      PID kD\n");
			DEBUGLOG("Peltier : %s\n", digitalRead(PELTIER_PIN) ? "On" : "Off");
			DEBUGLOG("Hot fan : %s\n", digitalRead(HOTFAN_PIN) ? "On" : "Off");
			DEBUGLOG("Cold speed : %d\n", coldFanSpeed);
			DEBUGLOG("PID kP : %.3f\n", pid.kp);
			DEBUGLOG("PID kI : %.3f\n", pid.ki);
			DEBUGLOG("PID kD : %.3f\n", pid.kd);
			DEBUGLOG("Manu : %s\n", manu ? "On" : "Off");
			displayMode();
	}
}

void sendToTeleplot() {
	static char buffer[300];
	static WiFiUDP udpPlot;

	sprintf(buffer, "temperature:%.2f\nhumidity:%.2f\ndewPoint:%.2f\ntempCold:%.2f\ntempHot:%.2f\ntempOut:%.2f\nfanSpeed:%d\nTimerStop:%d\nPID_p:%.3f\nPID_i:%.3f\nPID_d:%.3f\nPID:%.3f\ndelta:%.3f\n",
			temperature,
			humidity,
			tempDewPoint,
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
	pinMode(TEMPCOLD_PIN, INPUT);
	pinMode(TEMPHOT_PIN, INPUT);
	pinMode(TEMPOUT_PIN, INPUT);
	pinMode(HOTFAN_PIN, OUTPUT);
	pinMode(PELTIER_PIN, OUTPUT);
	pinMode(COLDFAN_PIN, OUTPUT);
	pinMode(LIGHT_PIN, OUTPUT);
	setLight(false);
	ledcSetup(0, 25000, COLDFAN_RESOLUTION);
	ledcAttachPin(COLDFAN_PIN, 0);

	Wire.setPins(SHT_SDA, SHT_SCL);
	if (sht3x.begin() != 0)
		DEBUGLOG("Failed to Initialize the chip....\n");
	if (!sht3x.softReset())
		DEBUGLOG("Failed to Reset the chip....\n");
	if (!sht3x.startPeriodicMode(sht3x.eMeasureFreq_10Hz, sht3x.eRepeatability_High))
		DEBUGLOG("Failed to enter the periodic mode\n");

	DEBUGLOG("Setup complet\n");
	setMode(mode::REFRESH);
}

void loop() {
	uint8_t regul;
	// Read values
	tempCold = FilterCold.updateEstimate(readTemp(TEMPCOLD_PIN));
	tempHot = FilterHot.updateEstimate(readTemp(TEMPHOT_PIN));
	tempOut = FilterOut.updateEstimate(readTemp(TEMPOUT_PIN));
	DFRobot_SHT3x::sRHAndTemp_t data = sht3x.readTemperatureAndHumidity();
	if (data.ERR == 0) {
		humidity = FilterHumidity.updateEstimate(data.Humidity);
		temperature = data.TemperatureC;
		tempDewPoint = FilterDewPoint.updateEstimate(sht3x.computeDewPoint(temperature, humidity));
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
			if (tempHot > TEMPHOT_MAX)  // Si TempHot trop chaud alors arrêt
				setMode(mode::REFRESH);

			regul = pid.compute(tempDewPoint + DEWPOINT_OFFSET - tempOut);
			if (!manu)
				setColdFan(regul);

			if ((regul >= COLDFAN_SPEEDMIN) || manu)
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
			setMode(mode::IDLE);
			break;
	}
	if (activeMode != mode::MISTINESS)
		memMillisStop = millis();  // Reset Timer arrêt

	sendToTeleplot();
	sendToNR();
	ArduinoOTA.handle();

	delay(1000);
}
