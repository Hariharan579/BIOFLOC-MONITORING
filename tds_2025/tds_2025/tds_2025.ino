#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "ESP_Wahaj.h"
#include "Wire.h"
#include <SimpleDHT.h>
#include <Adafruit_MCP3008.h>
#include <SoftwareSerial.h>

#define rxGPS D3
#define txGPS D4
SoftwareSerial ss(rxGPS, txGPS);

#define REPORTING_PERIOD_MS 1000
#define VREF 3.3
#define SCOUNT 30

Adafruit_MCP3008 adc;
LiquidCrystal_I2C lcd(0x27, 16, 2);

int pinDHT11 = D0;
SimpleDHT11 dht11(pinDHT11);

int pwm = 0;
float te, bpm, spo2, ph, tds, turbidity;
uint32_t tsLastReport = 0;

float getMedianNum(int bArray[], int iFilterLen) {
    int bTab[iFilterLen];
    for (byte i = 0; i < iFilterLen; i++)
        bTab[i] = bArray[i];
    int i, j, bTemp;
    for (j = 0; j < iFilterLen - 1; j++) {
        for (i = 0; i < iFilterLen - j - 1; i++) {
            if (bTab[i] > bTab[i + 1]) {
                bTemp = bTab[i];
                bTab[i] = bTab[i + 1];
                bTab[i + 1] = bTemp;
            }
        }
    }
    if ((iFilterLen & 1) > 0)
        bTemp = bTab[(iFilterLen - 1) / 2];
    else
        bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
    return bTemp;
}

void setup() {
    pinMode(A0, INPUT);
    adc.begin();
    Serial.begin(115200);
    ss.begin(9600);
    
    lcd.begin();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("SMART HYDRATION");
    lcd.setCursor(0, 1);
    lcd.print("MONITORING");
    
    start("Project", "12345678");
    delay(2000);
    lcd.clear();
}

void loop() {
    if (CheckNewReq() == 1) {
        // Read temperature from DHT11
        byte temperature = 0, humidity = 0;
        int err = dht11.read(&temperature, &humidity, NULL);
        if (err != SimpleDHTErrSuccess) {
            Serial.print("Read DHT11 failed, err=");
            Serial.println(err);
            delay(1000);
            return;
        }
        te = temperature;
        Serial.print(te);
        Serial.println(" *C");
        
        // Read TDS and Turbidity values from ADC
        int rawTDS = adc.readADC(0);
        int rawTurbidity = adc.readADC(1);
        
        float voltageTDS = (rawTDS * VREF) / 1023.0;
        float compensationCoefficient = 1.0 + 0.02 * (te - 25.0);
        float compensationVoltage = voltageTDS / compensationCoefficient;
        tds = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5;
        
        float voltageTurbidity = (rawTurbidity * 5.0) / 1023.0;
      
            turbidity = -1120.4 * (voltageTurbidity * voltageTurbidity) + 5742.3 * voltageTurbidity - 4353.8;
      
        
        Serial.print("TDS: ");
        Serial.println(tds);
        Serial.print("Turbidity: ");
        Serial.println(turbidity);
        
        // Read pH data from GPS module
        if (ss.available() > 0) {
            String incomingChar = ss.readStringUntil('\n');
            Serial.print("Received pH Data: ");
            Serial.println(incomingChar);
            int pHIndex = incomingChar.indexOf("PH:");
            if (pHIndex >= 0) {
                int commaIndex = incomingChar.indexOf(',', pHIndex);
                if (commaIndex >= 0) {
                    String pHValue = incomingChar.substring(pHIndex + 3, commaIndex);
                    pHValue.trim();
                    Serial.print("pH Value: ");
                    Serial.println(pHValue);
                    ph = pHValue.toFloat();
                }
            }
        }
        
        // Display values on LCD
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("TEMP: ");
        lcd.print(te);
        lcd.print("C");
        
        lcd.setCursor(0, 1);
        lcd.print("TDS: ");
        lcd.print(tds);
        
        delay(2000);
        
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("TURB: ");
        lcd.print(turbidity);
        
        lcd.setCursor(0, 1);
        lcd.print("PH: ");
        lcd.print(ph);
        
        delay(3000);
        
        // Send data
        String myString = String(te) + "-" + String(tds) + "-" + String(turbidity) + "-" + String(ph);
        returnThisStr(myString);
        delay(1000);
        
        String path = getPath();
        path.remove(0, 1);
        pwm = path.toInt();
    }
}
