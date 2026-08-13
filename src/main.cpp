//#define ARDUINO_MAIN
#include <Arduino.h>
#include <TMC2209.h>
#include <mydriver.h>
#include <ErriezSerialTerminal.h>


const unsigned long TIMEOUT_MS = 2*60*60*1000;
unsigned long last_active_time = 0;
bool debugMode = false;

String inputString = "";      // String to hold incoming serial data
bool stringComplete = false;  // Flag for when a full command is received

char newlineChar = '\r';
char delimiterChar = ' ';

int16_t target_voltage = 0;


MyDriver driver;
SerialTerminal term(newlineChar, delimiterChar);






void cmd_get_help()
{
  Serial.println("Available commands:");

  Serial.println("REV - reverse output)");
  Serial.println("ON - enable output)");
  Serial.println("ADC - enable ADC");
  Serial.println("SETV - set target mV");
  Serial.println("DBG - enable DBG");
  Serial.println("CAL - calibrate device");

}

void cmd_unrecognized(const char *command)
{
    Serial.print("Unknown command: ");
    Serial.println(command);
    Serial.println("Use HELP command to see available commands");
}


void set_timeout() {
    last_active_time = millis();
}

void timeout_handler() {
  if (millis() - last_active_time > TIMEOUT_MS) {
    last_active_time = millis();
  }
}

void cmd_set_echo()
{
  char* bool_str = term.getNext();
  if (bool_str == NULL)
  {
    Serial.println("ERROR: NO ARGUMENT");
    return;
  }
  if (atoi(bool_str)){
    term.setSerialEcho(true);
    Serial.println("OK");

  } 
  else
    term.setSerialEcho(false);
}

void cmd_set_rev()
{
  char* str = term.getNext();
  if (str == NULL)
  {
    Serial.println("ERROR: NO ARGUMENT");
    return;
  }
  if (atoi(str)){

    driver.outReverse(true);
    Serial.println("OUT REV = 1");
  } 
  else 
  {
    driver.outReverse(false);
    Serial.println("OUT REV = 0");
  }
}

void cmd_set_dbg()
{
  char* str = term.getNext();
  if (str == NULL)
  {
    Serial.println("ERROR: NO ARGUMENT");
    return;
  }
  if (atoi(str)){
    debugMode = true;
    Serial.println("DBG = 1");
  } 
  else 
  {
    debugMode = false;
    Serial.println("DBG = 0");
  }
}

void cmd_set_adc()
{
  char* str = term.getNext();
  if (str == NULL)
  {
    Serial.println("ERROR: NO ARGUMENT");
    return;
  }
  if (atoi(str)){

    driver.adcOnOff(true);
    Serial.println("ADC ON");
  } 
  else 
  {
    driver.adcOnOff(false);
    Serial.println("ADC OFF");
  }
}

void cmd_set_out()
{
  char* str = term.getNext();
  if (str == NULL)
  {
    Serial.println("ERROR: NO ARGUMENT");
    return;
  }
  if (atoi(str)){

    driver.outOnOff(true);
    Serial.println("OUT ON");
  } 
  else 
  {
    driver.outOnOff(false);
    Serial.println("OUT OFF");
  }
}

/*
Na początku:
1. wyłaczyc wyjście
2. właczyć adc
3. seekVoltage
3.1 jeżeli ujemne to REV
4. włączyć wyjście
5. seekVoltage
6. wyłączyć adc
*/

void cmd_set_Vout()
{
  char* str = term.getNext();
  if (str == NULL)
  {
    Serial.println("ERROR: NO ARGUMENT");
    return;
  }
  if (atoi(str)){

    driver.outOnOff(true);
    Serial.println("OUT ON");
  } 
  else 
  {
    driver.outOnOff(false);
    Serial.println("OUT OFF");
  }
}

void set_digipot_voltage(int16_t target_mV)
{
  driver.outOnOff(false); // disable output
  driver.adcOnOff(true); // enable adc
  // reverse output if necessary
  if (target_mV < 0) driver.outReverse(true);
  else driver.outReverse(false);
  uint16_t absVoltage = abs(target_mV);
  delay(1000);
  int16_t ret = driver.seekTarget(absVoltage);
  if (ret != -1)
  {
    driver.outOnOff(true);
    ret = driver.seekTarget(absVoltage, true);
    Serial.print("OK:");
    Serial.println(ret);
    return;
  }
  driver.outOnOff(false);
  Serial.println("ERR:0");
  return;
}



void cmd_set_target_voltage()
{
  char* voltage_str = term.getNext();
  if (voltage_str == NULL)
  {
    Serial.println("ERROR: NO ARGUMENT");
    return;
  }
  if (atoi(voltage_str))
  {
    target_voltage = atoi(voltage_str);
    set_digipot_voltage(target_voltage);
    Serial.println("OK");
  } 
  else {
    Serial.println("ERROR: INVALID ARGUMENT");
  }
}

void cmd_set_calibrate()
{
  //Serial.println("Calibration ...");
  driver.calibrate();
  Serial.println("OK");
}




void setup()
{
  // Debugowanie / Monitor portu przez wbudowany J-Link USB (P2.1 i P2.2)
  Serial.begin(115200);

  Serial.println("Hello");

  term.setDefaultHandler(cmd_unrecognized);
  term.addCommand("REV", cmd_set_rev);
  term.addCommand("ON", cmd_set_out);
  term.addCommand("ADC", cmd_set_adc);
  term.addCommand("SETV", cmd_set_target_voltage);
  term.addCommand("DBG", cmd_set_dbg);
  term.addCommand("CAL", cmd_set_calibrate);

  term.addCommand("SETECHO", cmd_set_echo);
  term.addCommand("HELP", cmd_get_help);
  
  term.setPostCommandHandler(set_timeout);
  term.setSerialEcho(false);

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);

  digitalWrite(LED1, LOW);

  VADC->GLOBICLASS[0] &= ~(0x1FU);
  
  // Ustawiamy maksymalny czas próbkowania (+31 cykli zegara ADC)
  VADC->GLOBICLASS[0] |= (0x1FU); 


  bool ret = driver.begin();
  Serial.print("Driver init: ");
  Serial.println(ret);
  Serial.flush();

  // PATLAS - consider if only calibrate over cmd
  ret = driver.calibrate();
  Serial.print("Driver calibration: ");
  Serial.println(ret);
 /////////////////

  /* TO REMOVE
    // pinMode(DIAG_PIN, INPUT_PULLUP);

    // pinMode(ADC_PIN, INPUT);
    // analogReadResolution(12);
   
   
  //   pinMode(ENABLE_PIN, OUTPUT);
  //   pinMode(OUT_EN_ON, OUTPUT);
  //   pinMode(OUT_EN_OFF, OUTPUT);
  //   pinMode(OUT_REV_ON, OUTPUT);
  //   pinMode(OUT_REV_OFF, OUTPUT);
  //   pinMode(ADC_EN_ON, OUTPUT);
  //   pinMode(ADC_EN_OFF, OUTPUT);

  // Serial.println("waiting ...");
  */

    set_timeout(); 
    while(!Serial.available()){
      timeout_handler();
    }

  Serial.println("Done");


}

void loop()
{
  timeout_handler();
  term.readSerial();

// if (digitalRead(DIAG_PIN) == HIGH) {
//   g_isStall = true;  // Zbocze narastające -> ustawiamy zmienną
//   digitalWrite(LED1, HIGH);
// } 
//  digitalToggle(LED2);
//  delay(500);
//  diagPin_isr(); // pseudo ISR
  if (debugMode)
  {
    Serial.print("Analog: ");
    Serial.println(driver.readAdcMv());
    delay(500);
  }
}
