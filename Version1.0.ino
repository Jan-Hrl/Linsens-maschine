#include <LiquidCrystal_I2C.h>  //SDA = A4; SCL= A5
#include <Encoder.h>
#include <Tone.h>
#include <EEPROM.h>

// Hinweis: Ena+ und Ena- sind auf Masse zu legen (Motorsteuerung)

// Drehgeber (Encoder) zur Einstellung im Menü
Encoder myEnc(3, 4);  // Encoder-Pins: CLK = Pin 3, DT = Pin 4

// Taster zur Starten des Einstellmodus (langer Druck)
const int buttonPin = 5;

// Eingang zur Erkennung des Arbeits- oder Ruhemodus
const int Aktiv = A0;

// Pin für Frequenzsignal an Schrittmotor (über Timer 1)
const int StepMotorPin = 9;

const int maxStepsPerMeter = 500, minStepsPerMeter = 10;  // Schritte pro Meter (einstellbar), Standard: 250
const int EEPROM_ADDR_STEPS = 0;  // Speicherstelle 0 falls es nicht mehr gespeichert wird einfach auf einen anderen Speicherstelle gesetzt werden.
int StepsPerMeter;

double MeterPerSec = 0;               // Geschwindigkeit in m/s
Tone PulsGeber;                       // Objekterzeugung für Frequenz-Ausgabe (Ton/Schrittmotor)

const int freqRampe    = 100;         // Max. Erhöhung der Frequenz pro Rampenschritt
int RampenAufloesung   = 1000;        // Intervall in ms zwischen Frequenzanpassungen (Rampe)
double Geschwindigkeit = 8.0;         // Geschwindigkeit im Testlauf in km/h

// LCD-Display initialisieren (Adresse 0x27, 16 Zeichen, 2 Zeilen) SDA = A4, SCL = A5
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Variablen für Interrupt-basierte Geschwindigkeitsmessung
volatile unsigned long startTime = 0;
volatile unsigned long measuredTime = 0;
volatile int pulseCount = 0;
volatile bool measurementDone = false;

// Status für die LCD-Ausgabe
enum lcdAusgabe {
  hello = 0,
  arbeit,
  einstellung,
  testlauf
};

void setup() 
{
  Serial.begin(9600);
  pinMode(Aktiv, INPUT);
  pinMode(buttonPin, INPUT_PULLUP);        // Taster mit internem Pull-Up-Widerstand
  PulsGeber.begin(StepMotorPin);           // Frequenz-Ausgabe für Schrittmotor vorbereiten
  PulsGeber.stop();                        // Initial keine Frequenz ausgeben

  lcd.init();                              // LCD initialisieren
  lcd.backlight();                         // Hintergrundbeleuchtung aktivieren
  WriteLCD(hello, false);                  // Begrüßung anzeigen

  pinMode(2, INPUT);                       // Interrupt-Eingang (z. B. von Rad-Encoder)
  attachInterrupt(digitalPinToInterrupt(2), onPulse, RISING); // Interrupt bei steigender Flanke

  delay(1000);                             // Ladezeit simulieren
  EEPROM.get(EEPROM_ADDR_STEPS, StepsPerMeter);
  // Prüfen ob Wert gültig ist (50–1000)
  if (StepsPerMeter < minStepsPerMeter || StepsPerMeter > maxStepsPerMeter) 
    StepsPerMeter = minStepsPerMeter; // fallback
  WriteLCD(arbeit, false);                 // In Arbeitsmodus wechseln
}

void loop() 
{
  static double freq = 0;
  static bool WorkPos = false, WorkPosOld =false; 
  static int i = 0;
  static unsigned long stopChecker = 0;

  // Geschwindigkeit berechnen aus Impulsdauer
  if (measurementDone) 
  {
    stopChecker = millis();
    double duration = ((double)measuredTime)/1000; // Zeit in Millisekunden
    measurementDone = false;
    freq = 1.0/((duration/1000)/12.0);             // freqenz für eine Impuls 
    MeterPerSec = freq*(1.0/130.0);                // Geschwindigkeit in m/s (*3,6 für km/h)
    i++;
    if (i>10)                  // Display nur alle 10 Zyklen aktualisieren
    {                                    
      WriteLCD(arbeit,WorkPos);
      i = 0;
    }
  }
   else if (millis() - stopChecker >= 1000) 
   {
    MeterPerSec = 0; // Stillstand erkannt
  }
  
  int Positoionsabfrage = analogRead(Aktiv);
  if (Positoionsabfrage <= 550)       
  {
    //*************ArbeitsPosition*****************
    WorkPos=true;    
    if (MeterPerSec >= 0.1)    // Nur bei Fahrt über 0.5 m/s aktivieren
    {
      SchrittRampe(MeterPerSec*StepsPerMeter);
    }
    else
    {
      SchrittRampe(0);  // Zu langsam → stoppen
    }
  }
  else  //***************RuhePosition******************
  {
    WorkPos = false;
    SchrittRampe(0);
  }                       
  
  // Langer Tastendruck → Einstellmodus
  if(digitalRead(buttonPin) == LOW){
    unsigned long start = millis();
    do{
      if(millis() - start >= 3000)
      {
        Einstellmodus();
        WriteLCD(arbeit,WorkPos);
      }
    }while(digitalRead(buttonPin) == LOW);
  }

  // Nur bei Statusänderung erneut anzeigen
  if(WorkPos != WorkPosOld){
    WriteLCD(arbeit,WorkPos);
  }
  WorkPosOld = WorkPos;
}

// Interruptfunktion: misst Zeit für 13 Impulse vom Geschwindigkeitssensor
void onPulse() 
{
  pulseCount++;

  if (pulseCount == 1) {
    startTime = micros(); // Zeitstempel beim ersten Impuls
  }


  if (pulseCount == 13) {
    measuredTime = micros() - startTime;
    measurementDone = true;
    pulseCount = 0;
  }
}

// LCD-Ausgabe in verschiedenen Modi anzeigen
void WriteLCD(lcdAusgabe ausgabe, bool aktiv) 
{
  lcd.clear();
  switch (ausgabe) {
    case hello:
      lcd.setCursor(2, 0);                
      lcd.print("Hallo Baura!");
      break;
    case arbeit:
      lcd.setCursor(0, 0);
      lcd.print("Position: ");
      if (!aktiv) 
        lcd.print("Oben");
      else
        lcd.print("Unten");
      lcd.setCursor(0, 1);
      lcd.print(MeterPerSec*3.6, 2);  // m/s → km/h
      lcd.print("km/h");
      break;

    case einstellung:
      lcd.setCursor(0,0);
      lcd.print("Einstellung");
      lcd.setCursor(0,1);
      lcd.print(StepsPerMeter);
      lcd.print(" steps/m");
      break;
    case testlauf:
      lcd.setCursor(0,0);
      lcd.print("Testlauf");
      lcd.setCursor(0,1);
      lcd.print("Laufzeit: ");
      lcd.print((int)(333.3/(Geschwindigkeit/3.6)));  // Sekundendauer berechnen
      lcd.print("s");
      break;
    default:
      break;
  }
}

// Modus zur Einstellung der Schritte pro Meter mit Encoder
void Einstellmodus()
{
  WriteLCD(einstellung, false);
  unsigned long startEinstellung = millis();
  bool lastButtonState = LOW;
  bool buttonState =  LOW;

  do{
    long newPosition = myEnc.read() /2;
    static long oldPosition = -5;

     // Encoderbewegung auswerten
    if (newPosition != oldPosition) 
    {
      if(newPosition > oldPosition){
        StepsPerMeter = StepsPerMeter <= minStepsPerMeter ? minStepsPerMeter : StepsPerMeter - 5;
      }
      else if (newPosition < oldPosition) {
        StepsPerMeter = StepsPerMeter >= maxStepsPerMeter ? maxStepsPerMeter : StepsPerMeter + 5;
      }
      oldPosition = newPosition;
      WriteLCD(einstellung, false);
      startEinstellung = millis();  // Timeout zurücksetzen
    }

    // Tasterdruck startet Testlauf
    buttonState = digitalRead(buttonPin);
    if (buttonState == LOW && lastButtonState == HIGH) 
    {
      Testlauf(StepsPerMeter);
      WriteLCD(einstellung, false);
      startEinstellung = millis();
      delay(500);   // Entprellpause
    }
    
    lastButtonState = buttonState;

  }while(millis() - startEinstellung <= 10000);  // 4 Sekunden ohne Eingabe → zurück

  int gespeicherterWert;
  EEPROM.get(EEPROM_ADDR_STEPS, gespeicherterWert);
  if (gespeicherterWert != StepsPerMeter) 
    EEPROM.put(EEPROM_ADDR_STEPS, StepsPerMeter);
    
}

// Führt einen Testlauf mit konstanter Geschwindigkeit aus
void Testlauf(int StepsPerMeter)
{
  WriteLCD(einstellung, false);
  unsigned long laufzeit = (333.3/(Geschwindigkeit/3.6))*1000;          //in ms
  bool lastButtonState, ButtonState, weitermachen = HIGH;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Testlauf aktiv");
  lcd.setCursor(0, 1);
  lcd.print(Geschwindigkeit);
  lcd.print(" km/h");
  delay(700);

  unsigned long startTest = millis();

  do 
  {
    SchrittRampe((Geschwindigkeit/3.6)*StepsPerMeter);  // konstante Frequenz
    lastButtonState = ButtonState;
    ButtonState = digitalRead(buttonPin);

    if (lastButtonState && !ButtonState) 
      weitermachen = false;   // manueller Abbruch möglich
  }
  while (millis() - startTest <=  + laufzeit && weitermachen);

  SchrittRampe(0);  // Test beenden
}

// Sanfte Frequenzänderung (Rampe) zur Steuerung des Schrittmotors
void SchrittRampe(uint16_t sollFreq) 
{
  static unsigned long RampenMerker = millis()  ;
  static uint16_t istFreq = 0;

  if (sollFreq == 0) {
    PulsGeber.stop();
    istFreq = 0;
  }

   // Nur bei Ablauf des Rampenintervalls verändern
  if (millis() - RampenMerker >= RampenAufloesung) 
  {
    RampenMerker = millis();

    if (sollFreq != istFreq) 
    {
      // Frequenz langsam anpassen
      if (sollFreq > istFreq + freqRampe) 
        istFreq += freqRampe;
      else 
        istFreq = sollFreq;

      // Mindestfrequenz prüfen (100 Hz als Schwelle)
      if (istFreq >= 10) 
        PulsGeber.play(istFreq);
      else 
        PulsGeber.stop();
      
    }
  }
}