/*
  Seeed_Arduino_LSM6DS3 - Accelerometer
  The circuit:
  - Seeedstudio XIAO nRF52840 Sense

  created 21 March 2026
  by Mario Koren  

1. Repository hinzufügen: Gehen Sie zu Datei > Voreinstellungen und fügen Sie die folgende URL bei "Zusätzliche Boardverwalter-URLs" ein:
   https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json

2.Board-Paket installieren:
    Navigieren Sie zu Werkzeuge > Board > Boardverwalter.
    Suchen Sie nach "Seeed nRF52".
    Installieren Sie das Paket Seeed nRF52 Boards (für Standard-Anwendungen) oder Seeed nRF52 mbed-enabled Boards (empfohlen für IMU- und Mikrofon-Nutzung).

3.Board auswählen: Wählen Sie unter Werkzeuge > Board > Seeed nRF52 Boards den Eintrag Seeed XIAO nRF52840 Sense aus.
######################################################################################################################

6-Achsen-IMU (LSM6DS3TR-C): Für die Nutzung der Bewegungsdaten wird die Seeed_Arduino_LSM6DS3 Bibliothek empfohlen.
######################################################################################################################

Pakeke nachinstallieren
sudo apt update
sudo apt install pipx
pipx ensurepath
pipx install adafruit-nrfutil
*/

#include <LSM6DS3.h>
#include <Wire.h>
#include <ArduinoBLE.h>
#include <math.h>
//#include <Adafruit_TinyUSB.h> //Wenn fehler mit Serial Referenz..

// Instanz für den Sensor (I2C)
LSM6DS3 IMU(I2C_MODE, 0x6A);

// Einzigartige IDs für deinen SmartHoop
BLEService hoopService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLEUnsignedIntCharacteristic schwungChar("19B10004-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEFloatCharacteristic kraftChar("19B10003-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);

// XIAO nRF52840 Sense RGB LED Pins
//Wichtig beim XIAO: Die interne LED ist "Active LOW". Das bedeutet: digitalWrite(pin, LOW) schaltet sie AN, und HIGH schaltet sie AUS.
const int LED_BLAU = LEDB;
const int LED_ROT = LEDR;
const int LED_GRUEN = LEDG;


// --- PARAMETER ---
const float REIFEN_MASSE_KG = 1.0;
// Schwellenwert für kombinierte Kraft (G-Kraft Vektor aus Taumeln + Fliehkraft)
const float COMBINED_THRESHOLD = 1.1;  // Wenn der Zähler zu sensibel ist, erhöhe den COMBINED_THRESHOLD schrittweise (z. B. auf 1.8 oder 2.0).
const unsigned long DEBOUNCE = 250;    // Mindestzeit für eine Umdrehung (ms) > max 170 RPM
const float MIN_RPM_THRESHOLD = 20.0;  // Mindestgeschwindigkeit zum Zählen

// --- VARIABLEN ---
int runden = 0;     // Zähler für die Gesamtzahl der Runden.
int schwuenge = 0;  // Beschleunigung (Impuls)
float rpm = 0.0;
unsigned long letzteZeit = 0;  // Zeitstempel der letzten Runde für die RPM-Berechnung
bool wasInTargetZone = false;
bool peakSperre = false;  // Verhindert, dass eine einzige Runde mehrfach gezählt wird
float gOffset = 1.0;      // Standardwert, wird in setup() kalibriert
float ax, ay, az;

void setup() {
  //----------------------------------Serial START ---------------------------------
  Serial.begin(115200);                                   //Startet die USB-Ausgabe zum PC
  unsigned long startVerzoegerung = millis();             // Kurze Pause für den Serial Monitor
  while (!Serial && millis() - startVerzoegerung < 3000)  // WICHTIG: Timeout für Serial, damit es auch am Akku startet
    ;

  //------------------------BLE START (Frühzeitig initialisieren)-------------------
  if (!BLE.begin()) {
    Serial.println("BLE Fehler!");
    while (1)
      ;
  }
  Serial.println("BLE Firmware geladen!");
  delay(100);

  //------------------------- Beschleunigungssensor Start------------------------------------
  // Initialisiert den Sensor und Wire1 intern in stellt sie samplerate auf 26Hz
  IMU.settings.accelSampleRate = 104;  // In Hz. Mögliche Werte: 13, 26, 52, 104, 208, 416, 833, 1666
  Serial.println("Starte IMU.begin()...");
  if (IMU.begin() != 0) {
    Serial.println("IMU Fehler, LSM6DS3 nicht gefunden!!");
    while (1)
      ;
  }
  Serial.println("IMU Firmware geladen!");
  delay(100);

  // -------------------------LED Pins als Ausgang definieren---------------------------------
  pinMode(LED_BLAU, OUTPUT);
  pinMode(LED_ROT, OUTPUT);
  pinMode(LED_GRUEN, OUTPUT);

  // Alle LEDs aus (HIGH = AUS beim XIAO)
  digitalWrite(LED_BLAU, HIGH);
  digitalWrite(LED_ROT, HIGH);
  digitalWrite(LED_GRUEN, HIGH);

  // -------------------- KALIBRIERUNG (Reifen ruhig halten!) ---------------------
  //Kalibrierung wenn der Reifen senkrecht hängt, ansonsten wir der Offset auf 1.0G gestellt 
  Serial.println("Kalibrierung auf Z-Achse...");
  // Blaue LED an zum Start der Kalibrierung
  digitalWrite(LED_BLAU, LOW); 
  float summeZ = 0;
  for (int i = 0; i < 50; i++) {
    summeZ += IMU.readFloatAccelZ();  // Wir messen die Schwerkraft auf Z, da der Reifen vertikal hängt
    delay(10);
  }
  gOffset = summeZ / 50.0;
  Serial.print("Offset auf Z erkannt: ");
  Serial.println(gOffset);

  // Falls der Offset negativ ist (Sensor über Kopf), machen wir ihn positiv
  gOffset = abs(gOffset);

  // Falls der Offset nahe 0 ist (Fehler beim Aufhängen), erzwinge 1.0
  if (gOffset < 0.5) {
    gOffset = 1.0;
    Serial.print("Offset angepasst da kleiner als 0.5: ");
    Serial.println(gOffset);
    digitalWrite(LED_BLAU, HIGH);
    digitalWrite(LED_ROT, LOW); // KORREKTUR: LED_ROT statt LED_RED
    delay(1000);
    digitalWrite(LED_ROT, HIGH);
    digitalWrite(LED_GRUEN, LOW); 
  }
  else {
     digitalWrite(LED_BLAU, HIGH); 
     digitalWrite(LED_GRUEN, LOW); // Grüne LED signalisiert: Kalibrierung erfolgreich!
  }
  

  //-------------------------- BLE Setup Android App zb.:nRF Connect-----------------------
  BLE.setLocalName("SmartHoop");
  BLE.setDeviceName("SmartHoop");  // Wichtig für manche Android-Versionen
  BLE.setAdvertisedService(hoopService);
  hoopService.addCharacteristic(schwungChar);
  hoopService.addCharacteristic(kraftChar);  // Datenfelder zum Service hinzufügen
  BLE.addService(hoopService);

  // ------------------------------Startwerte setzen----------------------------------------
  schwungChar.writeValue(0);  
  kraftChar.writeValue(0.0);

  //---------------------- Signal aussenden: "Ich bin bereit zum Koppeln"-------------------"
  BLE.advertise();
  Serial.println("Bluetooth aktiv. Suche nach 'SmartHoop' in nRF Connect.");

  Serial.println("Starte Datenausgabe...");  
}

void loop() {
  // ---------1. Bluetooth-Status pflegen (muss so oft wie möglich aufgerufen werden)-------
  BLE.poll();

  // -------------------------------- Aktuelle Zeit in Millisekunden -----------------------
  unsigned long jetzt = millis();
  
  //-------------------------- Beschleunigungs-Sensordaten auslesen ------------------------
  // DYNAMISCHE LOGIK:
  // Wir ziehen den gOffset (Schwerkraft) von der Y-Achse ab.
  // Dadurch ist der Wert im Stillstand nahe 0.0 statt 1.0.
  ay = IMU.readFloatAccelY();
  az = IMU.readFloatAccelZ();
  float dY = abs(ay - gOffset);  
  float dZ = abs(az);            

  // HYBRID-LOGIK: Es wird die Stärke der Bewegung aus
  // der Fliehkraft (Z) und dem Taumeln/Kippen (Y) berechnet.
  // Wir ignorieren X (Bewegung in Laufrichtung), um Rauschen zu minimieren.
  float kombiniertG = sqrt(dY * dY + dZ * dZ);

  static unsigned long letzteMessung = 0; 

  // --------------------------------- Hüftschwung-Zählung---------------------------------
  if (kombiniertG > COMBINED_THRESHOLD && !peakSperre) {
    //Wenn letzteZeit 0 ist, ist dies der allererste Schwung.
    // Wir starten nur den Timer, zählen aber noch keine Runde.
    if (letzteZeit == 0) {
      letzteZeit = jetzt;
      peakSperre = true;
      Serial.println("Messung gestartet...");
    } else {
      unsigned long dauer = jetzt - letzteZeit;
      if (dauer > DEBOUNCE) {
        float aktuelleRPM = 60000.0 / dauer;

        if (aktuelleRPM >= MIN_RPM_THRESHOLD) {
          schwuenge++;
          rpm = aktuelleRPM;
          schwungChar.writeValue(schwuenge);  
        }
        letzteZeit = jetzt;
        peakSperre = true;
      }
    }
  }

  // Hysterese zum Zurücksetzen der Sperre
  if (kombiniertG < (COMBINED_THRESHOLD - 0.3)) {
    peakSperre = false;
  }

  //------------------------- KRAFTBERECHNUNG (Bereinigte Zentripetalkraft)-----------------------------------
  float kraft_newton = REIFEN_MASSE_KG * (kombiniertG * 9.81);

  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 300) {
    if (millis() - letzteZeit > 3000) {  // Auto-Reset bei Stillstand (> 3 Sek)
      kraft_newton = 0.0;      
    }

    kraftChar.writeValue(kraft_newton);

    Serial.print("Schwuenge: ");
    Serial.print(schwuenge); 
    Serial.print(" | Gesamt G Kominiert: ");
    Serial.print(kombiniertG);
    Serial.print(" | Kraft: ");
    Serial.print(kraft_newton, 2);
    Serial.println(" N");
    lastUpdate = millis();
  }
}


/*
Um beide physikalischen Effekte (Taumelbewegung auf der Z-Achse und Hüftstöße/Fliehkraft auf der Y-Achse) gleichzeitig zu nutzen, wird es zu einem Gesamtbeschleunigungsvektor kombiniert.
Der Vorteil: Egal ob der Reifen eiert, kippt oder nur fest angestoßen wird – die resultierende Kraftspitze wird zuverlässig erfasst.

Vektorsumme (sqrt(ay*ay + az*az)): Diese mathematische Formel (Satz des Pythagoras) fasst die seitliche Kraft und das vertikale Wackeln zu einem einzigen Signalwert zusammen. Das macht das System extrem robust gegen verschiedene Hula-Hoop-Stile.
Kompensation: Wenn du den Reifen sehr flach hältst, reagiert das System primär auf die Y-Achse (Fliehkraft). Wenn der Reifen anfängt zu eiern, liefert die Z-Achse zusätzliche Daten, die den Peak verstärken.
Fehlertoleranz: Kleine Erschütterungen auf nur einer Achse lösen seltener Fehlzählungen aus, da die Kombination beider Achsen einen echten "Impuls" benötigt.
################################################################################################################################################################################

Was bewirkt die Kalibrierung?
  In den ersten 1-2 Sekunden nach dem Start (während die for-Schleife läuft) misst der Arduino 50-mal die Beschleunigung.
  Er berechnet den Durchschnittswert der Schwerkraft (gOffset), der auf die Y-Achse drückt.
  In der loop() wird dieser Wert von den Live-Daten abgezogen. Dadurch springt die Anzeige in nRF Connect im Stillstand sauber auf 0.00 N.

Wichtiger Hinweis: Du musst den Reifen beim Einschalten oder nach einem Reset für ca. 2 Sekunden ruhig halten, damit der Nullpunkt korrekt gesetzt wird.
################################################################################################################################################################################

Was bewirkt die 40-RPM-Sperre?
    Keine Geisterrunden: Wenn du den Reifen nur langsam in die Hand nimmst oder ihn langsam um die Hüfte "rollen" lässt, ohne wirklich zu schwingen, wird die Runde ignoriert.
    Präzision: Eine Umdrehung dauert bei 40 RPM genau 1,5 Sekunden. Alles, was länger dauert, wertet der Tracker als "nicht sportlich relevant".
    Erste Runde: Damit die Zeitmessung überhaupt starten kann, wird der allererste Impuls (nach dem Einschalten) immer akzeptiert, um letzteZeit zu setzen.
Probier es aus: Schwing den Reifen an. Er sollte erst ab einer gewissen Grundgeschwindigkeit anfangen zu zählen.
################################################################################################################################################################################
*/
