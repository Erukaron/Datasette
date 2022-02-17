/*
Datasette Interface
Version: 1.0
Datum: 24.04.2021

Dieses Programm ermöglicht das speichern und lesen von Daten auf einer Kasette. Die Datenspeicherung ist ähnlich zu der Datenspeicherung auf der Commodore Datasette (siehe unten).
Hierbei handelt es sich um ein Proof of Concept, entsprechend ist eine Datenspeicherung und ein Datenlesen nur über den seriellen Port via PC möglich.

Schaltplan:
  Schreiben:
    halfBridgePositivePin und halfBridgeNegativePin werden jeweils mit einem 460Ohm Widerstand auf 0V gezogen. 
    halfBridgePositivePin und der Widerstand wird mit dem Signal-Pin (zum Schreiben) des Kasettenrekorders verbunden.
    halfBridgeNegativePin und der Widerstand wird mit dem Masse-Pin (zum Schreiben) des Kasettenrekorders verbunden.
    halfBridgePositivePin: 460Ohm Widerstand gegen Arduino-Masse und Signal-Pin Kasettenrekorder
    halfBridgeNegativePin: 460Ohm Widerstand gegen Arduino-Masse und Masse-Pin Kasettenrekorder
  Lesen:
    Ein NPN BC547 wird mit dem Emitter gegen Masse geschaltet. Von der Basis geht ein 460Ohm Widerstand ebenfalls gegen Masse. Der Kollektor wird über einen 460Ohm Widerstand mit der 5V Leitung des Arduino bestromt.
    readPin (ein interruptfähiger Pin) wird mit dem Kollektor und dem Widerstand gegen 5V verbunden.
    Die Basis und der Widerstand gegen 0V wird mit dem Signal-Pin (zum Lesen) des Kasettenrekorders verbunden.
    Der Masse-Pin (zum Lesen) des Kasettenrekorders wird mit dem GND-Pin des Arduino verbunden.
    Emitter:    Arduino-Masse
    Basis:      460Ohm Widerstand gegen Arduino-Masse und Signal-Pin Kasettenrekorder
    Kollektor:  460Ohm Widerstand gegen Arduino-5V und readPin

Funktionsprinzip (Kurzfassung):
  Schreiben:
    Beim Schreiben wird ein Signal zwischen halfBridgePositivePin und halfBridgeNegativePin mit unterschiedlichen Pulslängen erzeugt.
    Es gibt Long Pulses, Medium Pulses und Short Pulses. Ein Bit wird durch zwei Pulse (Medium und Short) dargestellt. High Bits werden durch die Reihenfolge Medium, dann Short dargestellt, Low Bits durch die Reihenfolge Short, dann Medium.
    Dadurch hat ein Bit immer die gleiche Übertragungsdauer. Bytes werden mit dem MSB zuerst übertragen.
    Bevor ein Byte gesendet wird, wird ein New-Data-Marker gesendet. Dieser besteht aus einem Long gefolgt von einem Medium Puls.
    In Versuchen mit dem zu diesem Zeitpunkt verfügbaren Kasettenrekorder (24.04.2021) hat sich heraus gestellt, dass bevor Daten auf die Kasette geschrieben werden können, Der Schreibkopf des Kasettenrekorder vorbereitet werden muss. 
    Dies liegt möglicherweise an den Eigenschaften der Spule im Schreibkopf, aber ich hatte zu diesem Zeitpunkt keine Lust dieser Sache weiter auf den Grund zu gehen. 
    Bevor Daten geschrieben werden, wird der Kasettenrekorder durch den Leader vorbereitet. Der Leader besteht aus numberOfSyncBits (6000 haben sich als zuverlässig erwiesen) Short Pulses gefolgt von 2*Long Pulses Wartezeit.
  Lesen:
    Durch die Verschaltung werden die geschriebenen Signale beim Lesen invertiert vorliegen. Also entspricht eine fallende Flanke beim Lesen einer steigenden Flanke beim Schreiben.
    Zum Ermitteln der gelesenen Daten muss die Zeit zwischen zwei fallenden Flankenermittelt werden. Abhängig von den Zeiten kann dann folgende Entscheidung getroffen werden:
      Vorherige Flanke kurze Zeit her:
        Vorheriger Puls kurz: Leader-Sequenz, oder Start von einem 0 Bit
        Vorheriger Puls mittel: Bit 1 empfangen
        Sonst: Fehler
      Vorherige Flanke mittlere Zeit her:
        Vorheriger Puls kurz: Bit 0 empfangen
        Vorheriger Puls mittel: Start von einem 1 Bit
        Vorheriger Puls lang: Abschluss des New-Data-Marker
        Sonst: Fehler
      Vorherige Flanke lange Zeit her:
        Leader-Sequenz bereits gesendet: Start des New-Data-Marker
        Sonst: Fehler
      Vorherige Flanke längere Zeit her:
        Vorheriger Puls kurz: Abschluss des Leaders
        Sonst: Fehler
    Zum Lesen wird ein Interrupt verwendet, das bei der fallenden Flanke ausgeführt wird. Dieses führt die obige Überprüfung durch und setzten Variablen entsprechend.
    In der Leseschleife wird dauerhaft geprüft, ob ein neues Byte vom Interrupt bereit gestellt wurde, falls ja, wird dieses auf den Seriellen Stream geschrieben.
    Falls für transmissionOverTime µSekunden keine neuen Daten empfangen wurden, gilt die Übertragung als beendet.

Schreiben (Anwender):
  Zum Schreiben wird der Kasettenrekorder in den Aufnahme Modus geschaltet und die zu schreibenden Daten im Ascii Format an den Arduino via serieller Kommunikation übergeben.
  Bsp.: wDaten im Ascii Format
  Das w gibt den Schreibmodus an, die Daten stehen direkt dahinter und werden mit LF abgeschlossen.

Lesen (Anwender):
  Zum Lesen wird der Kasettenrekorder in den Abspiel Modus geschaltet und der Befehl r über die serielle Schnittstelle an den Arduino übergeben.




Beschreibung der Commodore Datasette:
https://noniq.at/0010/audiokassetten-als-datenspeicher/

Die Daten sind in einzelne Blöcke aufgeteilt. Jeder Block beginnt mit einem „Leader“, der noch keine Daten enthält, aber dem C64 erlaubt, sich mit dem Laufwerk zu synchronisieren. Anschließend folgen die eigentlichen Daten und eine kurze Pause, bevor der nächste Block folgt.

Wo in diesen Tönen stecken jetzt die eigentlichen Daten? Beim Schreiben erzeugt der C64 Rechteckwellen – sogenannte „Pulses“ – in drei unterschiedlichen Dauern bzw. Frequenzen

    Short Pulse: ca. 350 µs (2.860 Hz)
    Medium Pulse: ca. 510 µs (1.960 Hz)
    Long Pulse: ca. 670 µs (1.490 Hz)

Bei entsprechender hoher Zoomstufe sieht man diese Pulses im Audioeditor – die theoretischen Rechteckwellen sind in der Praxis ziemlich abgerundet, aber noch klar erkennbar:

Misst man die Zeit zwischen zwei Nulldurchgängen (fallende Flanke, also wenn das Signal „von oben“ durch die Nulllinie geht), dann ergeben sich die oben beschriebenen Pulsdauern
. Im folgenden Bild hab ich die ersten paar Pulses am Übergang von Leader zu Daten entsprechend beschriftet:

Der Leader besteht ausschließlich aus Short Pulses. Die eigentlichen Daten verwenden dann alle drei Varianten, wobei immer zwei aufeinanderfolgende Pulses gemeinsam betrachtet werden müssen:

    Long Pulse + Medium Pulse: New-Data-Marker (markiert den Beginn eines Bytes) 

    Short Pulse + Medium Pulse: Bit 0
    Medium Pulse + Short Pulse: Bit 1

Mit diesem Wissen lässt sich dann das erste Byte dekodieren:

Konkret ist dieses Byte (und auch jedes weitere) so aufgebaut:

    New-Data-Marker
    8 Datenbits in LSB-0-Reihenfolge
    1 Prüfbit: Wenn in den Datenbits eine gerade Anzahl von 1-Bits vorkommt, dann ist dieses Prüfbit 1, ansonsten ist es 0.

In unserem Beispiel ergeben sich also die Bits 10010001 und das zugehörige korrekte Prüfbit 0 (die Anzahl der 1-Bits ist ungerade). Wegen der LSB-0-Reihenfolge muss man die Bits von hinten lesen, die gewohnte Darstellung wäre also 10001001, was dezimal 137 bzw. hexadezimal 0x89 entspricht.

Dass das erste Byte diesen Wert hat, ist kein Zufall: Der erste Datenblock beginnt mit der Bytefolge 0x89 0x88 0x87 0x86 0x85 0x84 0x83 0x82 0x81, bevor die „wirklichen“ Daten kommen (auch hier gehts wieder um Synchronisation).
 */

#define VERBOSE_MODE
//#define DEBUG_READ_PULSE_TIMES // Zeigt Phasenzeiten an
//#define DEBUG_READ_PULSE_TIMES_ADJUSTED // Zeigt bereinigte Phasenzeiten an

// Schreibe-Pins
const int halfBridgePositivePin = 11;
const int halfBridgeNegativePin = 12;

// Lese-Pin
const int readPin = 3;

// Arbeits-Pin
const int operationInProgressPin = LED_BUILTIN;

// Anzahl der Leader-Pulses
const int numberOfSyncBits = 20000;

//    ___
//  __ ! __
// _________
// Bei Anpassung müssen die Error Schranken und waitTimeOnReadStartup neu ermittelt werden!
const float speedfactor = 2.25; // Geschwindigkeitsmultiplikator zum Lesen und Schreiben, beeinflusst die Pulszeiten (2,25 scheint recht zuverlässig zu sein -> Zum verlängern erhöhen, zum verringern vermindern)
const float errorModifierLow  = 0.03;
const float errorModifierHigh = 0.07;

// Pulszeiten Short Puls und Fehlerschranken (µs)
// Die Lese-Zeit ist doppelt so lang, wie die Schreibzeit, weil ein Puls aus einem HIGH-Signal der Pulszeit und einem LOW-Signal der Pulszeit besteht. Zum Lesen wird entsprechend die doppelte Pulszeit benötigt.
const unsigned long shortPulseWrite = 175*speedfactor;
const unsigned long shortPulse = shortPulseWrite*2;
const unsigned long shortPulseErrorShort = shortPulse * (1 - errorModifierLow);
const unsigned long shortPulseErrorLong  = shortPulse * (1 + errorModifierHigh);

// Pulszeiten Medium Puls und Fehlerschranken (µs)
// Die Lese-Zeit ist doppelt so lang, wie die Schreibzeit, weil ein Puls aus einem HIGH-Signal der Pulszeit und einem LOW-Signal der Pulszeit besteht. Zum Lesen wird entsprechend die doppelte Pulszeit benötigt.
const unsigned long mediumPulseWrite = 350*speedfactor;
const unsigned long mediumPulse = mediumPulseWrite*2;
const unsigned long mediumPulseErrorShort = mediumPulse * (1 - errorModifierLow / 2);
const unsigned long mediumPulseErrorLong  = mediumPulse * (1 + errorModifierHigh);

// Pulszeiten Medium Puls und Fehlerschranken (µs)
// Die Lese-Zeit ist doppelt so lang, wie die Schreibzeit, weil ein Puls aus einem HIGH-Signal der Pulszeit und einem LOW-Signal der Pulszeit besteht. Zum Lesen wird entsprechend die doppelte Pulszeit benötigt.
const unsigned long longPulseWrite = 700*speedfactor;
const unsigned long longPulse = longPulseWrite*2;
const unsigned long longPulseErrorShort = longPulse * (1 - errorModifierLow / 3);
const unsigned long longPulseErrorLong  = longPulse * (1 + errorModifierHigh / 2);

// Zeit bis Lesevorgang startet -> Normalisierung des Kasettenlaufs & Einschwingen des Leaders (ms)
//                                                                         µs->ms  Hälfte der Zeit warten
const unsigned long waitTimeOnReadStartup = numberOfSyncBits * shortPulse / 1000 / 2;

// Zeit bis zum Beenden der Übertragung beim Lesen, nachdem kein neues Byte empfangen wurde (ms)
const unsigned long transmissionOverTime = 500; 

// Typ zum Darstellen der vorherigen Pulslänge im Lese-Interrupt
enum PULSELENGTH
{
  EMPTY,
  SHORT,
  MEDIUM,
  LONG
};

volatile unsigned long interruptTimes[3] = {0, 0, 0}; // Zeitpunktarray das für jede Phase des Eingangssignals gefüllt wird
volatile bool intSynchronized = false; // Gibt an ob das Interrupt mit dem Signal synchronisiert ist -> Verarbeitung startet auf einem HIGH Byte
volatile bool lastIntFinished = true; // Gibt an, ob das letzte Signal vollständig ist (positive und negative Phase erhalten)
volatile enum PULSELENGTH lastPulseLength = EMPTY; // Die vorherige empfangene Pulslänge
volatile bool leaderFinished = false; // Gibt an, ob der Leader geendet ist
//                                                                                                                                           --New Data Marker--   --  Byte Wert 1  --
volatile bool takeInformation = false; // Gibt an, ob Informationen übernommen werden können, oder ob man sich in einer leer Phase befindet: 0 Long -> 1 Medium -> 2 Medium -> 3 Short
                                       // Eine leere Phase ist z.B. die von 1 -> 2 lastPulseLength wäre dann Medium, currentPulseLength wäre auch Medium, was ein unzulässiger Zustand ist, diese leere Phase muss also übersprungen werden
volatile unsigned long lastInterruptTime = 0; // Zeit (ms), zu der das Interrupt zum vorherigen Mal betreten wurde -> Wichtig zur Bestimmung ob alle Daten gelesen wurden
volatile bool newByte = false; // Gibt an, ob das Interrupt ein neues Byte bereit stellt
volatile int bitNumber = 7; // Gibt an, an welcher Bit-Position eines Bytes das Interrupt grade ist (von MSB zu LSB)
volatile char byteValue = 0; // Gibt den Byte-Wert des letzten empfangenen Bytes an (nur valide, falls newByte = true)

#ifdef DEBUG_READ_PULSE_TIMES
  const int maxPulseTimes = 500;
  volatile unsigned short pulseTimes[maxPulseTimes];
  volatile int currentPulseTime = 0;
#endif

void setup() {
  Serial.begin(1000000); // Möglichst schnelle Übertragung, damit zeitkritische Funktionen nicht zu lange warten müssen

  pinMode(halfBridgePositivePin, OUTPUT);
  pinMode(halfBridgeNegativePin, OUTPUT);
  pinMode(readPin, INPUT_PULLUP);
  pinMode(operationInProgressPin, OUTPUT);

  // Erstmal kein Schreibsignal setzen
  SignalNull();
  delay(100);

  #ifdef VERBOSE_MODE
    Serial.println("Time µs:low|act|high");
    Serial.print("Short:");
    Serial.print(shortPulseErrorShort);
    Serial.print("|");
    Serial.print(shortPulse);
    Serial.print("|");
    Serial.println(shortPulseErrorLong);
    Serial.print("Medium:");
    Serial.print(mediumPulseErrorShort);
    Serial.print("|");
    Serial.print(mediumPulse);
    Serial.print("|");
    Serial.println(mediumPulseErrorLong);
    Serial.print("Long:");
    Serial.print(longPulseErrorShort);
    Serial.print("|");
    Serial.print(longPulse);
    Serial.print("|");
    Serial.println(longPulseErrorLong);
    Serial.print("Leader bits:");
    Serial.println(numberOfSyncBits);
    Serial.print("Read wait ms:");
    Serial.println(waitTimeOnReadStartup);
    Serial.print("Transmission over ms:");
    Serial.println(transmissionOverTime);
    Serial.print("Avg. b/s:");
    Serial.println(1 / (((((float)(longPulse + mediumPulse)) + ((float)(shortPulse + mediumPulse)) * 8) / 8) / 1000000));
    Serial.println("r for read, w<Text> for write, c for clear.");
  #endif
}

// Gibt ein positives Signal zu Schreiben aus
void SignalPositive()
{
  digitalWrite(halfBridgeNegativePin, LOW);
  digitalWrite(halfBridgePositivePin, HIGH);
}

// Gibt ein negatives Signal zum Schreiben aus
void SignalNegative()
{
  digitalWrite(halfBridgePositivePin, LOW);
  digitalWrite(halfBridgeNegativePin, HIGH);
}

// Gibt ein null Signal zum Schreiben aus (beide Pins haben gleiches Potenzial)
void SignalNull()
{
  digitalWrite(halfBridgePositivePin, LOW);
  digitalWrite(halfBridgeNegativePin, LOW);
}

void loop() {
  // Dieser Code dient einem Funktionstest
  if (Serial.available())
  {
    char c = Serial.read();
    if (c == 'r')
    {
      Read();
    }
    if (c == 'w')
    {
      Write(Serial.readString());
    }
    if (c == 'c')
    {
      // Löschen des Tapes -> Dauersignal setzen bis ausschalten Arduino
      digitalWrite(halfBridgePositivePin, HIGH);
      digitalWrite(halfBridgeNegativePin, HIGH);

      Serial.println("Clearing mode... You need to restart the device when finished.");

      while (true){}
    }
    #ifdef VERBOSE_MODE
      Serial.println("r for read, w<Text> for write, c for clear.");
    #endif
  }
}

// Schreibt die als String übergebene Nachricht
void Write(String message)
{
  // Daten schreiben -> anzeigen
  digitalWrite(operationInProgressPin, HIGH);
  
  // Bevor Daten geschrieben werden, muss der Leader geschrieben werden
  WriteLeader();

  // Schreibe für jedes Byte in der Nachricht dieses Byte auf die Kasette
  for (int i = 0; i <= message.length(); i++)
  {
    WriteByte(message[i]);
  }

  // Nachdem die Daten geschrieben wurde, soll eine null-Spannung am Schreibeport liegen
  SignalNull();

  delay(transmissionOverTime * 2); // Warten, sodass beim überschreiben von Daten auf jeden Fall beim wieder auslesen genug Stille herrscht, um das Ende zu erkennen
  digitalWrite(operationInProgressPin, LOW);
  
  #ifdef VERBOSE_MODE
    Serial.println(" Written.");
  #endif
}

// Schreibt das als char übergebene Zeichen
void WriteByte(char message)
{
  // Bevor ein Byte geschrieben wird, muss ein New-Data-Marker geschrieben werden
  WriteNewDataMarker();

  // Gibt das zu speichernde Bit an
  int bitToStore = 0;
  // Das Byte wird von MSB zu LSB auf die Kasette geschrieben
  for (int j = 7; j >= 0; j--)
  {
    bitToStore = message & (1 << j); // Extrahieren des j-ten Bits
  
    if (bitToStore > 0) 
    {
      WriteOne();
    }
    else 
    {
      WriteZero();
    }
  }
}

// Schreibt den Leader
void WriteLeader()
{
  for (int i = 0; i < numberOfSyncBits; i++)
  {
    SignalPositive();
    delayMicroseconds(shortPulseWrite);
    SignalNegative();
    delayMicroseconds(shortPulseWrite);
  }
}

// Schreibt den New-Data-Marker
void WriteNewDataMarker()
{
  SignalPositive();
  delayMicroseconds(longPulseWrite);
  SignalNegative();
  delayMicroseconds(longPulseWrite);
  SignalPositive();
  delayMicroseconds(mediumPulseWrite);
  SignalNegative();
  delayMicroseconds(mediumPulseWrite);
}

// Schreibt ein 1-Bit
void WriteOne()
{
  SignalPositive();
  delayMicroseconds(mediumPulseWrite);
  SignalNegative();
  delayMicroseconds(mediumPulseWrite);
  SignalPositive();
  delayMicroseconds(shortPulseWrite);
  SignalNegative();
  delayMicroseconds(shortPulseWrite);
}

// Schreibt ein 0-Bit
void WriteZero()
{
  SignalPositive();
  delayMicroseconds(shortPulseWrite);
  SignalNegative();
  delayMicroseconds(shortPulseWrite);
  SignalPositive();
  delayMicroseconds(mediumPulseWrite);
  SignalNegative();
  delayMicroseconds(mediumPulseWrite);
}

// Liest Daten vom Kasettenrekorder und schreibt sie in den seriellen Stream
void Read()
{
  // Lesen starten...
  digitalWrite(operationInProgressPin, HIGH);

  // Auf normalisieren der Kasette und des Leaders warten...
  delay(waitTimeOnReadStartup);
    
  // Lesen-Interrupt starten
  attachInterrupt(digitalPinToInterrupt(readPin), DataIn, CHANGE);

  bool exec = true; // Schleife wird ausgeführt, bis exec = false
  bool readAtLeastOnce = false; // Gibt an, ob mindestens ein Byte empfangen wurde, denn erst danach kann die Schleife unterbrochen werden

  // Leseschleife - Prüft auf neue Bytes vom Interrupt und gibt diese aus
  while (exec) 
  {
    // Falls ein neues Byte gelesen -> ausgeben
    if (newByte)
    {
      Serial.write(byteValue); // Daten schreiben
      newByte = false; // Status zurücksetzen, damit beim nächsten Durchlauf (während das Interrupt möglicherweise noch die Daten aufbereitet) nicht das gleiche Zeichen erneut ausgegeben wird
      readAtLeastOnce = true;
    }

    // Falls mindestens einmal ein Byte erhalten, prüfen ob Übertragung zu Ende
    if (readAtLeastOnce)
    {
      // Errechne Zeit zwischen aktuell und letztem Aufruf des Interrupts (letztes Bit)
      // Beachte dabei Überlauf
      unsigned long lastIntCall = lastInterruptTime; // do not use volatile data
      unsigned long current = millis();
      unsigned long diff = 0;

      diff = current - lastIntCall;

      // Wenn transmissionOverTime erreicht, sind alle Daten empfangen worden
      if (diff > transmissionOverTime)
      {
        exec = false;
      }
    }
  }

  // Interrupt deaktivieren, damit es nicht zu inkonsistenten Zuständen kommt
  detachInterrupt(digitalPinToInterrupt(readPin));

  // Lesen beendet
  digitalWrite(operationInProgressPin, LOW);
}

// Aufgerufen, falls ein Übergang von einem HIGH zu einem LOW Signal oder LOW zu HIGH stattgefunden hat.
// Ermittelt die gesendeten Daten
void DataIn()
{
  unsigned long interruptStartTime = micros();
  lastInterruptTime = millis();
  
  bool high = digitalRead(readPin);
  if (high)
  {
    intSynchronized = true;
  }

  if (intSynchronized)
  {
    if (high)
    {
      if (lastIntFinished) // Only on first interrupt, handeled seperatly if there was a previous interrupt
      {
        // New marker
        interruptTimes[0] = interruptStartTime;
        lastIntFinished = false;
      }
      else
      {
        // Finish old marker
        interruptTimes[2] = interruptStartTime;
        lastIntFinished = true;
      }

      if (lastIntFinished)
      {
        unsigned long bitTime = interruptTimes[2] - interruptTimes[0];
        
#ifdef DEBUG_READ_PULSE_TIMES
#ifndef DEBUG_READ_PULSE_TIMES_ADJUSTED
        if (leaderFinished)
          ProcessPulseTime(bitTime);
#endif        
#endif

        // Pruefe Fehlerschranken
        if (   ( (bitTime > shortPulseErrorShort)  && (bitTime < shortPulseErrorLong)  )
            || ( (bitTime > mediumPulseErrorShort) && (bitTime < mediumPulseErrorLong) )
            || ( (bitTime > longPulseErrorShort)   && (bitTime < longPulseErrorLong)   ) )
        {
#ifdef DEBUG_READ_PULSE_TIMES
#ifdef DEBUG_READ_PULSE_TIMES_ADJUSTED
        if (leaderFinished)
          ProcessPulseTime(bitTime);
#endif        
#endif
          
          // Start next marker
          interruptTimes[0] = interruptStartTime; // End of one interrupt is the start of the next one
          lastIntFinished = false;
          
          enum PULSELENGTH currentPulseLength = EMPTY;
  
          if (bitTime < shortPulseErrorLong)
          {
            currentPulseLength = SHORT;
          }
          else if (bitTime < mediumPulseErrorLong)
          {
            currentPulseLength = MEDIUM;
          }
          else if (bitTime < longPulseErrorLong)
          {
            currentPulseLength = LONG;
          }
  
          if (!leaderFinished)
          {
            if (lastPulseLength == SHORT && currentPulseLength == LONG)
            {
              leaderFinished = true;
              takeInformation = true; // Next information is new data marker
            }
            else
            {
              // wait...
            } 
          }
          else
          {
            // Every other phase contains actual information, see table
            // current phase will be last of next interrupt call, but current phase is not the last phase of the next data
            // -> Skip one phase
            // Take information       Phase        information
            //                1   LONG  -> MED     New data marker
            //                0   MED   -> SHORT   NO DATA
            //                1   SHORT -> MED     data 0
            //                0   MED   -> MED     NO DATA
            //                1   MED   -> SHORT   data 1
            if (takeInformation)
            {
              if (lastPulseLength == LONG && currentPulseLength == MEDIUM)
              {
                // New data marker
                byteValue = 0;
                bitNumber = 7;
              }
              else if (lastPulseLength == SHORT && currentPulseLength == MEDIUM)
              {
                // 0 bit
                byteValue = byteValue | (0 << bitNumber--);
              }
              else if (lastPulseLength == MEDIUM && currentPulseLength == SHORT)
              {
                // 1 bit
                byteValue = byteValue | (1 << bitNumber--);
              }
  
              // Falls die Bitanzahl auf -1 gefallen ist, wurde ein Byte gelesen
              if (bitNumber == -1) newByte = true;
            }
            takeInformation = !takeInformation; // Wait one phase 
          }
          
          lastPulseLength = currentPulseLength;
        }
        else
        {
          lastIntFinished = false; // There was an error phase -> Last interrupt is not yet finished
        }
      }
    }
    else
    {
      // Transition time stamp
      interruptTimes[1] = interruptStartTime;
    }
  }
}

#ifdef DEBUG_READ_PULSE_TIMES
void ProcessPulseTime(unsigned short bitTime)
{
  detachInterrupt(digitalPinToInterrupt(readPin));

  if (currentPulseTime < maxPulseTimes)
  {
    // Add to array
    pulseTimes[currentPulseTime++] = bitTime;
  }
  else
  {
    // Display values
    for (int i = 0; i < maxPulseTimes; i++)
    {
      // If not in range
      if ( !(    ( (bitTime > shortPulseErrorShort)  && (bitTime < shortPulseErrorLong)  )
              || ( (bitTime > mediumPulseErrorShort) && (bitTime < mediumPulseErrorLong) )
              || ( (bitTime > longPulseErrorShort)   && (bitTime < longPulseErrorLong)   ) ) )
      {
        Serial.print("!!!");
      }

      Serial.println(pulseTimes[i]);
    }

    currentPulseTime = 0;
  }

  attachInterrupt(digitalPinToInterrupt(readPin), DataIn, CHANGE);
}
#endif
