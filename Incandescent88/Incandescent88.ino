// Librerie per la comunicazione i2c e il controllo dei driver pwm pca9685
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Configura i sei pca9685 sugli indirizzi i2c assegnati
Adafruit_PWMServoDriver pca0( 0x40);
Adafruit_PWMServoDriver pca1(0x41);
Adafruit_PWMServoDriver pca2(0x42 );
Adafruit_PWMServoDriver pca3(0x43);
Adafruit_PWMServoDriver pca4( 0x44);
Adafruit_PWMServoDriver pca5(0x45);

// Raccoglie i driver in un array per indirizzare ogni scheda tramite indice
Adafruit_PWMServoDriver* pca[6] = {
  &pca0, &pca1, &pca2, &pca3, &pca4, &pca5
};

// Parametri generali del sistema di illuminazione
const byte NUM_PCA = 6;
const byte NUM_LAMPS = 91;
const uint16_t STARTUP_LAMP_TEST_PWM = 3276;
const unsigned long STARTUP_LAMP_TEST_MS = 5000;

// Ingressi del selettore a quattro posizioni
const byte MODE_PIN_STANDBY =30;
const byte MODE_PIN_DETECTOR=31;
const byte MODE_PIN_PIANO = 32;
const byte MODE_PIN_ORGAN = 33;

// Stati operativi disponibili
enum Mode {
  MODE_STANDBY,
  MODE_DETECTOR,
  MODE_PIANO,
  MODE_ORGAN
};

// Gestisce la modalita attiva e il debounce del selettore
Mode currentMode = MODE_STANDBY;
Mode candidateMode = MODE_STANDBY;
unsigned long modeCandidateTime = 0;
const unsigned long MODE_DEBOUNCE_MS=30;

// Definisce l'estensione midi del pianoforte e il limite pwm delle note appena inferiore al massimo
const byte NUM_NOTES = 88;
const byte FIRST_NOTE = 21;
const byte LAST_NOTE = 108;
const uint16_t NOTE_PWM_MAX = 3686;

// Assegna le uscite dedicate a sustain e clip
const byte SUSTAIN_LAMP_1 = 88;
const byte SUSTAIN_LAMP_2 = 89;
const byte CLIP_LAMP = 90;
const uint16_t SUSTAIN_PWM = 2000;
const uint16_t CLIP_PWM = 2867;

// Parametri del vu meter pilotato direttamente da arduino
const byte VU_PIN = 7;
const byte VU_PWM_AT_3V = 180;

// Parametri di conversione dell'adc e ingressi analogici dal circuito envelope
const uint16_t ADC_REFERENCE_MV = 5000;
const uint16_t ADC_MAX = 1023;
const byte ADSR_LEFT_PIN = A0;
const byte ADSR_RIGHT_PIN = A1;
const unsigned long ADSR_SAMPLE_INTERVAL_US = 2500;
// Converte la soglia analogica di 3 v nel corrispondente valore adc
const uint16_t CLIP_THRESHOLD_ADC = (3000UL * 1023UL + 2500UL) / 5000UL;


// Parametri che regolano il minimo luminoso, l'attacco e la gestione finale della modalita detector
const uint16_t DETECTOR_MIN_CORRECTED_MV = 25;
const byte DETECTOR_MIN_PWM_PERCENT = 1;
// Converte la percentuale minima nel corrispondente valore pwm
const uint16_t DETECTOR_MIN_PWM = ((uint32_t)NOTE_PWM_MAX * DETECTOR_MIN_PWM_PERCENT + 50UL) / 100UL;

// Definisce le soglie raw applicate ai due ingressi appena letti in funzione del numero di note attive
const uint16_t DETECTOR_RAW_UNRELIABLE_LOW_MV = 50;
const uint16_t DETECTOR_RAW_UNRELIABLE_MID_MV = 75;
const uint16_t DETECTOR_RAW_UNRELIABLE_HIGH_MV = 100;

// Richiede due secondi continui sotto soglia prima di considerare non attendibile la coda nella modalita ADSR Detector
const unsigned long DETECTOR_RAW_UNRELIABLE_HOLD_MS = 2000;

// Imposta a 3,1 secondi la durata della coda delle note piu acute fino al floor dell'1 percento
const unsigned long DETECTOR_FINAL_FADE_MS = 3100;
const unsigned long DETECTOR_ATTACK_MS = 50;
// Aggiornamento delle lampade a 50 Hz
const unsigned long DETECTOR_LAMP_UPDATE_MS = 20;
unsigned long lastDetectorLampUpdate = 0;

// Tabelle usate per compensare la caduta del diodo bat43, dedotte dal dadasheet e poi ritoccate
const byte BAT43_LUT_SIZE = 20;
// Punti di riferimento
const uint16_t BAT43_LEVEL_MV[BAT43_LUT_SIZE] = {
  0, 10, 25, 50, 100, 150, 200, 300, 400, 500,
  750, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000
};
// Caduta di tensione associata a ciascun livello della tabella precedente
const uint16_t BAT43_DROP_MV[BAT43_LUT_SIZE] = {
  0, 70, 110, 145, 180, 195, 210, 230, 245, 255,
  270, 285, 305, 320, 330, 340, 351, 365, 380, 395
};

// Tabella che approssima la curva percettiva y=x^0.6 
const byte PERCEPTUAL_LUT_SIZE = 33;
const uint16_t PERCEPTUAL_INPUT[PERCEPTUAL_LUT_SIZE] = {
  0, 32, 64, 96, 128, 160, 192, 224, 256, 288, 320,
  352, 384, 416, 448, 480, 512, 544, 576, 608, 640, 672,
  704, 736, 768, 800, 832, 864, 896, 928, 960, 992, 1023
};
// Valori risultanti dall'applicazione approssimata della curva
const uint16_t PERCEPTUAL_OUTPUT[PERCEPTUAL_LUT_SIZE] = {
  0, 128, 194, 247, 294, 336, 375, 411, 446, 478, 509,
  539, 568, 596, 623, 650, 675, 700, 725, 749, 772, 795,
  818, 840, 861, 883, 904, 924, 945, 965, 985, 1004, 1023
};


//Casmpionamento
unsigned long lastAdsrSampleUs = 0;
// Ultimi valori grezzi letti direttamente dagli ingressi sx e dx
uint16_t detectorLeftRawADC = 0;
uint16_t detectorRightRawADC = 0;
// Memorizza gli stessi valori dopo la compensazione del bat43 e l'applicazione della curva percettiva
uint16_t detectorLeftProcessedADC = 0;
uint16_t detectorRightProcessedADC = 0;
// Valore minimo dell’envelope elaborato usato come estremo inferiore della mappatura
uint16_t detectorProcessedMinADC = 0;

// Stato con cui viene riconosciuta la parte finale non attendibile dell'envelope
bool detectorRawLowTiming = false;
// Memorizza l'istante in cui e iniziato il conteggio dei due secondi continui sotto soglia
unsigned long detectorRawLowStartMs = 0;
bool detectorFinalFadeActive = false;
// Istante di avvio della dissolvenza finale per calcolarne il progresso nel tempo
unsigned long detectorFinalFadeStartMs = 0;

bool clipState = false;


// Parametri della modalita piano, per simulare la natura percussiva del pianoforte, con diversi tempi di discesa di luminosità pitch dipendenti
const unsigned long HAMMER_DROP_MS = 220;
const byte HAMMER_REMAIN_PERCENT = 72;
const unsigned long BASS_DECAY_MS = 12000;
const unsigned long TREBLE_DECAY_MS = 3400;
const unsigned long BASS_EXTRA_DECAY_MS = 5100;
const byte DECAY_FLOOR_PERCENT = 1;

// Aggiornamento lampade a 50 Hz
const unsigned long PIANO_UPDATE_INTERVAL_MS = 20;

unsigned long lastPianoUpdate = 0;


// Intensità della risonanza simpatetica per ottave e quinte
const byte OCTAVE_RESONANCE_PERCENT = 9;
const byte FIFTH_RESONANCE_PERCENT = 6;


// Intervallo di luminosità usato nella modalita organ (40% min, 80% max)
const uint16_t ORGAN_PWM_MIN = 1638;
const uint16_t ORGAN_PWM_MAX = 3276;

// Array di stato delle note
// Tasti attivi dal midi
bool keyDown[NUM_NOTES];
//Tasti rilasciati ma mantenuti attivi dal pedale
bool sustained[NUM_NOTES];
// Indica se la nota è attualmente attiva e deve essere gestita
bool noteActive[NUM_NOTES];
// Valore pwm iniziale della nota determinato dalla velocity
uint16_t initialPWM[NUM_NOTES];
// Ultimo valore pwm assegnato alla lampada di ciascuna nota
uint16_t lastPWM[NUM_NOTES];
// Velocity midi con cui e stata attivata ciascuna nota
byte noteVelocity[NUM_NOTES];
// Istante di attivazione di ciascuna nota
unsigned long noteStartTime[NUM_NOTES];
// Livello di riferimento dell'envelope dopo compensazione bat43 e curva percettiva, letto dopo 50 ms dalla pressione di ciascuna nota (modalita detector)
uint16_t detectorReferenceADC[NUM_NOTES];
// Indica se il valore di riferimento dell'envelope è gia stato acquisito per ciascuna nota, nella modalita detector
bool detectorReferenceCaptured[NUM_NOTES];

// Memorizza per ogni nota la luminosità corrente se il valore di envelope è sotto soglia da cui deve iniziare la dissolvenza finale
uint16_t detectorFinalFadeStartPWM[NUM_NOTES];

// Stato pedale sustain
bool sustainPedal = false;

// Variabili lettura midi con running status
byte runningStatus = 0;
byte midiData1 = 0;
byte midiDataCount = 0;

// Converte l'indice globale della lampada nella relativa scheda e nel relativo canale pca9685
void setLamp(byte index, uint16_t pwm) {
  // Individua quale delle sei schede pca9685 controlla la lampada richiesta
  byte board = index / 16;
  // Individua il canale della scheda corrispondente alla lampada richiesta
  byte channel = index % 16;
  // Imposta sul canale individuato il valore pwm da generare
  pca[board]->setPin(channel, pwm);
}

// Applica il limite massimo alle lampade, imposta il valore e memorizza l'ultimo pwm inviato
void setPianoLamp(byte noteIndex, uint16_t pwm) {
  if ( pwm > NOTE_PWM_MAX) pwm = NOTE_PWM_MAX;
  setLamp(noteIndex,  pwm);
  lastPWM[noteIndex] = pwm;
}

// Converte la tensione in millivolt nel corrispondente valore adc a 10 bit
uint16_t millivoltsToADC(uint16_t mv) {
  if (mv >= ADC_REFERENCE_MV ) return ADC_MAX;
  return ((uint32_t)mv * ADC_MAX + ADC_REFERENCE_MV / 2) / ADC_REFERENCE_MV;
}
// Stima la caduta del bat43 interpolando linearmente i valori della tabella
uint16_t estimateBat43DropMV(uint16_t measuredMV) {
  if ( measuredMV == 0) return 0;
  // Oltre l'ultimo punto disponibile usa l'ultima caduta presente nella tabella
  if (measuredMV >= BAT43_LEVEL_MV[BAT43_LUT_SIZE - 1])
    return BAT43_DROP_MV[BAT43_LUT_SIZE - 1];
  for ( byte i = 0; i < BAT43_LUT_SIZE - 1; i++) {
    uint16_t x0 = BAT43_LEVEL_MV[i];
    uint16_t x1 = BAT43_LEVEL_MV[i + 1];
    // Verifica se il valore misurato si trova tra i due punti
    if (measuredMV >= x0 && measuredMV <= x1) {
      uint16_t y0 = BAT43_DROP_MV[i];
      uint16_t y1 = BAT43_DROP_MV[i + 1];
      // Calcola la posizione del valore misurato all'interno dell'intervallo
      uint32_t position = measuredMV - x0;
      // Calcola l'ampiezza dell'intervallo di tensione
      uint32_t spanX = x1 - x0;
      // Calcola la differenza tra i due valori di caduta
      uint32_t spanY = y1 - y0;
      // Interpola linearmente la caduta del diodo tra i due punti della tabella
      return y0 + (position * spanY) / spanX;
    }
  }
  // Valore 0 nel caso non venga trovato alcun intervallo valido
  return 0;
}

// Ricostruisce il livello precedente al bat43 e lo riporta nel dominio adc

uint16_t compensateBat43ADC(uint16_t rawADC) {
  if (rawADC == 0 ) return 0;
  // Converte il valore adc grezzo nella tensione effettivamente misurata
  uint32_t measuredMV = ((uint32_t)rawADC * ADC_REFERENCE_MV + 511UL) / ADC_MAX;
  // Stima la caduta introdotta dal bat43 a quella tensione
  uint16_t diodeDropMV = estimateBat43DropMV((uint16_t)measuredMV);
  // Somma la caduta stimata per ricostruire la tensione precedente al diodo
  uint32_t correctedMV = measuredMV + diodeDropMV;
  // Impedisce alla tensione corretta di superare il riferimento adc
  if (correctedMV > ADC_REFERENCE_MV)
    correctedMV = ADC_REFERENCE_MV;
  // Riconverte la tensione compensata nel dominio adc
  return millivoltsToADC((uint16_t)correctedMV);
}


// Applica la curva percettiva tramite interpolazione tra punti consecutivi della lut
uint16_t applyPerceptualCurve(uint16_t inputADC) {
  if (inputADC == 0) return 0;
  if (inputADC >= ADC_MAX) return ADC_MAX;
  // Individua l'intervallo della lut corrispondente al valore adc in ingresso
  byte index = inputADC / 32;
  // Protegge dall'accesso oltre l'ultimo elemento della tabella
  if (index >= PERCEPTUAL_LUT_SIZE - 1)
    return ADC_MAX;
  // Punto adc inferiore dell'intervallo
  uint16_t x0 = PERCEPTUAL_INPUT[index];
  // Punto adc superiore dell'intervallo
  uint16_t x1 = PERCEPTUAL_INPUT[index + 1];
  // Valore percettivo associato al punto inferiore
  uint16_t y0 = PERCEPTUAL_OUTPUT[index];
  // Valore percettivo associato al punto superiore
  uint16_t y1 = PERCEPTUAL_OUTPUT[index + 1];
  // Calcola la posizione dell'ingresso all'interno dell'intervallo
  uint32_t position = inputADC - x0;
  // Calcola l'ampiezza dell'intervallo adc
  uint32_t spanX = x1 - x0;
  // Calcola la differenza tra i valori percettivi estremi
  uint32_t spanY = y1 - y0;
  // Interpola linearmente il valore percettivo tra i due punti della lut
  return y0 + (position * spanY) / spanX;
}

// Elabora ogni canale applicando prima la compensazione bat43 e poi la curva percettiva
uint16_t processEnvelopeChannel(uint16_t rawADC) {
  // Compensa la caduta introdotta dal bat43
  uint16_t correctedADC = compensateBat43ADC(rawADC);
  // Applica la curva percettiva al valore compensato
  return applyPerceptualCurve( correctedADC);
}

// Azzera tutti gli stati associati alle note e al sustain
void clearNoteState() {
  // Scorre tutte le 88 note e azzera tutte le variabili associate a note ed envelope
  for (byte n = 0;  n < NUM_NOTES; n++) {
    keyDown[n] = false;
    sustained[n] = false;
    noteActive[n] = false;
    initialPWM[n] = 0;
    lastPWM[n] = 0;
    noteVelocity[n] = 0;
    noteStartTime[n] = 0;
    detectorReferenceADC[n] = 0;
    detectorReferenceCaptured[n] = false;
    detectorFinalFadeStartPWM[n] = 0;
  }
  sustainPedal = false;
}

// Spegne tutti i 96 canali dei pca9685
void allLampsOff() {
  for (byte board = 0; board < NUM_PCA; board++) {
    for (byte channel = 0; channel < 16; channel++) {
      pca[board]->setPin(channel, 0);
    }
  }
}


// All'avvio esegue un test di cinque secondi sulle 91 lampade
void startupLampTest() {
  for (byte lamp = 0; lamp < NUM_LAMPS; lamp++)
    setLamp( lamp, STARTUP_LAMP_TEST_PWM);
  delay(STARTUP_LAMP_TEST_MS);
  allLampsOff();
}

// Azzeramento delle variabili temporali e interruzione di eventuale dissolvenza finale
void resetDetectorFinalFade() {
  detectorRawLowTiming = false;
  detectorRawLowStartMs = 0;
  detectorFinalFadeActive = false;
  detectorFinalFadeStartMs = 0;
}

// Gestione dell'interferenza sul circuito envelope generato dalle lampade in pvm
void updateDetectorRawReliability() {
  unsigned long now = millis();
  // Conta solo le note attive o mantenute dal sustain
  byte activeDetectorNotes = 0;
  for (byte n = 0; n < NUM_NOTES; n++) {
    if (noteActive[n])
      activeDetectorNotes++;
  }
  // Adatta la soglia di tensione al numero di note premute
  uint16_t detectorRawUnreliableMV;
  // Fino a dieci note 50mv
  if (activeDetectorNotes <= 10)
    detectorRawUnreliableMV = DETECTOR_RAW_UNRELIABLE_LOW_MV;
  // Da undici a venti note 75mv
  else if (activeDetectorNotes <= 20)
    detectorRawUnreliableMV = DETECTOR_RAW_UNRELIABLE_MID_MV;
  // Oltre venti note 100mv
  else
    detectorRawUnreliableMV = DETECTOR_RAW_UNRELIABLE_HIGH_MV;
  // Converte la soglia scelta dai millivolt al corrispondente valore adc
  uint16_t detectorRawUnreliableADC =
    ((uint32_t)detectorRawUnreliableMV * ADC_MAX) / ADC_REFERENCE_MV;
  // Verifica che entrambi i canali raw siano contemporaneamente sotto soglia
  bool bothRawLow =
    detectorLeftRawADC <= detectorRawUnreliableADC &&
    detectorRightRawADC <= detectorRawUnreliableADC;
  // Un campione sopra soglia interrompe il conteggio o la dissolvenza e riabilita l'inseguimento dell'envelope
  if (!bothRawLow ) {
    // Ripristina lo stato normale del detector
    resetDetectorFinalFade();
    return;
  }
  // Durante la dissolvenza continua a verificare che entrambi i canali restino sotto soglia
  if ( detectorFinalFadeActive)
    return;
  // Avvia il conteggio sotto soglia e memorizza la luminosita corrente delle note attive
  if ( !detectorRawLowTiming) {
    // Segna l'inizio della fase sotto soglia
    detectorRawLowTiming = true;
    // Memorizza l'istante di ingresso sotto soglia
    detectorRawLowStartMs = now;
    // Scorre tutte le note per congelarne la luminosita corrente
    for (byte n = 0; n < NUM_NOTES; n++) {
      // Memorizza il pwm solo delle note attive
      if (noteActive[n])
        detectorFinalFadeStartPWM[n] = lastPWM[n];
    }
    return;
  }
  // Continua ad attendere finche non sono trascorsi due secondi continui sotto soglia
  if (now - detectorRawLowStartMs < DETECTOR_RAW_UNRELIABLE_HOLD_MS)
    return;
  // Dopo due secondi continui avvia la dissolvenza dal livello memorizzato
  detectorFinalFadeActive = true;
  // Memorizza l'istante di inizio della dissolvenza
  detectorFinalFadeStartMs = now;
}

// Riporta uscite e variabili dinamiche allo stato iniziale
void everythingOff() {
  allLampsOff();
  analogWrite(VU_PIN, 0);
  clearNoteState();
  detectorLeftRawADC = 0;
  detectorRightRawADC = 0;
  detectorLeftProcessedADC = 0;
  detectorRightProcessedADC = 0;
  resetDetectorFinalFade();
  clipState = false;
}

// Scarta i byte midi residui e azzera lo stato del parser
void flushMidi() {
  // Legge e scarta tutti i byte ancora presenti nel buffer seriale
  while (Serial1.available() > 0)
    Serial1.read();
  runningStatus = 0;
  midiDataCount = 0;
}

// Lettura selettore
Mode readModeRaw() {
  bool s0 = digitalRead(MODE_PIN_STANDBY) == LOW;
  bool s1 = digitalRead(MODE_PIN_DETECTOR) == LOW;
  bool s2 = digitalRead(MODE_PIN_PIANO) == LOW;
  bool s3 = digitalRead(MODE_PIN_ORGAN) == LOW;
  // Conta quante posizioni risultano contemporaneamente attive
  byte active = (byte)s0 + (byte)s1 + (byte)s2 + (byte)s3;
  // Se la lettura non identifica una sola posizione forza la modalita standby
  if (active != 1) return MODE_STANDBY;
  // Restituisce la modalita corrispondente alla posizione selezionata
  if (s0) return MODE_STANDBY;
  if (s1) return MODE_DETECTOR;
  if (s2) return MODE_PIANO;
  if (s3) return MODE_ORGAN;
  // Condizione di sicurezza nel caso nessuna delle condizioni precedenti venga soddisfatta
  return MODE_STANDBY;
}

// Esegue una transizione pulita tra modalita azzerando lo stato precedente
void enterMode(Mode newMode) {
  everythingOff();
  flushMidi();
  // Imposta la nuova modalita operativa
  currentMode = newMode;
  // Reinizializza il riferimento temporale del campionamento adsr
  lastAdsrSampleUs = micros();
  // Reinizializza il riferimento temporale di aggiornamento del detector
  lastDetectorLampUpdate = millis();
  // Reinizializza il riferimento temporale di aggiornamento del piano
  lastPianoUpdate = millis();
}

// Applica un debounce di 30 ms prima di confermare il cambio di modalita
void updateModeSelector() {
  // Legge la posizione corrente del selettore
  Mode raw = readModeRaw();
  // Se la posizione letta cambia avvia un nuovo tempo di verifica
  if (raw != candidateMode) {
    // Memorizza la nuova modalita candidata
    candidateMode = raw;
    // Memorizza l'istante del cambiamento
    modeCandidateTime = millis();
  }
  // Conferma la nuova modalita solo se resta stabile per almeno 30 ms
  if (candidateMode != currentMode &&
      millis() - modeCandidateTime >= MODE_DEBOUNCE_MS) {
    // Esegue la transizione alla modalita confermata
    enterMode( candidateMode);
  }
}

// Mappa linearmente la velocity midi sul massimo pwm previsto per il detector
uint16_t detectorVelocityToPWM(byte velocity) {
  if ( velocity == 0) return 0;
  // Converte linearmente la velocity 1-127 nel range 0-NOTE_PWM_MAX
  return ((uint32_t)velocity * NOTE_PWM_MAX) / 127UL;
}

// Calcolo pvm combinando una risposta lineare e una quadratica per la velocity della modalita ADSR Piano
uint16_t pianoVelocityToPWM(byte velocity) {
  if (velocity == 0) return 0;
  // Copia la velocity in una variabile a 32 bit per i calcoli
  uint32_t v = velocity;
  // Calcola una componente lineare della risposta
  uint32_t linear = (v * NOTE_PWM_MAX) / 127UL;
  // Calcola una componente quadratica piu sensibile alle velocity elevate
  uint32_t quadratic = (v * v * NOTE_PWM_MAX) / (127UL * 127UL);
  // Usa la media delle due componenti come pwm finale
  return (uint16_t)((linear + quadratic) / 2UL);
}

// Mappa la velocity midi nell'intervallo di luminosita definito per la modalita ADSR Organ
uint16_t organVelocityToPWM(byte velocity) {
  if (velocity == 0) return 0;
  // Calcola l'ampiezza dell'intervallo compreso tra minimo e massimo dell'organ
  uint32_t range = ORGAN_PWM_MAX - ORGAN_PWM_MIN;
  // Porta la velocity 1 al punto iniziale dell'intervallo
  uint32_t position = velocity - 1;
  // Interpola linearmente la velocity tra il 40 e l'80% del pwm massimo
  return ORGAN_PWM_MIN + (position * range) / 126UL;
}


// Calcola un tempo di decadimento molto piu lungo nei bassi e piu breve negli acuti con un contributo quadratico
unsigned long decayTimeForNote(byte n) {
  // Converte la posizione della nota lungo la tastiera in una scala da 0 a 1000
  uint32_t position = ((uint32_t)n * 1000UL) / (NUM_NOTES - 1);
  // Calcola la differenza tra il tempo base dei bassi e quello degli acuti
  uint32_t baseDifference = BASS_DECAY_MS - TREBLE_DECAY_MS;
  // Interpola il tempo base dal registro grave verso quello acuto
  uint32_t baseDecay = BASS_DECAY_MS - (baseDifference * position) / 1000UL;
  // Calcola quanto la nota appartiene alla zona grave della tastiera
  uint32_t bassAmount = 1000UL - position;
  // Applica una curva quadratica per concentrare l'estensione aggiuntiva soprattutto nei bassi
  uint32_t bassCurve = (bassAmount * bassAmount) / 1000UL;
  // Calcola il tempo aggiuntivo riservato alle note piu gravi
  uint32_t extraDecay = (BASS_EXTRA_DECAY_MS * bassCurve) / 1000UL;
  // Restituisce la somma tra decadimento base e decadimento aggiuntivo
  return baseDecay + extraDecay;
}

// Calcola il profilo luminoso della nota a partire dal tempo trascorso dalla note on
uint16_t calculatePianoDecayAtElapsed(byte n, unsigned long elapsed) {
  // Recupera il pwm iniziale associato alla velocity della nota
  uint16_t startPWM = initialPWM[n];
  if (startPWM == 0) return 0;
  // Calcola il pwm corrispondente al 72 percento del livello iniziale
  uint16_t hammerEndPWM =
    ((uint32_t)startPWM * HAMMER_REMAIN_PERCENT) / 100UL;
  // Prima fase del piano con discesa lineare fino al 72 percento in 220 ms
  if (elapsed < HAMMER_DROP_MS) {
    // Calcola la luminosita totale da ridurre durante la fase iniziale
    uint32_t difference = startPWM - hammerEndPWM;
    // Calcola la parte di riduzione raggiunta al tempo corrente
    uint32_t reduction = (difference * elapsed) / HAMMER_DROP_MS;
    // Restituisce la luminosita corrente della fase iniziale
    return startPWM - reduction;
  }
  // Calcola il tempo trascorso dopo la fine dell'hammer drop
  unsigned long decayElapsed = elapsed - HAMMER_DROP_MS;
  // Ricava la durata del decadimento in funzione della posizione della nota
  unsigned long decayTime = decayTimeForNote(n);
  // Calcola il floor luminoso come percentuale del pwm iniziale
  uint16_t floorPWM =
    ((uint32_t)startPWM * DECAY_FLOOR_PERCENT) / 100UL;
  if (floorPWM < 1)
    floorPWM = 1;
  // Una volta concluso il decadimento mantiene il floor luminoso
  if (decayElapsed >= decayTime)
    return floorPWM;
  // Seconda fase con decadimento cubico fino al floor luminoso
  // Calcola l'avanzamento normalizzato del decadimento da 0 a 1000
  uint32_t progress = ((uint32_t)decayElapsed * 1000UL) / decayTime;
  // Calcola la parte di decadimento ancora rimanente
  uint32_t remaining = 1000UL - progress;
  // Applica una curva cubica alla parte rimanente
  uint32_t curvedRemaining =
    (remaining * remaining * remaining) / 1000000UL;
  // Calcola l'intervallo compreso tra il livello dopo l'hammer drop e il floor
  uint32_t range = hammerEndPWM - floorPWM;
  // Ricava il pwm corrente lungo la curva cubica
  uint32_t pwm =
    floorPWM + (range * curvedRemaining) / 1000UL;
  // Restituisce il pwm calcolato
  return (uint16_t)pwm;
}

// Ricava il decadimento corrente usando il tempo assoluto di sistema
uint16_t calculatePianoDecay(byte n, unsigned long now) {
  // Calcola il tempo trascorso dalla note on e richiama il modello di decadimento
  return calculatePianoDecayAtElapsed(n,  now - noteStartTime[n]);
}

// Ricerca sulla curva di decadimento il tempo corrispondente a un pwm obiettivo
unsigned long findPianoElapsedForPWM(byte n, uint16_t targetPWM) {
  // Inizia la ricerca dalla fine dell'hammer drop
  unsigned long low = HAMMER_DROP_MS;
  // Imposta come limite superiore la fine completa del decadimento
  unsigned long high = HAMMER_DROP_MS + decayTimeForNote(n);
  // Ricerca binaria del punto della curva piu vicino al pwm richiesto
  while (low < high) {
    // Calcola il punto temporale centrale dell'intervallo corrente
    unsigned long mid = low + (high - low) / 2;
    // Calcola il pwm prodotto dalla curva in quel punto
    uint16_t pwm = calculatePianoDecayAtElapsed(n, mid);
    // Se il pwm e ancora troppo alto sposta la ricerca avanti nel tempo
    if (pwm > targetPWM)
      low = mid + 1;
    // Altrimenti restringe la ricerca alla parte precedente
    else
      high = mid;
  }
  // Memorizza il miglior tempo individuato dalla ricerca
  unsigned long best = low;
  // Confronta anche il millisecondo precedente per ridurre l'errore di quantizzazione
  if (best > HAMMER_DROP_MS) {
    // Calcola il pwm nel punto individuato
    uint16_t pwmA = calculatePianoDecayAtElapsed(n, best);
    // Calcola il pwm un millisecondo prima
    uint16_t pwmB = calculatePianoDecayAtElapsed(n, best - 1);
    // Calcola la distanza dal pwm obiettivo nel primo punto
    uint16_t diffA =
      (pwmA > targetPWM) ? pwmA - targetPWM : targetPWM - pwmA;
    // Calcola la distanza dal pwm obiettivo nel punto precedente
    uint16_t diffB =
      (pwmB > targetPWM) ? pwmB - targetPWM : targetPWM - pwmB;
    // Se il punto precedente e piu vicino lo utilizza come risultato
    if (diffB < diffA)
      best--;
  }
  // Restituisce il tempo della curva piu vicino al pwm richiesto
  return best;
}

// Aumento temporaneo della luminosita delle note attive, in rapporto di ottava o quinta
void applyPianoSympatheticResonance(byte triggerNote) {
  // Memorizza il tempo corrente
  unsigned long now = millis();
  // Scorre tutte le note per cercare quelle armonicamente correlate
  for (byte n = 0; n < NUM_NOTES; n++) {
    // Esclude la stessa nota che ha generato la risonanza
    if ( n == triggerNote) continue;
    // Esclude le note non attive
    if (!noteActive[n]) continue;
    // Esclude le note che non sono ne premute ne mantenute dal sustain
    if (!keyDown[n] && !sustained[n]) continue;
    // Calcola la distanza in semitoni dalla nota appena suonata
    int interval = abs((int)triggerNote - (int)n);
    // Esclude un eventuale intervallo nullo
    if (interval == 0) continue;
    // Considera solo relazioni armoniche di ottava e quinta
    // Inizializza a zero l'intensita della risonanza
    byte resonancePercent = 0;
    // Riconosce le relazioni di ottava
    if (interval % 12 == 0)
      resonancePercent = OCTAVE_RESONANCE_PERCENT;
    // Riconosce le relazioni di quinta
    else if (interval % 12 == 7)
      resonancePercent = FIFTH_RESONANCE_PERCENT;
    // Ignora tutte le altre relazioni intervallari
    if (resonancePercent == 0)
      continue;
    // Recupera il pwm iniziale della nota da rinforzare
    uint16_t startPWM = initialPWM[n];
    // Esclude note prive di luminosita iniziale
    if (startPWM == 0)
      continue;
    // Calcola il livello corrente della nota lungo il decadimento
    uint16_t currentPWM = calculatePianoDecay(n, now);
    // Imposta come limite massimo del rinforzo il livello successivo all'hammer drop
    uint16_t resonanceCeiling =
      ((uint32_t)startPWM * HAMMER_REMAIN_PERCENT) / 100UL;
    // Non rinforza note che si trovano gia sopra il limite previsto
    if (currentPWM >= resonanceCeiling)
      continue;
    // Il rinforzo dipende dalla luminosita gia persa durante il decadimento
    // Calcola quanta luminosita e stata persa rispetto al livello iniziale
    uint32_t lostBrightness = startPWM - currentPWM;
    // Calcola il rinforzo percentuale in funzione dell'intervallo armonico
    uint32_t boost =
      (lostBrightness * resonancePercent) / 100UL;
    if (boost == 0)
      continue;
    // Somma il rinforzo alla luminosita corrente
    uint32_t targetPWM = currentPWM + boost;
    // Limita il rinforzo al tetto previsto
    if (targetPWM > resonanceCeiling)
      targetPWM = resonanceCeiling;
    // Evita aggiornamenti che non aumenterebbero effettivamente la luminosita
    if (targetPWM <= currentPWM)
      continue;
    // Riposiziona il tempo della nota per far proseguire il decadimento dal nuovo livello
    // Ricerca sulla curva il tempo equivalente al nuovo pwm
    unsigned long newElapsed =
      findPianoElapsedForPWM(n, (uint16_t)targetPWM);
    // Riallinea il decadimento al livello rinforzato
    noteStartTime[n] = now - newElapsed;
    // Aggiorna la lampada con il nuovo livello
    setPianoLamp(n, (uint16_t)targetPWM);
  }
}

// Aggiorna le due lampade del sustain
void updateSustainLamps() {
  // Se il sustain e attivo usa il pwm previsto altrimenti zero
  uint16_t pwm = sustainPedal ? SUSTAIN_PWM : 0;
  setLamp(SUSTAIN_LAMP_1, pwm);
  setLamp(SUSTAIN_LAMP_2, pwm);
}

// Disattiva completamente una nota e cancella i dati associati
void stopNote(byte n) {
  keyDown[n] = false;
  sustained[n] = false;
  noteActive[n] = false;
  initialPWM[n] = 0;
  noteVelocity[n] = 0;
  detectorReferenceADC[n] = 0;
  detectorReferenceCaptured[n] = false;
  detectorFinalFadeStartPWM[n] = 0;
  setPianoLamp(n, 0);
}


// Gestisce una note on in modo diverso per ADSR Detector, ADSR Piano e ADSR Organ.
void noteOn(byte midiNote, byte velocity) {
  if (midiNote < FIRST_NOTE || midiNote > LAST_NOTE) {
    return;
  }
  // Converte la nota midi nell'indice 0-87 utilizzato dagli array
  byte n = midiNote - FIRST_NOTE;
  // Nel detector la velocity definisce il massimo e l'envelope controllera il decadimento
  if (currentMode == MODE_DETECTOR) {
    // Una nuova note on annulla un'eventuale fase finale gia avviata e riabilita la lettura dell'envelope
    resetDetectorFinalFade();
    // Converte la velocity nel pwm massimo della nota
    uint16_t pwm = detectorVelocityToPWM(velocity);
    // Segna il tasto come premuto
    keyDown[n] = true;
    // Elimina un eventuale precedente stato di sustain della stessa nota
    sustained[n] = false;
    // Segna la nota come attiva
    noteActive[n] = true;
    // Memorizza il pwm massimo determinato dalla velocity
    initialPWM[n] = pwm;
    // Memorizza la velocity midi
    noteVelocity[n] = velocity;
    // Memorizza l'istante della note on
    noteStartTime[n] = millis();
    // Azzera il precedente riferimento dell'envelope
    detectorReferenceADC[n] = 0;
    // Richiede una nuova acquisizione del riferimento dopo 50 ms
    detectorReferenceCaptured[n] = false;
    // Azzera un eventuale precedente livello di partenza della dissolvenza
    detectorFinalFadeStartPWM[n] = 0;
    // Accende immediatamente la lampada al livello determinato dalla velocity
    setPianoLamp(n, pwm);
  }
  // Nella modalità ADSR piano la velocity avvia il modello sintetico di decadimento e risonanza
  else if (currentMode == MODE_PIANO) {
    // Converte la velocity secondo la curva prevista per il piano
    uint16_t pwm = pianoVelocityToPWM(velocity);
    // Applica la risonanza simpatetica alle eventuali ottave e quinte gia attive
    applyPianoSympatheticResonance(n);
    // Segna il nuovo tasto come premuto
    keyDown[n] = true;
    // Elimina un eventuale precedente stato di sustain
    sustained[n] = false;
    // Segna la nota come attiva
    noteActive[n] = true;
    // Memorizza il pwm iniziale
    initialPWM[n] = pwm;
    // Memorizza la velocity
    noteVelocity[n] = velocity;
    // Memorizza l'istante da cui calcolare il decadimento
    noteStartTime[n] = millis();
    // Accende la lampada al pwm iniziale
    setPianoLamp(n, pwm);
  }
  // Nell'organ la luminosita resta costante fino alla note off o al rilascio del sustain
  else if (currentMode == MODE_ORGAN) {
    // Converte la velocity nell'intervallo luminoso previsto per l'organ
    uint16_t pwm = organVelocityToPWM(velocity);
    keyDown[n] = true;
    sustained[n] = false;
    noteActive[n] = true;
    initialPWM[n] = pwm;
    noteVelocity[n] = velocity;
    setPianoLamp(n, pwm);
  }
}


// Gestione del note off distinguendo tra rilascio del tasto e mantenimento tramite sustain
void noteOff(byte midiNote) {
  if (midiNote < FIRST_NOTE || midiNote > LAST_NOTE) {
    return;
  }
  // Converte il numero midi nell'indice della nota
  byte n = midiNote - FIRST_NOTE;
  // Segna il tasto come fisicamente rilasciato
  keyDown[n] = false;
  // Se il sustain e premuto mantiene la nota attiva
  if (sustainPedal)
    sustained[n] = true;
  // Altrimenti disattiva completamente la nota
  else
    stopNote(n);
}
// Aggiorna il sustain e spegne le note mantenute quando il pedale viene rilasciato
void setSustain(bool state) {
  // Ignora il sustain quando il sistema non si trova in una modalita musicale
  if (currentMode != MODE_DETECTOR &&
      currentMode != MODE_PIANO &&
      currentMode != MODE_ORGAN) {
    return;
  }
  // Evita elaborazioni se lo stato del pedale non e cambiato
  if (state == sustainPedal)
    return;
  // Memorizza il nuovo stato del sustain
  sustainPedal = state;
  // Aggiorna le due lampade dedicate al pedale
  updateSustainLamps();
  // Al rilascio del pedale vengono chiuse solo le note non piu fisicamente premute
  if (!sustainPedal) {
    // Scorre tutte le note
    for (byte n = 0; n < NUM_NOTES; n++) {
      // Individua le note mantenute dal sustain ma con tasto gia rilasciato
      if (sustained[n] && !keyDown[n]) {
        // Elimina lo stato di sustain della nota
        sustained[n] = false;
        // Segna la nota come non attiva
        noteActive[n] = false;
        // Azzera il pwm iniziale
        initialPWM[n] = 0;
        // Azzera la velocity memorizzata
        noteVelocity[n] = 0;
        // Azzera il riferimento del detector
        detectorReferenceADC[n] = 0;
        // Segna il riferimento come non acquisito
        detectorReferenceCaptured[n] = false;
        // Azzera il livello memorizzato per la dissolvenza finale
        detectorFinalFadeStartPWM[n] = 0;
        // Spegne la lampada della nota
        setPianoLamp(n, 0);
      }
    }
  }
}

// Aggiornamento a intervalli regolari del decadimento delle note attive nella modalita ADSR Piano
void updatePianoDecay() {
  // Memorizza il tempo corrente
  unsigned long now = millis();
  // Attende l'intervallo previsto tra due aggiornamenti successivi
  if (now - lastPianoUpdate < PIANO_UPDATE_INTERVAL_MS) {
    return;
  }
  // Memorizza l'istante dell'aggiornamento appena avviato
  lastPianoUpdate = now;
  // Scorre tutte le note
  for (byte n = 0; n < NUM_NOTES; n++) {
    // Ignora le note non attive
    if (!noteActive[n])
      continue;
    // Aggiorna soltanto note premute o mantenute dal sustain
    if (keyDown[n] || sustained[n]) {
      // Calcola il pwm corrente lungo il decadimento
      uint16_t pwm = calculatePianoDecay(n, now);
      // Aggiorna la lampada soltanto se il valore e cambiato
      if (pwm != lastPWM[n])
        setPianoLamp(n, pwm);
    }
  }
}

// Distribuisce i due canali dell'envelope lungo la tastiera con pesatura stereo progressiva
uint16_t detectorEnvelopeForNote(byte n) {
  // Gli estremi della tastiera usano un solo canale mentre le note interne sono interpolate (la nota piu bassa usa esclusivamente il canale sinistro)
  if (n == 0)
    return detectorLeftProcessedADC;
  // La nota piu alta usa esclusivamente il canale destro
  if (n == NUM_NOTES - 1)
    return detectorRightProcessedADC;
  // Calcola il peso del canale sinistro in funzione della posizione della nota
  uint32_t leftWeight = NUM_NOTES - n;
  // Calcola il peso del canale destro in funzione della posizione della nota
  uint32_t rightWeight = n;
  // Somma i due envelope moltiplicati per i rispettivi pesi
  uint32_t weighted =
    ((uint32_t)detectorLeftProcessedADC * leftWeight) +
    ((uint32_t)detectorRightProcessedADC * rightWeight);
  // Normalizza il risultato rispetto al numero complessivo delle note
  weighted /= NUM_NOTES;
  // Restituisce l'envelope stereo interpolato per quella nota
  return (uint16_t)weighted;
}

// Acquisisce per ogni nota il riferimento dell'envelope dopo i primi 50 ms
void captureDetectorReferences() {
  // Memorizza il tempo corrente
  unsigned long now = millis();
  // Scorre tutte le note
  for (byte n = 0; n < NUM_NOTES; n++) {
    // Ignora le note non attive
    if (!noteActive[n])
      continue;
    // Ignora le note che possiedono gia un riferimento acquisito
    if (detectorReferenceCaptured[n])
      continue;
    // Calcola il tempo trascorso dalla note on
    unsigned long age = now - noteStartTime[n];
    // Attende 50 ms dalla note on prima di fissare il riferimento della singola nota
    if (age < DETECTOR_ATTACK_MS)
      continue;
    // Legge il valore dell'envelope interpolato nella posizione della nota
    uint16_t reference = detectorEnvelopeForNote(n);
    // Impedisce al riferimento di scendere sotto il minimo elaborato previsto
    if (reference < detectorProcessedMinADC)
      reference = detectorProcessedMinADC;
    // Memorizza il riferimento definitivo della nota
    detectorReferenceADC[n] = reference;
    // Segna il riferimento come acquisito
    detectorReferenceCaptured[n] = true;
  }
}

// Campionamento dei due ingressi analogici a 400 hz e aggiornamento vu meter e lampada clip
bool updateAdsrDetector() {
  // Legge il tempo corrente in microsecondi
  unsigned long nowUs = micros();
  // Esegue una nuova acquisizione solo dopo l'intervallo di 2500 microsecondi
  if ((unsigned long)(nowUs - lastAdsrSampleUs) < ADSR_SAMPLE_INTERVAL_US) {
    return false;
  }
  // Memorizza l'istante del nuovo campionamento
  lastAdsrSampleUs = nowUs;
  // La prima lettura di ogni canale viene scartata per favorire l'assestamento dell'adc
  analogRead(ADSR_LEFT_PIN);
  // Acquisisce il valore raw effettivamente utilizzato del canale sinistro
  detectorLeftRawADC = analogRead(ADSR_LEFT_PIN);
  // Esegue una lettura preliminare del canale destro
  analogRead(ADSR_RIGHT_PIN);
  // Acquisisce il valore raw effettivamente utilizzato del canale destro
  detectorRightRawADC = analogRead(ADSR_RIGHT_PIN);
  // La validita della coda viene valutata sui campioni raw appena acquisiti e prima delle compensazioni
  updateDetectorRawReliability();
  // Applica compensazione bat43 e curva percettiva ai campioni utili del canale sinistro
  detectorLeftProcessedADC =
    processEnvelopeChannel(detectorLeftRawADC);
  // Applica compensazione bat43 e curva percettiva ai campioni utili del canale destro 
  detectorRightProcessedADC =
    processEnvelopeChannel(detectorRightRawADC);
  // Vu meter e clip seguono il massimo dei due valori grezzi senza correzioni
  // Seleziona il maggiore tra il valore raw sinistro e destro
  uint16_t envelopeMaxRaw =
    (detectorLeftRawADC > detectorRightRawADC)
    ? detectorLeftRawADC
    : detectorRightRawADC;
  // Variabile pwm destinata al vu meter
  uint8_t vuPWM;
  // A partire dalla soglia di 3 v porta il vu meter al livello massimo previsto
  if (envelopeMaxRaw >= CLIP_THRESHOLD_ADC)
    vuPWM = VU_PWM_AT_3V;
  // Sotto i 3 v scala linearmente il vu meter in funzione del segnale raw
  else
    vuPWM =
      ((uint32_t)envelopeMaxRaw * VU_PWM_AT_3V) /
      CLIP_THRESHOLD_ADC;
  // Aggiorna direttamente il pwm del vu meter
  analogWrite(VU_PIN, vuPWM);
  // Determina se il segnale raw ha superato la soglia di clipping
  bool newClipState =
    envelopeMaxRaw > CLIP_THRESHOLD_ADC;
  // Aggiorna la lampada di clip soltanto quando cambia lo stato
  if (newClipState != clipState) {
    // Memorizza il nuovo stato di clipping
    clipState = newClipState;
    // Accende o spegne la lampada dedicata
    setLamp(
      CLIP_LAMP,
      clipState ? CLIP_PWM : 0
    );
  }
  // Segnala che e stato eseguito un nuovo campionamento completo
  return true;
}

// Conversione dell'envelope elaborato nel pwm delle note attive nella modalita ADSR Detector
void updateDetectorLamps() {
  // Memorizza il tempo corrente
  unsigned long now = millis();
  // Limita l'aggiornamento delle lampade alla frequenza prevista
  if (now - lastDetectorLampUpdate < DETECTOR_LAMP_UPDATE_MS) {
    return;
  }
  // Memorizza l'istante dell'aggiornamento corrente
  lastDetectorLampUpdate = now;
  // Scorre tutte le note
  for (byte n = 0; n < NUM_NOTES; n++) {
    // Ignora le note non attive
    if (!noteActive[n])
      continue;
    // Mantiene la luminosita iniziale per i primi 50 ms dalla note on e calcola il tempo trascorso dalla note on
    unsigned long age = now - noteStartTime[n];
    // Durante i primi 50 ms non modifica il pwm iniziale
    if (age < DETECTOR_ATTACK_MS)
      continue;
    // Attende che sia stato acquisito il riferimento dell'envelope
    if (!detectorReferenceCaptured[n])
      continue;
    // La velocity imposta il massimo mentre il floor definisce il minimo della nota
    uint16_t velocityCap = initialPWM[n];
    uint16_t floorPWM = DETECTOR_MIN_PWM;
    // Impedisce al floor di superare il pwm massimo delle velocity molto basse
    if (floorPWM > velocityCap)
      floorPWM = velocityCap;
    // Variabile del pwm definitivo della nota
    uint16_t finalPWM;
    // Dopo due secondi sotto soglia applica una coda cubica con durata crescente verso le note basse similmente alla modalita piano
    if (detectorFinalFadeActive) {
      // Recupera la luminosita congelata all'inizio della fase sotto soglia
      uint16_t fadeStartPWM = detectorFinalFadeStartPWM[n];
      // Impedisce alla dissolvenza di partire sotto il floor
      if (fadeStartPWM < floorPWM)
        fadeStartPWM = floorPWM;
      // Calcola il tempo trascorso dall'inizio della dissolvenza finale
      unsigned long fadeElapsed = now - detectorFinalFadeStartMs;
      // Calcola la durata pitch dipendente
      unsigned long fadeTime =
        ((uint32_t)decayTimeForNote(n) * DETECTOR_FINAL_FADE_MS) / TREBLE_DECAY_MS;
      // Se la dissolvenza e terminata mantiene direttamente il floor
      if (fadeElapsed >= fadeTime) {
        finalPWM = floorPWM;
      }
      else {
        // Calcola l'avanzamento normalizzato della dissolvenza da 0 a 1000
        uint32_t progress = ((uint32_t)fadeElapsed * 1000UL) / fadeTime;
        // Calcola la parte temporale ancora rimanente
        uint32_t remaining = 1000UL - progress;
        // Applica una curva cubica alla parte rimanente
        uint32_t curvedRemaining =
          (remaining * remaining * remaining) / 1000000UL;
        // Calcola l'intervallo luminoso compreso tra il punto iniziale e il floor
        uint32_t fadeRange = fadeStartPWM - floorPWM;
        // Calcola il pwm corrente lungo la dissolvenza cubica
        finalPWM =
          floorPWM + (fadeRange * curvedRemaining) / 1000UL;
      }
    }
    // Durante i due secondi sotto soglia si mantiene fissa la luminosita memorizzata
    else if (detectorRawLowTiming) {
      finalPWM = detectorFinalFadeStartPWM[n];
    }
    else {
      // Ricava il valore corrente dell'envelope nella posizione stereo della nota
      uint16_t currentEnvelope =
        detectorEnvelopeForNote(n);
      // Recupera il riferimento acquisito 50 ms dopo la note on
      uint16_t reference =
        detectorReferenceADC[n];
      // Sopra il riferimento la nota resta al massimo consentito dalla velocity
      if (currentEnvelope >= reference) {
        finalPWM = velocityCap;
      }
      // Sotto il minimo elaborato la nota viene portata direttamente al floor
      else if (currentEnvelope <= detectorProcessedMinADC) {
        finalPWM = floorPWM;
      }
      // Tra valore minimo e valore di riferimento la luminosita viene interpolata linearmente
      else {
        // Calcola l'intervallo utile dell'envelope elaborato
        uint32_t envelopeRange =
          reference - detectorProcessedMinADC;
        // Protegge da divisione per zero nel caso in cui riferimento e minimo coincidano
        if (envelopeRange == 0) {
          finalPWM = velocityCap;
        }
        else {
          // Calcola la posizione corrente dell'envelope all'interno dell'intervallo utile
          uint32_t envelopePosition =
            currentEnvelope - detectorProcessedMinADC;
          // Calcola l'intervallo pwm compreso tra floor e massimo della nota
          uint32_t pwmRange =
            velocityCap - floorPWM;
          // Converte linearmente la posizione dell'envelope nel corrispondente pwm
          finalPWM =
            floorPWM +
            (envelopePosition * pwmRange) /
            envelopeRange;
        }
      }
    }
    // Aggiorna la lampada soltanto se il pwm calcolato e diverso dall'ultimo inviato
    if (finalPWM != lastPWM[n])
      setPianoLamp(n,  finalPWM);
  }
}

// Interpreta note on note off e controllo sustain indipendentemente dal canale midi: channel agnostic
void processMidiMessage(byte status, byte data1, byte data2) {
  // Ignora i messaggi midi nelle modalita che non utilizzano la tastiera
  if (currentMode != MODE_DETECTOR &&
      currentMode != MODE_PIANO &&
      currentMode != MODE_ORGAN) {
    return;
  }
  // Isola il tipo di messaggio eliminando i quattro bit del canale midi
  byte type = status & 0xF0;
  // Gestisce i messaggi note on
  if (type == 0x90) {
    // Una velocity maggiore di zero viene interpretata come note on
    if (data2 > 0)
      noteOn(data1, data2);
    // Una note on con velocity zero viene interpretata come note off
    else
      noteOff(data1);
  }
  // Gestisce i messaggi note off espliciti
  else if (type == 0x80) {
    noteOff(data1);
  }
  // Gestisce il control change 64 dedicato al pedale sustain
  else if (type == 0xB0 && data1 == 64) {
    // Valori da 64 in su indicano sustain premuto
    setSustain(data2 >= 64);
  }
}

// Ricostruzione dei messaggi midi ricevuti su serial1 gestendo anche il running status
void readMidi() {
  // Continua finche sono presenti byte nel buffer midi
  while (Serial1.available() > 0) {
    // Legge il byte successivo
    byte b = Serial1.read();
    // I messaggi realtime vengono ignorati senza interrompere il running status
    if (b >= 0xF8)
      continue;
    // Uno status di canale aggiorna il running status mentre uno status di sistema lo azzera
    if (b & 0x80) {
      // Gli status inferiori a 0xf0 appartengono ai messaggi di canale
      if (b < 0xF0) {
        // Memorizza lo status per i messaggi successivi
        runningStatus = b;
        // Azzera il conteggio dei dati del nuovo messaggio
        midiDataCount = 0;
      }
      else {
        // Gli status di sistema annullano il running status corrente
        runningStatus = 0;
        // Azzera anche il conteggio dei dati
        midiDataCount = 0;
      }
      // Dopo aver elaborato lo status passa al byte successivo
      continue;
    }
    // Ignora i data byte se non esiste uno status valido a cui associarli
    if (runningStatus == 0)
      continue;
    // Isola il tipo di messaggio midi ignorando il canale
    byte type = runningStatus & 0xF0;
    // Program change e channel pressure richiedono un data byte
    byte bytesNeeded =
      (type == 0xC0 || type == 0xD0)
      ? 1
      : 2;
    // Se deve ancora essere acquisito il primo data byte
    if (midiDataCount == 0) {
      // Memorizza il primo dato del messaggio
      midiData1 = b;
      // Segna che il primo data byte e stato ricevuto
      midiDataCount = 1;
      // Per i messaggi a un solo dato considera subito conclusa la ricezione
      if (bytesNeeded == 1)
        midiDataCount = 0;
    }
    else {
      // Memorizza il secondo data byte del messaggio
      byte midiData2 = b;
      // Invia il messaggio completo alla funzione di interpretazione
      processMidiMessage(
        runningStatus,
        midiData1,
        midiData2
      );
      // Prepara il parser alla ricezione del messaggio successivo mantenendo il running status
      midiDataCount = 0;
    }
  }
}



// Inizializza ingressi uscite bus i2c midi adc e driver pwm
void setup() {
  // Configura gli ingressi del selettore utilizzando le resistenze pullup interne
  pinMode(MODE_PIN_STANDBY, INPUT_PULLUP);
  pinMode(MODE_PIN_DETECTOR, INPUT_PULLUP);
  pinMode(MODE_PIN_PIANO, INPUT_PULLUP);
  pinMode(MODE_PIN_ORGAN, INPUT_PULLUP);
  // Configura come uscita il pin pwm del vu meter
  pinMode(VU_PIN, OUTPUT);
  // Inizializza il vu meter spento
  analogWrite(VU_PIN, 0);
  // Usa il riferimento analogico predefinito e prepara il minimo elaborato del detector
  analogReference(DEFAULT);
  // Converte i mv minimi nel corrispondente valore adc
  uint16_t minCorrectedADC =
    millivoltsToADC(DETECTOR_MIN_CORRECTED_MV);
  // Applica la curva percettiva per ottenere il limite inferiore della mappatura elaborata
  detectorProcessedMinADC =
    applyPerceptualCurve(minCorrectedADC);
  // Inizializza la porta midi alla velocita standard di 31250 baud
  Serial1.begin(31250);
  // Avvia il bus i2c a 200 khz
  Wire.begin();
  Wire.setClock(200000);
  // Attende l'assestamento iniziale dell'avvio dell'elettronica
  delay(300);
  // Inizializza i sei pca9685 e imposta il pwm a 100 hz
  for (byte i = 0; i < NUM_PCA; i++) {
    pca[i]->begin();
    pca[i]->setPWMFreq(100);
  }
  // Avvio test lampade
  startupLampTest();
  // Elimina eventuali dati midi ricevuti durante l'avvio
  flushMidi();
  // Attende brevemente prima di leggere la posizione del selettore
  delay(50);
  // Legge la modalita presente sul selettore al termine dell'avvio
  candidateMode = readModeRaw();
  // Entra nella modalita selezionata
  enterMode(candidateMode);
}

// Esegue continuamente il controllo della modalita e le funzioni richieste dallo stato attivo
void loop() {
  // Controlla continuamente eventuali cambiamenti del selettore
  updateModeSelector();
  switch (currentMode) {
    // In standby non viene eseguita alcuna elaborazione degli input
    case MODE_STANDBY:
      break;
    // Nella modalita ADSR Detector si gestisce midi campionamento envelope riferimenti e lampade
    case MODE_DETECTOR:
      // Legge e interpreta i messaggi midi disponibili e campiona l'envelope, solo dopo un nuovo campione aggiorna gli eventuali riferimenti delle note
      readMidi();
      if (updateAdsrDetector())
        captureDetectorReferences();
      // Aggiorna la luminosita delle note in funzione dell'envelope
      updateDetectorLamps();
      break;
    // Nella modalita ADSR Piano gestisce midi e decadimento delle note
    case MODE_PIANO:
      // Legge e interpreta i messaggi midi disponibili e aggiorna il decadimento luminoso delle note attive
      readMidi();
      updatePianoDecay();
      break;
    case MODE_ORGAN:
      // Legge note sustain e relativi messaggi midi
      readMidi();
      break;
  }
}
