#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the prototype TTN
// network initially.
//ttn
static const PROGMEM u1_t NWKSKEY[16] = { 0xA3, 0xFB, 0xB7, 0x18, 0xA9, 0xCB, 0x4E, 0xB1, 0x2A, 0x3C, 0xEE, 0x54, 0xBA, 0xF8, 0xF2, 0xD9 };
// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the prototype TTN
// network initially.
//ttn
static const u1_t PROGMEM APPSKEY[16] = { 0xAE, 0x41, 0xF3, 0x1B, 0x03, 0x3C, 0x53, 0x0D, 0x94, 0x22, 0xA8, 0xE0, 0x89, 0xD1, 0xB4, 0x79 };

//
// LoRaWAN end-device address (DevAddr)
// See https://thethingsnetwork.org/wiki/AddressSpace
// ttn
static const u4_t DEVADDR = 0x26031433;
static const int LM35 = 0;
int temp_lida = 0;
float temperatura;
int contador = 0;

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }


static uint8_t mydata[] = { 0, 0, 0, 0, 0, 0, 0, 0};
static osjob_t initjob, sendjob, blinkjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 20;

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 9,
  .dio = {2, 6, 7},
};
void do_send(osjob_t* j) {

  dtostrf(temperatura, 5, 2, (char*)mydata);

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare transmission at the next possible time.
    LMIC_setTxData2(1, mydata, strlen((char*) mydata), 0);
    Serial.println(); 
    Serial.println("Packet queued");
    Serial.print("TX nº: ");
    Serial.println(contador);
    contador++;
    Serial.println(LMIC.freq);
    Serial.print("Temperatura = ");
    Serial.print(temperatura);
    Serial.println(" *C");
    Serial.println((char*)mydata);
  }
}
void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  Serial.println(ev);
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println("EV_SCAN_TIMEOUT");
      break;
    case EV_BEACON_FOUND:
      Serial.println("EV_BEACON_FOUND");
      break;
    case EV_BEACON_MISSED:
      Serial.println("EV_BEACON_MISSED");
      break;
    case EV_BEACON_TRACKED:
      Serial.println("EV_BEACON_TRACKED");
      break;
    case EV_JOINING:
      Serial.println("EV_JOINING");
      break;
    case EV_JOINED:
      Serial.println("EV_JOINED");
      break;
    case EV_RFU1:
      Serial.println("EV_RFU1");
      break;
    case EV_JOIN_FAILED:
      Serial.println("EV_JOIN_FAILED");
      break;
    case EV_REJOIN_FAILED:
      Serial.println("EV_REJOIN_FAILED");
      break;
    case EV_TXCOMPLETE:
      Serial.println("EV_TXCOMPLETE (includes waiting for RX windows)");
      if (LMIC.dataLen) {
        // data received in rx slot after tx
        Serial.print("Data Received: ");
        Serial.write(LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
        Serial.println();
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println("EV_LOST_TSYNC");
      break;
    case EV_RESET:
      Serial.println("EV_RESET");
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println("EV_RXCOMPLETE");
      break;
    case EV_LINK_DEAD:
      Serial.println("EV_LINK_DEAD");
      break;
    case EV_LINK_ALIVE:
      Serial.println("EV_LINK_ALIVE");
      break;
    default:
      Serial.println("Unknown event");
      break;
  }
}

void setup() {
  Serial.begin(9600);
  analogReference(INTERNAL);
  while (!Serial);
  Serial.println("Starting");
#ifdef VCC_ENABLE
  // For Pinoccio Scout boards
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000);
#endif

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  //LMIC_setClockError(MAX_CLOCK_ERROR * 1/100);
  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
#ifdef PROGMEM
  // On AVR, these values are stored in flash and only copied to RAM
  // once. Copy them to a temporary buffer here, LMIC_setSession will
  // copy them into a buffer of its own again.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
#else
  // If not running an AVR with PROGMEM, just use the arrays directly
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF9, 14);

  for (int i = 1; i < 64; i++)
  {
    LMIC_disableChannel(i);  // only the first channel 902.3Mhz works now.
  }

  // Start job
  do_send(&sendjob);
}

void loop() {
  if(contador == 100){
    Serial.println("FIM");
    while(1);
    }
  os_runloop_once();
  temp_lida = analogRead(LM35);
  temperatura = temp_lida * 0.1075268817204301;
}
