//Configuration

#define USE_TIMER1

//#define ENABLE_DEBUG_OUTPUT

//Depended parameters

#ifdef USE_TIMER1
#define BIT_LOW_LEVEL_DURATION_MIN  (350) //value in timer ticks, 1tiks=4us, 1400/4 = 350
#define BIT_LOW_LEVEL_DURATION_MAX  (500) //value in timer ticks, 1tiks=4us, 2000/4 = 500
#else
#define BIT_LOW_LEVEL_DURATION_MIN  (1400)  //value in us
#define BIT_LOW_LEVEL_DURATION_MAX  (2000)  //value in us
#endif

#ifdef ENABLE_DEBUG_OUTPUT
#define DEBUG_PRINT(...)  if(Serial){ Serial.print(__VA_ARGS__); }
#else
#define DEBUG_PRINT(...)
#endif

//Constants
#define IO_PIN_INPUT_MODE (INPUT_PULLUP) //INPUT_PULLUP OUTPUT INPUT

#define RX_TIMEOUT_MS   12U
#define IN_BUFFER_SIZE  16U

#define NIBBLE_RESET_BIT_POS   0x08

#define IO_PIN 3U

/* message mustbe aligned to bytes*/
typedef struct rxMessage {
  uint8_t target;
  uint8_t command;
  uint8_t data[];
} rxMessage_t;

typedef enum rxMessageTarget_e {
  Target_TapeDesk = 0x00,
  Target_Unknown = 0x01,
  Target_CDDesk = 0x03,
  Target_CDChangerExt = 0x05,
  Target_CDChangerUpper = 0x06,
  Target_MDDesk = 0x07,
  Target_BaseUnit = 0x08
} rxMessageTarget_t;

typedef enum rxMessageCommand_e {
  Command_Control = 0x01,
  Command_AnyBodyHome = 0x08,
  Command_WakeUp = 0x09
} rxMessageCommand_t;

typedef enum rxMessageSubCommand_e {
  SubCommand_Playback = 0x01,
  SubCommand_SeekTrack = 0x03,
  SubCommand_SetConfig = 0x04
} rxMessageSubCommand_t;


// data present in nibbles, byte equal nibble
const uint8_t TAPECMD_POWER_ON[] =        {0x08, 0x08, 0x01, 0x02};
const uint8_t TAPECMD_STOPPED[] =         {0x08, 0x09, 0x00, 0x0C, 0x0E};
const uint8_t TAPECMD_PLAYING[] =         {0x08, 0x09, 0x04, 0x01, 0x05};
const uint8_t TAPECMD_SEEKING[] =         {0x08, 0x09, 0x05, 0x01, 0x06};
const uint8_t TAPECMD_CASSETE_PRESENT[] = {0x08, 0x0B, 0x09, 0x00, 0x04, 0x00, 0x00, 0x0C, 0x03};
const uint8_t TAPECMD_PLAYBACK[] =        {0x08, 0x0B, 0x09, 0x00, 0x04, 0x00, 0x00, 0x01, 0x00};
const uint8_t TAPECMD_RANDOM_PLAY[] =     {0x08, 0x0B, 0x09, 0x00, 0x06, 0x00, 0x00, 0x01, 0x0E};
const uint8_t TAPECMD_REPEAT_PLAY[] =     {0x08, 0x0B, 0x09, 0x00, 0x05, 0x00, 0x00, 0x01, 0x0F};
const uint8_t TAPECMD_FAST_REWIND[] =     {0x08, 0x0B, 0x09, 0x03, 0x04, 0x00, 0x01, 0x01, 0x0E};
const uint8_t TAPECMD_FAST_FORWARD[] =    {0x08, 0x0B, 0x09, 0x02, 0x04, 0x00, 0x01, 0x01, 0x0D};

static uint8_t inNibblesBuffer[IN_BUFFER_SIZE] = {0U};
static uint8_t nibblesReceived = 0;
static uint8_t biteShiftMask = NIBBLE_RESET_BIT_POS;
static uint32_t rx_time_us = 0;
static uint32_t rx_time_ms = 0;

void setup() {
  pinMode(IO_PIN, IO_PIN_INPUT_MODE);
  attachInterrupt(digitalPinToInterrupt(IO_PIN), collectInputData, CHANGE);

#ifdef ENABLE_DEBUG_OUTPUT
  Serial.begin(9600);
#endif

  DEBUG_PRINT("Init....\r\n");

#ifdef USE_TIMER1
  noInterrupts(); {
    TCCR1A = 0;
    TCNT1 = 0;

    //TCCR1B = ((0 << CS12) | (0 << CS11) | (1 << CS10) ); // 1 prescaler, 16MHz => 1tick = 0.0625us, full overflow time = 4ms
    //TCCR1B = ((0 << CS12) | (1 << CS11) | (0 << CS10) ); // 8 prescaler, 2MHz => 1tick = 0.5us, full overflow time = 32ms
    TCCR1B = ((0 << CS12) | (1 << CS11) | (1 << CS10) ); // 64 prescaler, 250khz => 1tick = 4us, full overflow time = 262ms
    //TCCR1B = ((1 << CS12) | (0 << CS11) | (0 << CS10) ); // 256 prescaler, 62.5khz => 1tick = 16us, full overflow time =  16.7s  //TIMSK1 |= (1 << TOIE1);  // enable timer overflow interrupt
  } interrupts();
#endif
}

void loop() {

  if ( ( millis() - rx_time_ms ) > RX_TIMEOUT_MS) {
    if (nibblesReceived != 0U ) {
      DEBUG_PRINT("\r\nLen=");
      DEBUG_PRINT(nibblesReceived);
      DEBUG_PRINT(" >> ");

      noInterrupts(); {
        process_radio_message((rxMessage_t*)inNibblesBuffer);
        for (int i = 0; i < nibblesReceived; i++) {
          DEBUG_PRINT(inNibblesBuffer[i], HEX);
        }
        DEBUG_PRINT("\r\n");
        bufferReset();

        rx_time_ms = millis();
      } interrupts();
    }
  }
}

void bufferReset() {
  for (uint8_t i = 0U; i < nibblesReceived; i++) {
    inNibblesBuffer[i] = 0U;
  }

  nibblesReceived = 0;
  biteShiftMask = NIBBLE_RESET_BIT_POS;
}

void collectInputData() {
  uint32_t elapsed_time = 0;

#ifdef USE_TIMER1
  elapsed_time = TCNT1;
#else
  // calculate pulse time
  elapsed_time = micros() - rx_time_us;
  rx_time_us = micros();
#endif
  rx_time_ms = millis();

  if (digitalRead(IO_PIN) == LOW) {
#ifdef USE_TIMER1
    TCNT1 = 0;
#endif
    return;
  }

  if ( (elapsed_time > BIT_LOW_LEVEL_DURATION_MIN) && (elapsed_time < BIT_LOW_LEVEL_DURATION_MAX) ) {
    inNibblesBuffer[nibblesReceived] |= biteShiftMask;
  }

  biteShiftMask >>= 1U;

  if (biteShiftMask == 0U) {
    biteShiftMask = NIBBLE_RESET_BIT_POS; //save one nibble to one byte
    ++nibblesReceived;
  }

  if (nibblesReceived >= IN_BUFFER_SIZE) {
    DEBUG_PRINT("Buffer overflow, reset!");
    bufferReset();
  }
}

static void send_nibble(const uint8_t nibble) {
  uint8_t nibbleShiftMask = 0x08;
  uint8_t bit_value = 0U;

  while (nibbleShiftMask != 0U) {
    // Pull the bus down
    digitalWrite(IO_PIN, LOW);

    bit_value = nibble & nibbleShiftMask;

    //Set Logic 0 or 1 time
    if (bit_value) {
      delayMicroseconds(1780);
    } else {
      delayMicroseconds(600);
    }

    // Release the bus
    digitalWrite(IO_PIN, HIGH);

    //End logic pause
    if (bit_value) {
      delayMicroseconds(1200);
    } else {
      delayMicroseconds(2380);
    }

    nibbleShiftMask >>= 1U;
  }
}

// Send a message on the Mazda radio bus
void send_message(const uint8_t *message, const uint8_t lenght) {
  DEBUG_PRINT("Send message...");
  DEBUG_PRINT(lenght);
  DEBUG_PRINT("\r\n");

  noInterrupts(); {

    do {
      delay(10);
    } while (digitalRead(IO_PIN) != HIGH);

    detachInterrupt(digitalPinToInterrupt(IO_PIN));
    digitalWrite(IO_PIN, HIGH);
    pinMode(IO_PIN, OUTPUT);

    for (uint8_t i = 0; i < lenght; i++) {
      send_nibble(message[i]);
    }

    pinMode(IO_PIN, IO_PIN_INPUT_MODE);
    attachInterrupt(digitalPinToInterrupt(IO_PIN), collectInputData, CHANGE);

  } interrupts();
}


void process_radio_message(const rxMessage_t *message) {
  //check target, 0 is tape desk
  if (message->target != Target_TapeDesk) {
    return;
  }

  if (message->command == Command_AnyBodyHome) {
    DEBUG_PRINT("Any body home msg\r\n");

    send_message(TAPECMD_POWER_ON, sizeof(TAPECMD_POWER_ON));
    delay(8);
    send_message(TAPECMD_CASSETE_PRESENT, sizeof(TAPECMD_CASSETE_PRESENT));

  } else if (message->command == Command_WakeUp) {
    DEBUG_PRINT("Wake up msg\r\n");

    send_message(TAPECMD_CASSETE_PRESENT, sizeof(TAPECMD_CASSETE_PRESENT));
    delay(10);
    send_message(TAPECMD_STOPPED, sizeof(TAPECMD_STOPPED));
  } else {
    DEBUG_PRINT("another msg\r\n");
    DEBUG_PRINT(message->command);
    DEBUG_PRINT("\r\n");

    send_message(TAPECMD_PLAYING, sizeof(TAPECMD_PLAYING));
    delay(7);
    send_message(TAPECMD_PLAYBACK, sizeof(TAPECMD_PLAYBACK));
    delay(7);
  }
}
