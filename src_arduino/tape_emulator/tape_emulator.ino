
#define CMD_PLAY        0x01
#define CMD_FASTFORWARD 0x04
#define CMD_REWIND      0x08
#define CMD_STOP        0x60
#define CMD_REPEAT      0x01
#define CMD_RANDOM      0x02
#define CMD_SEEK_UP     0x10
#define CMD_SEEK_DOWN   0x20


#define RX_TIMEOUT_US   10000
#define IN_BUFFER_SIZE  16U

#define RESET_BIT_POS   0x08


//#define ENABLE_DEBUG_OUTPUT 1


#if ENABLE_DEBUG_OUTPUT
#define DEBUG_PRINT(...)  Serial.print(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#endif

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

const int IO_PIN = 3;

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

static uint8_t inBuffer[IN_BUFFER_SIZE] = {0U};
static uint8_t byteReceived = 0;
static uint8_t biteShiftMask = RESET_BIT_POS;
static uint32_t rx_time = 0;


void setup() {
  pinMode(IO_PIN, INPUT_PULLUP); //INPUT_PULLUP OUTPUT INPUT
  attachInterrupt(digitalPinToInterrupt(IO_PIN), collectInputData, CHANGE);
}

void loop() {

  if (rx_time >= (micros() - RX_TIMEOUT_US)) {
    if (byteReceived != 0U ) {
      DEBUG_PRINT("Message resived\r\n");
      process_radio_message((rxMessage_t*)inBuffer);
      bufferReset();
    }
  }
}

void bufferReset() {

  for (uint8_t i = 0U; i < byteReceived; i++) {
    inBuffer[i] = 0U;
  }

  byteReceived = 0;
  biteShiftMask = RESET_BIT_POS;

}

void collectInputData() {

  uint32_t elapsed_time = 0;

  // calculate pulse time
  elapsed_time = micros() - rx_time;
  rx_time = micros();

  if (digitalRead(IO_PIN) == LOW) {
    return;
  }

  if ( (elapsed_time > 1000) && (elapsed_time < 2200) ) {
    inBuffer[byteReceived] |= biteShiftMask;
  } else {
    DEBUG_PRINT("Long low pulse. Reset buffer.\r\n");
    bufferReset();
    return;
  }

  biteShiftMask >>= 1U;

  if (biteShiftMask == 0U) {
    biteShiftMask = RESET_BIT_POS; //save one nibble to one byte
    ++byteReceived;
  }

  if (byteReceived >= IN_BUFFER_SIZE) {
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

  detachInterrupt(digitalPinToInterrupt(IO_PIN));
  pinMode(IO_PIN, OUTPUT);

  for (uint8_t i = 0; i < lenght; i++) {
    send_nibble(message[i]);
  }

  pinMode(IO_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(IO_PIN), collectInputData, CHANGE);
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

    send_message(TAPECMD_PLAYING, sizeof(TAPECMD_PLAYING));
    delay(7);
    send_message(TAPECMD_PLAYBACK, sizeof(TAPECMD_PLAYBACK));
    delay(7);
  }
}
