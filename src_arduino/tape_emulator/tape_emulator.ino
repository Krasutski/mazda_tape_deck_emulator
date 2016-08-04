//Configuration

#define ENABLE_DEBUG_OUTPUT

//Depended parameters

#define BIT_LOW_LEVEL_DURATION_MIN  (1400)  //value in us
#define BIT_LOW_LEVEL_DURATION_MAX  (2000)  //value in us

#ifdef ENABLE_DEBUG_OUTPUT
#define DEBUG_PRINT(...)  if(Serial){ Serial.print(__VA_ARGS__); }
#else
#define DEBUG_PRINT(...)
#endif

//Constants
#define IO_PIN_INPUT_MODE (INPUT_PULLUP) //INPUT_PULLUP OUTPUT INPUT

#define RX_TIMEOUT_MS   12U
#define IN_BUFFER_SIZE  96U

#define NIBBLE_RESET_BIT_POS    0x08

#define IO_PIN                  3U
#define MIC_PIN                 2U

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

typedef enum SubConmmandPlayback_e {
  Playback_Play = 0x01,
  Playback_FF = 0x04,
  Playback_REW = 0x08,
  Playback_Stop = 0x60
} SubConmmandPlayback_t;

typedef enum SubCommandSetConfig_e {
  SetConfig_RepeatMode = 0x01,
  SetConfig_RandomMode = 0x02,
  SetConfig_FastForwarding = 0x10,
  SetConfig_FastRewinding = 0x20
} SubCommandSetConfig_t;

typedef enum Jack35Control_e {
  Jack35_ShortPress,
  Jack35_LongPress,
  Jack35_DoublePress,
  Jack35_VolUp,
  Jack35_VolDown,
} Jack35Control_t;

// data present in nibbles, byte equal nibble
//Wakeup notification
const uint8_t TAPECMD_POWER_ON[] =        {0x08, 0x08, 0x01, 0x02};         //Wake up notification
//Status messages: {target, command(status), arg1, arg2, checksum}
const uint8_t TAPECMD_STOPPED[] =         {0x08, 0x09, 0x00, 0x0C, 0x0E};   //0 - Stopped, C - not use desk
const uint8_t TAPECMD_PLAYING[] =         {0x08, 0x09, 0x04, 0x01, 0x05};   //4 - Playing, 1 - tape in use
const uint8_t TAPECMD_SEEKING[] =         {0x08, 0x09, 0x05, 0x01, 0x06};   //5 - seeking, 1 - tape in use
//Detailed status  {target, command(det. status), arg1, arg2, arg3, arg4, arg5, arg6, checksum}
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
  pinMode(MIC_PIN, INPUT);

  pinMode(IO_PIN, IO_PIN_INPUT_MODE);
  attachInterrupt(digitalPinToInterrupt(IO_PIN), collectInputData, CHANGE);

#ifdef ENABLE_DEBUG_OUTPUT
  Serial.begin(9600);
#endif

  DEBUG_PRINT("Init....\r\n");
}

void loop() {

  if ( ( millis() - rx_time_ms ) > RX_TIMEOUT_MS) {
    if (nibblesReceived != 0U ) {

      noInterrupts(); {
        DEBUG_PRINT("RX[");
        DEBUG_PRINT(nibblesReceived);
        DEBUG_PRINT("] ");

        for (int i = 0; i < nibblesReceived; i++) {
          DEBUG_PRINT(inNibblesBuffer[i], HEX);
        }
        DEBUG_PRINT("\r\n");

        process_radio_message((rxMessage_t*)inNibblesBuffer);

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

  // calculate pulse time
  elapsed_time = micros() - rx_time_us;
  rx_time_us = micros();
  rx_time_ms = millis();

  if (digitalRead(IO_PIN) == LOW) {
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
    DEBUG_PRINT("Buffer overflow, reset!\r\n");
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
  DEBUG_PRINT("TX[");
  DEBUG_PRINT(lenght);
  DEBUG_PRINT("] ");

  for (int i = 0; i < lenght; i++) {
    DEBUG_PRINT(((uint8_t*)message)[i], HEX);
  }

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

  switch (message->command) {
    case Command_AnyBodyHome:
      DEBUG_PRINT("Any body home msg\r\n");

      send_message(TAPECMD_POWER_ON, sizeof(TAPECMD_POWER_ON));
      send_message(TAPECMD_CASSETE_PRESENT, sizeof(TAPECMD_CASSETE_PRESENT));
      break;
    case Command_WakeUp:
      DEBUG_PRINT("Wake up msg\r\n");

      send_message(TAPECMD_CASSETE_PRESENT, sizeof(TAPECMD_CASSETE_PRESENT));
      send_message(TAPECMD_STOPPED, sizeof(TAPECMD_STOPPED));
      break;
    case Command_Control:
      if (message->data[0] == SubCommand_Playback) {
        uint8_t subCmd = ((message->data[1] << 4U) & 0xF0) | (message->data[2] & 0x0F);
        if (subCmd == Playback_Play) {
          DEBUG_PRINT("Playback MSG = Playback_Play\r\n");
          send_message(TAPECMD_PLAYING, sizeof(TAPECMD_PLAYING));
          send_message(TAPECMD_PLAYBACK, sizeof(TAPECMD_PLAYBACK));
        } else if (subCmd == Playback_FF) {
          Jack35Control(Jack35_ShortPress);
          DEBUG_PRINT("Playback MSG = Playback_FF\r\n");
          send_message(TAPECMD_PLAYBACK, sizeof(TAPECMD_PLAYBACK));
        } else if (subCmd == Playback_REW) {
          Jack35Control(Jack35_DoublePress);
          DEBUG_PRINT("Playback MSG = Playback_REW\r\n");
          send_message(TAPECMD_PLAYBACK, sizeof(TAPECMD_PLAYBACK));
        } else if (subCmd == Playback_Stop) {
          DEBUG_PRINT("Playback MSG = Playback_Stop\r\n");
          send_message(TAPECMD_STOPPED, sizeof(TAPECMD_STOPPED));
        } else {
          DEBUG_PRINT("Playback MSG = ");
          DEBUG_PRINT(subCmd);
          DEBUG_PRINT("\r\n");
        }
      } else if (message->data[0] == SubCommand_SeekTrack) {
        DEBUG_PRINT("SubCommand_SeekTrack\r\n");
      } else if (message->data[0] == SubCommand_SetConfig) {
        uint8_t subCmd = ((message->data[1] << 4U) & 0xF0) | (message->data[2] & 0x0F);
        if ( subCmd == SetConfig_RepeatMode) {
          DEBUG_PRINT("SetConfig_RepeatMode\r\n");
        } else if ( subCmd == SetConfig_RandomMode) {
          DEBUG_PRINT("SetConfig_RandomMode\r\n");
        } else if ( subCmd == SetConfig_FastForwarding) {
          DEBUG_PRINT("SetConfig_FastForwarding\r\n");
        } else if ( subCmd == SetConfig_FastRewinding ) {
          DEBUG_PRINT("SetConfig_FastRewinding\r\n");
        } else {
          DEBUG_PRINT("SubCommand_SetConfig = ");
          DEBUG_PRINT(subCmd);
          DEBUG_PRINT("\r\n");
        }
      } else {
        DEBUG_PRINT("UNCKNOWN Sub command\r\n");
      }
      break;
    default:
      DEBUG_PRINT("another cmd = ");
      DEBUG_PRINT(message->command);
      DEBUG_PRINT("\r\n");
      break;
  }
}

void Jack35Control(Jack35Control_t event) {
  pinMode(MIC_PIN, OUTPUT);

  switch (event) {
    case   Jack35_ShortPress:
      DEBUG_PRINT("Jack35_ShortPress\r\n");
      digitalWrite(MIC_PIN, LOW);
      delay(500);
      digitalWrite(MIC_PIN, HIGH);
      break;
    case Jack35_LongPress:
      DEBUG_PRINT("Jack35_LongPress\r\n");
      digitalWrite(MIC_PIN, LOW);
      delay(2500);
      digitalWrite(MIC_PIN, HIGH);
      break;
    case Jack35_DoublePress:
      DEBUG_PRINT("Jack35_DoublePress\r\n");
      digitalWrite(MIC_PIN, LOW);
      delay(200);
      digitalWrite(MIC_PIN, HIGH);
      delay(250);
      digitalWrite(MIC_PIN, LOW);
      delay(200);
      digitalWrite(MIC_PIN, HIGH);
      break;
    case Jack35_VolUp:
      DEBUG_PRINT("Jack35_VolUp\r\n");
      //don't implemeted hardware, see spetification
      //Wired Audio Headset Specification (v1.1)
      break;
    case Jack35_VolDown:
      DEBUG_PRINT("Jack35_VolDown\r\n");
      //don't implemeted hardware, see spetification
      //Wired Audio Headset Specification (v1.1)
      break;
    default:
      DEBUG_PRINT("Jack35 unckon argument");
      break;
  }

  pinMode(MIC_PIN, INPUT);
}
