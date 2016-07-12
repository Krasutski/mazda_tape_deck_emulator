
#define CMD_PLAY        0x01
#define CMD_FASTFORWARD 0x04
#define CMD_REWIND      0x08
#define CMD_STOP        0x60
#define CMD_REPEAT      0x01
#define CMD_RANDOM      0x02
#define CMD_SEEK_UP     0x10
#define CMD_SEEK_DOWN   0x20

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
const uint8_t TAPECMD_PLAYING[] =          {0x08, 0x09, 0x04, 0x01, 0x05};
const uint8_t TAPECMD_SEEKING[] =         {0x08, 0x09, 0x05, 0x01, 0x06};
const uint8_t TAPECMD_CASSETE_PRESENT[] = {0x08, 0x0B, 0x09, 0x00, 0x04, 0x00, 0x00, 0x0C, 0x03};
const uint8_t TAPECMD_PLAYBACK[] =        {0x08, 0x0B, 0x09, 0x00, 0x04, 0x00, 0x00, 0x01, 0x00};
const uint8_t TAPECMD_RANDOM_PLAY[] =     {0x08, 0x0B, 0x09, 0x00, 0x06, 0x00, 0x00, 0x01, 0x0E};
const uint8_t TAPECMD_REPEAT_PLAY[] =     {0x08, 0x0B, 0x09, 0x00, 0x05, 0x00, 0x00, 0x01, 0x0F};
const uint8_t TAPECMD_FAST_REWIND[] =     {0x08, 0x0B, 0x09, 0x03, 0x04, 0x00, 0x01, 0x01, 0x0E};
const uint8_t TAPECMD_FAST_FORWARD[] =    {0x08, 0x0B, 0x09, 0x02, 0x04, 0x00, 0x01, 0x01, 0x0D};

#define RX_TIMEOUT_US   10000
#define IN_BUFFER_SIZE  16U
#define RESET_BIT_POS   0x08
uint8_t inBuffer[IN_BUFFER_SIZE];
uint8_t bytePos = 0;
uint8_t biteShiftMask = RESET_BIT_POS;

static uint32_t rx_time = 0;


void setup() {
  pinMode(IO_PIN, INPUT_PULLUP); //INPUT_PULLUP OUTPUT INPUT
  attachInterrupt(digitalPinToInterrupt(IO_PIN), collectInputData, CHANGE);

}

void loop() {
  // put your main code here, to run repeatedly:
#if DEBUG_CODE
  static int init = 0;

  if (init == 0) {
    send_message(&TAPECMD_POWER_ON);
    delay(1000);
    send_message(&TAPECMD_CASSETE_PRESENT);
    delay(1000);
    init = 1;
  } else {
    delay(60000);
    init = 0;
  }
#endif

  if (rx_time >= (micros() - RX_TIMEOUT_US)) {
    process_radio_message((rxMessage_t*)inBuffer);
    bufferReset();
  }

}

void bufferReset()
{
  bytePos = 0;
  biteShiftMask = RESET_BIT_POS;

  //Clear buffer
  for (uint8_t i = 0; i < IN_BUFFER_SIZE; i++) {
    inBuffer[i] = 0;
  }
}

void collectInputData() {

  uint32_t elapsed_time = 0;

  // calculate pulse time
  elapsed_time = micros() - rx_time;
  rx_time = micros();

  if ( digitalRead(IO_PIN) == LOW) {
    return;
  }

  if ( (elapsed_time > 1000) && (elapsed_time < 2200) ) {
    inBuffer[bytePos] |= biteShiftMask;
  }

  biteShiftMask >>= 1U;

  if (biteShiftMask == 0U) {
    biteShiftMask = RESET_BIT_POS; //save one nibble to one byte
    ++bytePos;
  }

  if (bytePos >= IN_BUFFER_SIZE) {
    bufferReset();
  }
}

static void send_nibble(const uint8_t nibble)
{
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
  if (message->target != Target_TapeDesk)
  {
    return;
  }

  if (message->command == Command_AnyBodyHome) {

    send_message(TAPECMD_POWER_ON, sizeof(TAPECMD_POWER_ON));
    delay(8);
    send_message(TAPECMD_CASSETE_PRESENT, sizeof(TAPECMD_CASSETE_PRESENT));

  } else if (message->command == Command_WakeUp) {

    send_message(TAPECMD_CASSETE_PRESENT, sizeof(TAPECMD_CASSETE_PRESENT));
    delay(10);
    send_message(TAPECMD_STOPPED, sizeof(TAPECMD_STOPPED));
  } else {
    send_message(TAPECMD_PLAYING, sizeof(TAPECMD_PLAYING));
    delay(7);
    send_message(TAPECMD_PLAYBACK, sizeof(TAPECMD_PLAYBACK));
    delay(7);
  }

#if PROC_COMMAND
  // Control command
  if (message->command == 0x01) {

    // Extract the specific subcommand and command
    uint8_t subcommand = message->data[0U];
    uint8_t command = (message->data[1] << 4) | (message->data[2]);

    // Playback control
    if (subcommand == 0x1) {

      if (command & CMD_PLAY) {
        //on_play_changed( command );
      }

      if (command & CMD_FASTFORWARD) {
        //on_fastforward_changed( command );
      }

      if (command & CMD_REWIND) {
        //on_rewind_changed( command );
      }

      if (command & CMD_STOP) {
        //on_stop_changed( command );
      }
    }

    // Set configuration data
    if (subcommand == 0x4) {
      if (command == 0) {
        //on_repeat_changed( command );
      }

      if (command & CMD_REPEAT) {
        //on_repeat_changed( command );
      }

      if (command & CMD_RANDOM) {
        //on_random_pressed();
      }

      if (command & CMD_SEEK_UP) {
        //on_seek_up_pressed();
      }

      if (command & CMD_SEEK_DOWN) {
        //on_seek_down_pressed();
      }
    }
  }
#endif
}
