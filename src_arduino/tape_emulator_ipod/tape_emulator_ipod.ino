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
#define TYPE_IO_PIN_INPUT_MODE          (INPUT_PULLUP)
#define TYPE_IO_PIN                     3U

#define RX_TIMEOUT_MS                   12U
#define IN_BUFFER_SIZE                  96U

#define NIBBLE_RESET_BIT_POS            0x08


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

typedef enum IpodControl_e {
  Ipod_Play,
  Ipod_Pause,
  Ipod_Next,
  Ipod_Prev,
  Ipod_On,
  Ipod_Off,
  Ipod_Shuffle,
  Ipod_Repeat,
  Ipod_SeekForward,
  Ipod_SeekBackward,
  Ipod_SeekStop,
} IpodControl_t;

typedef struct {
  uint8_t mode;
  uint8_t command_h;
  uint8_t command_l;
  uint8_t parameters[4];
  uint8_t param_len;
  uint8_t checksum;
} const ipod_command;

// See: http://ipodlinux.org/wiki/Apple_Accessory_Protocol
// checksum: 0x100 - ((0x03 + mode + command_l + command_h
//                     + parameters + param_len) & 0xff)
ipod_command IPOD_REMOTE_MODE       = { 0x00, 0x01, 0x02, {}, 0, 0xfa };
ipod_command IPOD_BUTTON_RELEASED   = { 0x02, 0x00, 0x00, {}, 0, 0xfb };
ipod_command IPOD_PLAY_PAUSE        = { 0x02, 0x00, 0x01, {}, 0, 0xfa };
ipod_command IPOD_PLAY          = { 0x02, 0x00, 0x00, {0x01}, 1, 0xf9 };
ipod_command IPOD_PAUSE         = { 0x02, 0x00, 0x00, {0x02}, 1, 0xf8 };
ipod_command IPOD_STOP              = { 0x02, 0x00, 0x80, {}, 0, 0x7b };
ipod_command IPOD_NEXT              = { 0x02, 0x00, 0x08, {}, 0, 0xf3 };
ipod_command IPOD_PREV              = { 0x02, 0x00, 0x10, {}, 0, 0xeb };
ipod_command IPOD_ON      = { 0x02, 0x00, 0x00, {0x00, 0x08}, 2, 0xf1 };
ipod_command IPOD_OFF     = { 0x02, 0x00, 0x00, {0x00, 0x04}, 2, 0xf5 };
ipod_command IPOD_MUTE          = { 0x02, 0x00, 0x00, {0x04}, 1, 0xf6 };
ipod_command IPOD_SHUFFLE       = { 0x02, 0x00, 0x00, {0x80}, 1, 0x7a };
ipod_command IPOD_REPEAT  = { 0x02, 0x00, 0x00, {0x00, 0x01}, 2, 0xf8 };

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

static bool ipod_seeking = false;


void setup() {

  pinMode(TYPE_IO_PIN, TYPE_IO_PIN_INPUT_MODE);
  attachInterrupt(digitalPinToInterrupt(TYPE_IO_PIN), collectInputData, CHANGE);

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

  if (digitalRead(TYPE_IO_PIN) == LOW) {
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
    digitalWrite(TYPE_IO_PIN, LOW);

    bit_value = nibble & nibbleShiftMask;

    //Set Logic 0 or 1 time
    if (bit_value) {
      delayMicroseconds(1780);
    } else {
      delayMicroseconds(600);
    }

    // Release the bus
    digitalWrite(TYPE_IO_PIN, HIGH);

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
    } while (digitalRead(TYPE_IO_PIN) != HIGH);

    detachInterrupt(digitalPinToInterrupt(TYPE_IO_PIN));
    digitalWrite(TYPE_IO_PIN, HIGH);
    pinMode(TYPE_IO_PIN, OUTPUT);

    for (uint8_t i = 0; i < lenght; i++) {
      send_nibble(message[i]);
    }

    pinMode(TYPE_IO_PIN, TYPE_IO_PIN_INPUT_MODE);
    attachInterrupt(digitalPinToInterrupt(TYPE_IO_PIN), collectInputData, CHANGE);

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
          DEBUG_PRINT("Playback MSG = Playback_FF\r\n");
          send_message(TAPECMD_PLAYBACK, sizeof(TAPECMD_PLAYBACK));
        } else if (subCmd == Playback_REW) {
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

void send_byte( uint8_t c ) {
  c = ~c;
  // IPOD_STX_PORT &= ~_BV(IPOD_STX_BIT);   // start bit
  for ( uint8_t i = 10; i; i-- ) {         // 10 bits
    delayMicroseconds( 100 );                  // bit duration
    if ( c & 1 ) {
      // IPOD_STX_PORT &= ~_BV(IPOD_STX_BIT);    // data bit 0
    } else {
      // IPOD_STX_PORT |= _BV(IPOD_STX_BIT);     // data bit 1 or stop bit
    }
    c >>= 1;
  }
  delayMicroseconds(30); // Wait at least 10 us between bytes
}

void _ipod_send_command( const ipod_command * command ) {
  // Send the header
  send_byte( 0xff );
  send_byte( 0x55 );

  // Total length
  send_byte( 0x03 + pgm_read_byte(&command->param_len) );

  // Mode byte
  send_byte( pgm_read_byte(&command->mode) );

  // Command word
  send_byte( pgm_read_byte(&command->command_h) );
  send_byte( pgm_read_byte(&command->command_l) );

  // Send parameters
  uint8_t i = 0;
  while ( i < pgm_read_byte(&command->param_len) ) {
    send_byte( pgm_read_byte(&command->parameters[i]) );
    i++;
  }

  // Send the checksum
  send_byte( pgm_read_byte(&command->checksum) );
}

void ipod_send_command(ipod_command * command) {

  if (ipod_seeking) {
    _ipod_send_command( &IPOD_BUTTON_RELEASED );
    delay(5);
    _ipod_send_command( &IPOD_BUTTON_RELEASED );
    ipod_seeking = false;
  }

  _ipod_send_command( &IPOD_REMOTE_MODE );
  for (uint8_t i = 0; i < 3; i++) {
    _ipod_send_command( command );
    delay(10);
  }
  _ipod_send_command( &IPOD_BUTTON_RELEASED );
}

void IpodControl(IpodControl_t event) {

  switch (event) {
    case Ipod_Play:
      ipod_send_command(&IPOD_PLAY);
      break;
    case Ipod_Pause:
      ipod_send_command(&IPOD_PAUSE);
      break;
    case Ipod_Next:
      ipod_send_command(&IPOD_NEXT);
      break;
    case Ipod_Prev:
      ipod_send_command(&IPOD_PREV);
      break;
    case Ipod_On:
      ipod_send_command(&IPOD_ON);
      break;
    case Ipod_Off:
      ipod_send_command(&IPOD_OFF);
      break;
    case Ipod_Shuffle:
      ipod_send_command(&IPOD_SHUFFLE);
      break;
    case Ipod_Repeat:
      ipod_send_command(&IPOD_REPEAT);
      break;
    case Ipod_SeekForward:
      _ipod_send_command(&IPOD_REMOTE_MODE);
      _ipod_send_command(&IPOD_NEXT);
      delay(10);
      _ipod_send_command(&IPOD_NEXT);
      ipod_seeking = true;
      break;
    case Ipod_SeekBackward:
      _ipod_send_command(&IPOD_REMOTE_MODE);
      _ipod_send_command(&IPOD_PREV);
      delay(10);
      _ipod_send_command(&IPOD_PREV);
      ipod_seeking = true;
      break;
    case Ipod_SeekStop:
      _ipod_send_command( &IPOD_BUTTON_RELEASED );
      delay(5);
      _ipod_send_command( &IPOD_BUTTON_RELEASED );
      ipod_seeking = false;
      break;
    default:
      DEBUG_PRINT("Ipod unckown argument");
      break;
  }
}
