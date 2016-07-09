
#define CMD_PLAY        0x01
#define CMD_FASTFORWARD 0x04
#define CMD_REWIND      0x08
#define CMD_STOP        0x60
#define CMD_REPEAT      0x01
#define CMD_RANDOM      0x02
#define CMD_SEEK_UP     0x10
#define CMD_SEEK_DOWN   0x20

const int outPin = 3;
const int interruptPin = 2;

typedef struct {
  uint8_t lenght;
  uint8_t data[16];
} tape_cmd_t;

const tape_cmd_t TAPECMD_POWER_ON = {0x04, {0x88, 0x12}};
const tape_cmd_t TAPECMD_CASSETE_PRESENT = {0x09, {0x8B, 0x90, 0x40, 0x0C, 0x30}};
const tape_cmd_t TAPECMD_STOPPED = {0x05, {0x89, 0x0C, 0xE0}};
const tape_cmd_t TAPECMD_PLAING = {0x05, {0x89, 0x41, 0x50}};
const tape_cmd_t TAPECMD_PLAYBACK = {0x09, {0x8B, 0x90, 0x40, 0x01, 0x00}};
const tape_cmd_t TAPECMD_RANDOM_PLAY = {0x09, {0x8B, 0x90, 0x60, 0x01, 0xE0}};
const tape_cmd_t TAPECMD_REPEAT_PLAY = {0x09, {0x8B, 0x90, 0x50, 0x01, 0xF0}};
const tape_cmd_t TAPECMD_SEEKING = {0x05, {0x89, 0x51, 0x60}};
const tape_cmd_t TAPECMD_FAST_REWIND = {0x09, {0x8B, 0x93, 0x40, 0x11, 0xE0}};
const tape_cmd_t TAPECMD_FAST_FORWARD = {0x09, {0x8B, 0x92, 0x40, 0x11, 0xD0}};

#define IN_BUFFER_SIZE  3
uint8_t inBuffer[IN_BUFFER_SIZE];
uint8_t bytePos = 0;
uint8_t bitePos = 0;

void setup() {
  //pinMode(outPin, OUTPUT);      // sets the digital pin as output
  pinMode(outPin, INPUT_PULLUP);

  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), procInputData, CHANGE);

  delay(10000);
}

void loop() {
  // put your main code here, to run repeatedly:

  static int init = 0;

  if (init == 0) {
    send_message(&TAPECMD_POWER_ON);
    delay(1000);


    send_message(&TAPECMD_CASSETE_PRESENT);
    delay(1000);
    init = 1;
  }
  else {
    delay(60000);
    init = 0;
  }
}

void procInputData() {

  //TODO save time
  uint16_t elapsed_time = 0;
  if ( digitalRead(interruptPin) == 0) {
    //TODO reset timer;
    return;
  }

  if (elapsed_time > 22 && elapsed_time < 32) {
    inBuffer[bytePos] |= 1 << (7 - bitePos);
  }

  ++bitePos;

  if (bitePos >= 8) {
    bitePos = 0;
    ++bytePos;
  }

  if (bytePos >= IN_BUFFER_SIZE) {
    bytePos = 0;

    //Clear buffer
    for (uint8_t i = 0; i < IN_BUFFER_SIZE; i++) {
      inBuffer[i] = 0;
    }
  }
}

// Send a message on the Mazda radio bus
void send_message(const tape_cmd_t *message) {
  uint8_t bit_value = 0;
  uint8_t array_len = (message->lenght >> 1);
  uint8_t current_message = 0;

  for (uint8_t i = 0; i < array_len; i++) {
    current_message = message->data[i];

    pinMode(outPin, OUTPUT);

    for (int8_t j = 7; j >= 0 ; j--) {

      // Pull the bus down
      digitalWrite(outPin, LOW);

      bit_value = current_message & (1 << j);

      //Set Logic 0 or 1 time
      if (bit_value) {
        delayMicroseconds(1780);
      } else {
        delayMicroseconds(600);
      }

      // Release the bus
      digitalWrite(outPin, HIGH);

      //End logic pause
      if (bit_value) {
        delayMicroseconds(1200);
      } else {
        delayMicroseconds(2380);
      }
    }
  }

  if (message->lenght & 0x01) {

    current_message = message->data[array_len];

    for (int8_t j = 7; j >= 4 ; j--) {

      // Pull the bus down
      digitalWrite(outPin, LOW);

      bit_value = current_message & (1 << j);

      //Set Logic 0 or 1 time
      if (bit_value) {
        delayMicroseconds(1780);
      } else {
        delayMicroseconds(600);
      }

      // Release the bus
      digitalWrite(outPin, HIGH);

      //End logic pause
      if (bit_value) {
        delayMicroseconds(1200);
      } else {
        delayMicroseconds(2380);
      }
    }
  }

  pinMode(outPin, INPUT_PULLUP);
}


void process_radio_message(uint8_t *message) {
  // Anyone home?
  if (message[0] == 0x08) {
    send_message(&TAPECMD_POWER_ON);
    delay(8);
    send_message(&TAPECMD_CASSETE_PRESENT);
  }

  // Wake up!
  if (message[0] == 0x09) {
    send_message(&TAPECMD_CASSETE_PRESENT);
    delay(10);
    send_message(&TAPECMD_STOPPED);
  }

  // Control command
  if (message[0] == 0x01) {

    // Extract the specific subcommand and command
    uint8_t subcommand = message[1] >> 4;
    uint8_t command = (message[1] << 4) | (message[2] >> 4);

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
}
