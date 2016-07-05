
#define CMD_PLAY        0x01
#define CMD_FASTFORWARD 0x04
#define CMD_REWIND      0x08
#define CMD_STOP        0x60
#define CMD_REPEAT      0x01
#define CMD_RANDOM      0x02
#define CMD_SEEK_UP     0x10
#define CMD_SEEK_DOWN   0x20

const int outPin = 2;
const int interruptPin = 3;

const uint8_t TAPECMD_POWER_ON[] = {0x88, 0x12};
const uint8_t TAPECMD_CASSETE_PRESENT[] = {0x8B, 0x90, 0x40, 0x0C, 0x30};
const uint8_t TAPECMD_STOPPED[] = {0x89, 0x0C, 0xE0};
const uint8_t TAPECMD_PLAING[] = {0x89, 0x41, 0x50};
const uint8_t TAPECMD_PLAYBACK[] = {0x8B, 0x90, 0x40, 0x01, 0x00};
const uint8_t TAPECMD_RANDOM_PLAY[] = {0x8B, 0x90, 0x60, 0x01, 0xE0};
const uint8_t TAPECMD_REPEAT_PLAY[] = {0x8B, 0x90, 0x50, 0x01, 0xF0};
const uint8_t TAPECMD_SEEKING[] = {0x89, 0x51, 0x60};
const uint8_t TAPECMD_FAST_REWIND[] = {0x8B, 0x93, 0x40, 0x11, 0xE0};
const uint8_t TAPECMD_FAST_FORWARD[] = {0x8B, 0x92, 0x40, 0x11, 0xD0};

#define IN_BUFFER_SIZE  3
uint8_t inBuffer[IN_BUFFER_SIZE];
uint8_t bytePos = 0;
uint8_t bitePos = 0;

void setup() {
  pinMode(outPin, OUTPUT);      // sets the digital pin as output

  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), procInputData, CHANGE);
}

void loop() {
  // put your main code here, to run repeatedly:

}

void procInputData() {

  //TODO save time
  uint16_t elapsed_time = 0;
  if( digitalRead(interruptPin) == 0) {
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
void send_message(const uint8_t *message, uint8_t lenght) {
  for (uint8_t i = 0; i < lenght; i++) {
    uint8_t current_message = message[i];

    for (int8_t j = 3; j >= 0 ; j--) {

      // Pull the bus down
      digitalWrite(outPin, LOW);

      //Set Logic 0 or 1 time
      if (current_message & (1 << j)) {
        delayMicroseconds(1700);
      } else {
        delayMicroseconds(500);
      }

      // Release the bus
      digitalWrite(outPin, HIGH);

      //End logic pause
      if (current_message & (1 << j)) {
        delayMicroseconds(1300);
      } else {
        delayMicroseconds(2500);
      }
    }
  }
}

void process_radio_message(uint8_t *message) {
  // Anyone home?
  if (message[0] == 0x08) {
    send_message(TAPECMD_POWER_ON, sizeof(TAPECMD_POWER_ON));
    delay(8);
    send_message(TAPECMD_CASSETE_PRESENT, sizeof(TAPECMD_CASSETE_PRESENT));
  }

  // Wake up!
  if (message[0] == 0x09) {
    send_message(TAPECMD_CASSETE_PRESENT, sizeof(TAPECMD_CASSETE_PRESENT));
    delay(10);
    send_message(TAPECMD_STOPPED, sizeof(TAPECMD_STOPPED));
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
