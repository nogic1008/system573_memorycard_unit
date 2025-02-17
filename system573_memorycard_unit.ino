#include "Arduino.h"
#include <SPI.h>

#define TRANSFER_WAIT 16
#define ACT_WAIT 2500

#ifdef DEBUG
#define debug_begin(...) Serial.begin(__VA_ARGS__)
#define debug(...) Serial.print(__VA_ARGS__)
#define debugln(...) Serial.println(__VA_ARGS__)
#else
#define debug_begin(...)
#define debug(...)
#define debugln(...)
#endif

//Memory Card Responses
//0x47 - Good
//0x4E - BadChecksum
//0xFF - BadSector

#pragma region Pins
#define interruptPin 2  //Attention (Select)
#define AttPin1 3       //Attention (Select)
#define AttPin2 4       //Attention (Select)
// JVS Data- (JVS Serial does not use TX pin)
#define SerialCtlPin 5
// JVS Sense
#define JvsSensePin 6
#pragma endregion

//JVS
#define JvsSerial Serial1
#define JvsMaxPacketSize 256
bool noack = false;
bool init_jvs = false;
byte send_data[256];
bool initDone;
int answer_count = 0;
int req_index = 0;

static uint8_t device_id[48] = { 0x2F, 0x4B, 0x4F, 0x4E, 0x41, 0x4D, 0x49, 0x20, 0x43, 0x4F, 0x2E, 0x2C, 0x4C, 0x54, 0x44, 0x2E, 0x3B, 0x57, 0x68, 0x69, 0x74, 0x65, 0x20, 0x49, 0x2F, 0x4F, 0x3B, 0x56, 0x65, 0x72, 0x31, 0x2E, 0x30, 0x3B, 0x57, 0x68, 0x69, 0x74, 0x65, 0x20, 0x49, 0x2F, 0x4F, 0x20, 0x50, 0x43, 0x42, 0x00 };
static uint8_t jvs_command_info[2] = { 0x01, 0x11 };
static uint8_t jvs_version_info[2] = { 0x01, 0x20 };
static uint8_t jvs_trans_info[2] = { 0x01, 0x20 };
static uint8_t jvs_function_info[2] = { 0x01, 0x00 };

static uint8_t command_OK1[1] = { 0x00 };
static uint8_t command_OK2[2] = { 0x01, 0x01 };
//sec
static uint8_t sec_plate_reg[5] = { 0xFF, 0xFF, 0xAC, 0x09, 0x00 };
static uint8_t sec_plate0_data[5] = { 0x4A, 0x42, 0x00, 0x00, 0x73 };
static uint8_t sec_plate1_data[5] = { 0x4A, 0x43, 0x00, 0x00, 0x72 };
byte sec_plate1_status = 0;
byte sec_plate2_status = 0;
byte select_slot = 0;
//mem
byte select_port = 2;

byte memory_card1_status1 = 0x00;
byte memory_card2_status1 = 0x00;
byte memory_card3_status1 = 0x00;

byte memory_card1_status2 = 0x08;
byte memory_card2_status2 = 0x08;
byte memory_card3_status2 = 0x00;
#define rs485_tx HIGH
#define rs485_rx LOW
#define mode_memory_card 0
#define mode_sec_plate 1
byte buf_mode = 0;

int base_address;
byte pcb_buf[128];
unsigned long time_start;
unsigned long time_end;

namespace PSX {
  enum Device {
    MemoryCard = 0x81,
    Controller = 0x01,
  };

  uint8_t memory_card_buf[128];
  uint8_t controller_buf[4];

  void _attention(int slot, int status) {
    if (slot == 0) {
      digitalWrite(AttPin1, status);
    } else {
      digitalWrite(AttPin2, status);
    }
  }

  /**
   * @brief Setup the pins for the PSX controller
   */
  void pinSetup() {
    SPI.beginTransaction(SPISettings(125000, LSBFIRST, SPI_MODE3));
    pinMode(MISO, OUTPUT);
    pinMode(AttPin1, OUTPUT);
    pinMode(AttPin2, OUTPUT);
    pinMode(interruptPin, INPUT);
  }

  /**
   * @brief Send a command byte to the PSX controller
   * @param[in] command The command byte to send
   * @return The response byte from the controller
   */
  uint8_t sendCommand(uint8_t command = 0x00) {
    uint8_t res = SPI.transfer(command);  // Send the command byte
    delayMicroseconds(16);
    return res;
  }

  int readFrame(int slot, uint16_t address) {
    memset(memory_card_buf, 0x00, 128);
    byte command[30];
    unsigned long time;
    int i = 0;
    time = millis();

    _attention(slot, LOW);
    delayMicroseconds(TRANSFER_WAIT);

    command[0] = PSX::sendCommand(Device::MemoryCard);     // Access Memory Card
    command[1] = PSX::sendCommand('R');                    // Send read command
    command[2] = PSX::sendCommand(0x00);                   // Memory Card ID1
    command[3] = PSX::sendCommand(0x00);                   // Memory Card ID2
    command[4] = PSX::sendCommand((address >> 8) & 0xFF);  // Address MSB
    command[5] = PSX::sendCommand(address & 0xFF);         // Address LSB
    command[6] = PSX::sendCommand(0x00);                   // Memory Card ACK1
    command[7] = PSX::sendCommand(0x00);                   // Memory Card ACK2
    delayMicroseconds(ACT_WAIT);
    command[8] = PSX::sendCommand(0x00);  // Confirm MSB
    command[9] = PSX::sendCommand(0x00);  // Confirm LSB
    //Get 128 byte data from the frame
    for (i = 0; i < 128; i++) {
      memory_card_buf[i] = PSX::sendCommand(0x00);
    }
    command[10] = PSX::sendCommand(0x00);  // Checksum (MSB xor LSB xor Data)
    command[11] = PSX::sendCommand(0x00);  // Memory Card status byte

    //Deactivate device
    _attention(slot, HIGH);

    return (int)command[11];
  }

  /**
   * @brief Initialize the PSX library
   * @param[in] slot The slot to initialize
   */
  int writeFrame(int slot, uint16_t address) {
    int ret = 0;

    _attention(slot, LOW);

    delayMicroseconds(TRANSFER_WAIT);
    PSX::sendCommand(Device::MemoryCard);     // Access Memory Card
    PSX::sendCommand('W');                    // Send write command
    PSX::sendCommand(0x00);                   // Memory Card ID1
    PSX::sendCommand(0x00);                   // Memory Card ID2
    PSX::sendCommand((address >> 8) & 0xFF);  // Address MSB
    PSX::sendCommand(address & 0xFF);         // Address LSB

    delayMicroseconds(ACT_WAIT);

    uint8_t checksum = ((address >> 8) & 0xFF) ^ address & 0xFF;
    //Write 128 byte data to the frame
    for (int i = 0; i < 128; i++) {
      PSX::sendCommand(memory_card_buf[i]);
      checksum ^= memory_card_buf[i];
    }

    PSX::sendCommand(checksum);         // Checksum (MSB xor LSB xor Data)
    PSX::sendCommand(0x00);             // Memory Card ACK1
    PSX::sendCommand(0x00);             // Memory Card ACK2
    ret = (int)PSX::sendCommand(0x00);  // Memory Card status byte
    _attention(slot, HIGH);

    return ret;
  }

  void updateControllerBuffer() {
    byte command[5];
    memset(controller_buf, 0x00, 4);
    memset(command, 0x00, 5);
    _attention(0, LOW);
    delayMicroseconds(TRANSFER_WAIT);
    command[0] = PSX::sendCommand(Device::Controller);
    command[1] = PSX::sendCommand(0x42);
    command[2] = PSX::sendCommand(0x00);
    command[3] = PSX::sendCommand(0x00);
    command[4] = PSX::sendCommand(0x00);
    _attention(0, HIGH);
    if (command[1] == 0x41 && command[2] == 0x5A) {
      controller_buf[0] = command[3];
      controller_buf[1] = command[4];
    }
    memset(command, 0x00, 5);
    _attention(1, LOW);
    delayMicroseconds(TRANSFER_WAIT);
    command[0] = PSX::sendCommand(Device::Controller);
    command[1] = PSX::sendCommand(0x42);
    command[2] = PSX::sendCommand(0x00);
    command[3] = PSX::sendCommand(0x00);
    command[4] = PSX::sendCommand(0x00);
    _attention(1, HIGH);
    if (command[1] == 0x41 && command[2] == 0x5A) {
      controller_buf[2] = command[3];
      controller_buf[3] = command[4];
    }
  }

  int findSlotMemoryCard(int slot) {
    int ret = 0;
    byte command[4];
    _attention(slot, LOW);
    delayMicroseconds(TRANSFER_WAIT);

    command[0] = PSX::sendCommand(Device::MemoryCard);  // Access Memory Card
    command[1] = PSX::sendCommand('R');                 // Send read command
    command[2] = PSX::sendCommand(0x00);                // Memory Card ID1
    command[3] = PSX::sendCommand(0x00);                // Memory Card ID2

    _attention(slot, HIGH);
    if (command[2] == 0x5A) {
      ret = 01;
    }
    return ret;
  }
}

namespace JVS {
  enum Command {
    Reset = 0xF0,
    SetAddress = 0xF1,
    IOId = 0x10,
    CommandRev = 0x11,
    JvRev = 0x12,
    ProtocolVer = 0x13,
    FunctionCheck = 0x14,
    Retry = 0x2F,

    K573ControlBuffer = 0x70,
    K573Status = 0x71,
    K573SecurityPlate = 0x72,
    K573Firmware = 0x73,
    K573MemoryCard = 0x76,
    K573Controller = 0x77,
  };

  enum SpecialChar {
    Sync = 0xE0,
    Escape = 0xD0,
  };

  bool escaped;
  int req_i;
  uint8_t requestPacket[JvsMaxPacketSize];
  uint8_t acknowledgePacket[JvsMaxPacketSize];

  void getRequestPacket() {
    byte inByte = 0x00;
    bool is_req_ok = false;

    if (JvsSerial.available() > 0) {
      is_req_ok = true;
      inByte = JvsSerial.read();
    }

    if (is_req_ok == true) {
      if (inByte <= 0xF) {
        debug("0");
      }
      debug(inByte, HEX);
      debug(" ");
      if (inByte == JVS::SpecialChar::Sync) {
        escaped = false;
        req_i = 0;
      }
      if (inByte == JVS::SpecialChar::Escape) {
        escaped = true;
      }
      if (inByte != JVS::SpecialChar::Sync && inByte != JVS::SpecialChar::Escape) {
        if (escaped) {
          inByte = inByte + 1;
          escaped = false;
        }
        requestPacket[req_i] = inByte;
        req_i++;
      }
    }
  }

  bool requestCompleted() {
    if (req_i >= 4) {
      if (req_i == (2 + JVS::requestPacket[1])) {
        time_start = millis();
        debugln("");
        return true;
      }
    }
    return false;
  }

  bool validateRequest() {
    byte sum = 0;
    int bufsize = 2 + JVS::requestPacket[1];
    for (int i = 0; i < bufsize - 1; i++) {
      sum += JVS::requestPacket[i];
    }
    return (sum == JVS::requestPacket[bufsize - 1]);
  }

  void returnAckPacket(const byte* addr, byte len) {
    time_end = millis();
    digitalWrite(SerialCtlPin, HIGH);
    JvsSerial.write(addr, len);
    JvsSerial.flush();  // Wait for the transmission to complete
    digitalWrite(SerialCtlPin, LOW);

    debug(time_end - time_start, DEC);
    debugln("");
    for (int i = 0; i < len; i++) {
      if (addr[i] < 0x10) {
        debug("0");
      }
      debug(addr[i], HEX);
      debug(":");
    }
    debugln("");
    debugln("");
  }
}

void setup() {
  memset(JVS::requestPacket, 0x00, 256);
  memset(JVS::acknowledgePacket, 0x00, 256);
  memset(pcb_buf, 0x00, 128);

  debug_begin(115200);
  JvsSerial.begin(115200);

  pinMode(SerialCtlPin, OUTPUT);
  pinMode(JvsSensePin, INPUT);
  digitalWrite(JvsSensePin, LOW);
  digitalWrite(SerialCtlPin, LOW);
  PSX::pinSetup();
  initDone = true;
  debugln("START");
}

void loop() {
  digitalWrite(LED_BUILTIN, LOW);
  serialEvent3_();
}

void serialEvent3_() {
  if (initDone) {
    JVS::getRequestPacket();
  }

  if (JVS::requestCompleted()) {
    digitalWrite(LED_BUILTIN, HIGH);
    if (JVS::validateRequest()) {
      processRequest();

      req_i = 0;
      memset(JVS::requestPacket, 0x00, 256);
    } else {
      req_i = 0;
    }
  }
}

void processRequest() {
  memset(JVS::acknowledgePacket, 0x00, 256);
  processRequest_(JVS::requestPacket, JVS::acknowledgePacket);
  if (noack == false) {
    sendAnswer(JVS::acknowledgePacket);
  } else {
    noack = false;
  }
  if (init_jvs) {
    pinMode(JvsSensePin, OUTPUT);
  }
}

void sendAnswer(byte* answer) {
  byte sum = 0;
  int bufsize = 2 + answer[1];

  memset(send_data, 0x00, 256);
  for (int i = 0; i < bufsize - 1; i++) {
    sum += answer[i];
  }
  answer_count = 0;
  answer[bufsize - 1] = sum;
  send_data[0] = JVS::SpecialChar::Sync;
  for (int i = 0; i < bufsize + 2; i++) {
    byte outByte = answer[i];
    if (outByte == JVS::SpecialChar::Sync || outByte == JVS::SpecialChar::Escape) {

      outByte = outByte - 1;
      send_data[answer_count + 1] = JVS::SpecialChar::Escape;
      answer_count++;
    }
    send_data[answer_count + 1] = outByte;
    answer_count++;
  }

  JVS::returnAckPacket(send_data, answer_count - 1);
}

void sec_plate(byte* request, byte* answer) {
  debug("SEC_PLATE_COMMAND::");
  if (request[req_index + 1] != 0x40 && request[req_index + 1] != 0x20 && request[req_index + 1] != 0x10) {
    req_index = req_index + 2;
    debug("SLOT-");
    select_slot = (byte)(request[req_index + 1] & 0x01);
    debugln(select_slot, HEX);
  }
  if (select_slot == 0x00) {
    sec_plate1_status = 0x00;
  } else {
    sec_plate2_status = 0x00;
  }
  if (request[req_index + 1] == 0x10) {
    req_index = req_index + 10;
    debugln("SET_PASS::");
  }
  if (request[req_index + 1] == 0x20) {
    req_index = req_index + 9;
    memset(pcb_buf, 0x00, 128);
    buf_mode = mode_sec_plate;
    if (select_slot == 0x00) {
      debugln("TRANS_DATA_0::");
      memcpy(pcb_buf, sec_plate0_data, 5);
    } else {
      debugln("TRANS_DATA_1::");
      memcpy(pcb_buf, sec_plate1_data, 5);
    }
  }
  if (request[req_index + 1] == 0x40) {
    req_index = req_index + 5;
    debugln("TRANS_REG::");
    buf_mode = mode_sec_plate;
    memset(pcb_buf, 0x00, 128);
    memcpy(pcb_buf, sec_plate_reg, 5);
  }
  make_responce(answer, command_OK1);
}

void control_buf(byte* request, byte* answer) {
  debug("CONTROL_BUF::");
  int offset = ((request[req_index + 3] & 0x7F) << 1) + ((request[req_index + 4] & 0x80) >> 7);
  int len = request[req_index + 5];
  if (request[req_index + 1] == 0) {
    if (buf_mode == mode_memory_card) {
      debug("MODE:MEMORY_CARD::");
      memset(PSX::memory_card_buf, 0x00, 128);
      if (PSX::findSlotMemoryCard(select_port) == 1) {
        debugln("CARD_FIND::");
        if (PSX::readFrame(select_port, base_address + offset) == 0x47) {

          debugln("READ_OK");
          memset(pcb_buf, 0x00, 128);
          memcpy(pcb_buf, PSX::memory_card_buf, 128);
        } else {
          debugln("READ_NG");
        }
      } else {
        debugln("NO_INSERT");
      }
    }

    debug("PCB_BUF_READ::base_address=");
    debug(base_address, HEX);
    debug(":offset=");
    debug(offset, HEX);
    debug(":LEN=");
    debug(len, HEX);
    debug(":port=");
    debug(select_port, HEX);
    debug(":MODE=");
    debugln(buf_mode, HEX);

    byte buf_data[128 + 1];
    buf_data[0] = len;
    memcpy(buf_data + 1, pcb_buf, len);
    req_index = req_index + 6;
    make_responce(answer, buf_data);
  }
  if (request[req_index + 1] == 1) {
    debug("PCB_BUF_WRITE::base_address=");
    debug(base_address, HEX);
    debug(":LEN=");
    debugln(len, HEX);
    memset(pcb_buf, 0x00, 128);
    for (int i = 0; i < len; i++) {
      pcb_buf[i] = request[(req_index + 6 + i)];
    }
    req_index = req_index + 6 + len;
    make_responce(answer, command_OK1);
  }
  if (request[req_index + 1] == 2) {
    debugln("PCB_BUF_UNKNOWN::");
    req_index = req_index + 5;
    make_responce(answer, command_OK1);
  }
}

void memory_card_function(byte* request, byte* answer) {
  if (request[req_index + 1] == 0x74) {
    debugln("MEMORY_CARD_READ_TRANS_BUF::");
    buf_mode = mode_memory_card;
    if ((request[req_index + 2] & 0xF0) == 0x00) {
      debugln("PORT-00::");
      select_port = 0;
    } else {
      debugln("PORT-01::");
      select_port = 1;
    }

    byte base_address_H = (request[req_index + 2] & 0x0F);
    byte base_address_L = request[req_index + 3];

    base_address = (base_address_H << 8) + base_address_L;
    debug("base_address::");
    debug(base_address, HEX);
    debug(":");
    if (PSX::findSlotMemoryCard(select_port) == 1) {
      if (PSX::readFrame(select_port, base_address) == 0x47) {
        memset(pcb_buf, 0x00, 128);
        memcpy(pcb_buf, PSX::memory_card_buf, 128);

        debugln("state::READ_OK");
        if (select_port == 0) {
          memory_card1_status1 = 0x80;
          memory_card1_status2 = 0x00;
        } else {
          memory_card2_status1 = 0x80;
          memory_card2_status2 = 0x00;
        }
      } else {
        debugln("state::READ_NG");
        if (select_port == 0) {
          memory_card1_status1 = 0x00;
          memory_card1_status2 = 0x00;
        } else {
          memory_card2_status1 = 0x00;
          memory_card2_status2 = 0x00;
        }
      }
    } else {
      debugln("state::NO_INSERT");
      if (select_port == 0) {
        memory_card1_status1 = 0x00;
        memory_card1_status2 = 0x08;
      } else {
        memory_card2_status1 = 0x00;
        memory_card2_status2 = 0x08;
      }
    }
  }
  if (request[req_index + 1] == 0x75) {
    debugln("MEMORY_CARD_WRITE::");
    if ((request[req_index + 5] & 0xF0) == 0x00) {
      debugln("PORT-00::");
      select_port = 0;
    } else {
      debugln("PORT-01::");
      select_port = 1;
    }
    byte base_address_H = (byte)(request[req_index + 5] & 0x0F);
    byte base_address_L = request[req_index + 6];
    int address = (base_address_H << 8) + base_address_L;
    debug("write_address::");
    debug(address, HEX);
    debug(":");
    memset(PSX::memory_card_buf, 0x00, 128);
    memcpy(PSX::memory_card_buf, pcb_buf, 128);
    if (PSX::writeFrame(select_port, address) == 0x47) {
      debugln("state::WRITE_OK");
      if (select_port == 0) {
        memory_card1_status1 = 0x80;
        memory_card1_status2 = 0x00;
      } else {
        memory_card2_status1 = 0x80;
        memory_card2_status2 = 0x00;
      }
    } else {
      debugln("state::WRITE_NG");
      if (select_port == 0) {
        memory_card1_status1 = 0x00;
        memory_card1_status2 = 0x00;
      } else {
        memory_card2_status1 = 0x00;
        memory_card2_status2 = 0x00;
      }
    }
  }
  req_index = req_index + 9;
  make_responce(answer, command_OK2);
}

void make_status(byte* request, byte* answer) {
  debug("STATUS_CHECK::");
  byte status1 = 0x00;
  byte status2 = 0x00;

  if (select_port == 0) {
    debugln("PORT-00::");
    status1 = memory_card1_status1;
    status2 = memory_card1_status2;
  }
  if (select_port == 1) {

    debugln("PORT-01::");
    status1 = memory_card2_status1;
    status2 = memory_card2_status2;
  }
  if (select_port == 2) {

    debugln("PORT---");
    status1 = memory_card3_status1;
    status2 = memory_card3_status2;
  }
  if (select_slot == 0) {
    debugln("slot-00::");
    status2 = status2 | sec_plate1_status;
  }
  if (select_slot == 1) {
    debugln("slot-01::");
    status2 = status2 | sec_plate2_status;
  }
  byte state[3];
  state[0] = 0x02;
  state[1] = status1;
  state[2] = status2;
  req_index = req_index + 1;
  make_responce(answer, state);
}
void make_responce(byte* answer, byte* res) {
  int index = answer[1];
  answer[index + 2] = 0x01;
  answer[1] = answer[1] + res[0] + 1;
  for (int i = 0; i < res[0]; i++) {
    answer[index + 3 + i] = res[i + 1];
  }
}

void processRequest_(byte* request, byte* answer) {

  req_index = 0;
  memset(answer, 0x00, 256);
  answer[0] = 0;  // node id
  answer[1] = 1;  // res_count
  answer[2] = 1;  // status1

  req_index = 2;

  while (request[1] >= req_index) {
    switch (request[req_index]) {
      case JVS::Command::Reset:
        req_index = req_index + 2;
        pinMode(JvsSensePin, INPUT);

        select_slot = 0;
        select_port = 2;
        base_address = 0;
        noack = true;
        init_jvs = false;
        break;
      case JVS::Command::SetAddress:
        debugln("JVS_SET_DEV_ID");
        req_index = req_index + 2;
        make_responce(answer, command_OK1);
        base_address = 0;
        init_jvs = true;
        break;
      case JVS::Command::Retry:
        debugln("JVS_RESEND");
        req_index = req_index + 1;
        JVS::returnAckPacket(send_data, answer_count - 1);
        noack = true;
        break;
      case JVS::Command::IOId:
        debugln("JVS_GET_DEV_ID");
        req_index = req_index + 1;
        make_responce(answer, device_id);
        break;
      case JVS::Command::CommandRev:
        debugln("JVS_GET_COMMAND_REV");
        req_index = req_index + 1;
        make_responce(answer, jvs_command_info);
        break;
      case JVS::Command::JvRev:
        debugln("JVS_GET_VERSION");
        req_index = req_index + 1;
        make_responce(answer, jvs_version_info);
        break;
      case JVS::Command::ProtocolVer:
        debugln("JVS_GET_TRANS_VERSION");
        req_index = req_index + 1;
        make_responce(answer, jvs_trans_info);
        break;
      case JVS::Command::FunctionCheck:
        debugln("JVS_GET_FUNCTION_INFO");
        req_index = req_index + 1;
        make_responce(answer, jvs_function_info);
        break;
      case JVS::Command::K573ControlBuffer:
        control_buf(request, answer);
        break;
      case JVS::Command::K573Status:
        make_status(request, answer);
        break;
      case JVS::Command::K573SecurityPlate:
        sec_plate(request, answer);
        break;
      case JVS::Command::K573Firmware:
        debugln("PCB_FW_WRITE_DONE");
        req_index = req_index + 1;
        make_responce(answer, command_OK1);
      case JVS::Command::K573MemoryCard:
        memory_card_function(request, answer);
        break;
      case JVS::Command::K573Controller:
        debugln("PS_CONTROLLER_GET");
        req_index = req_index + 1;
        PSX::updateControllerBuffer();
        byte ps_data[5];
        ps_data[0] = 4;
        ps_data[1] = PSX::controller_buf[0];
        ps_data[2] = PSX::controller_buf[1];
        ps_data[3] = PSX::controller_buf[2];
        ps_data[4] = PSX::controller_buf[3];
        make_responce(answer, ps_data);
        break;
      default:
        req_index = req_index + 256;
        debugln("UNKNOWN");
        byte null_data[1];
        null_data[0] = 0x00;
        make_responce(answer, null_data);
        break;
    }
  }

  answer[1] = answer[1] + 1;  // data_size + sum
}
