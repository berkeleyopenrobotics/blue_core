#include <serial/serial.h>
#include <time.h>
#include <iostream>
#include <vector>
#include <comms.h>
#include <BLDCControllerClient.h>
#include <string>
#include <stdio.h>

void sleep(unsigned int milliseconds) {
  struct timespec time_out, remains;

  time_out.tv_sec = 0;
  time_out.tv_nsec = milliseconds * 1000000;
  nanosleep(&time_out, &remains);
}

BLDCControllerClient::BLDCControllerClient(std::string port, unsigned int baud, serial::Timeout to) {
  ser.setPort(port);
  ser.setBaudrate(baud);
  ser.setTimeout(to);
  ser.open();
}

void BLDCControllerClient::writeRequest(uint8_t server_id, uint8_t func_code, bytebuf_t& data) {
  data.at(0) = data.size() - 1;
  data.at(1) = server_id;
  data.at(2) = func_code;
  //TODO: Make this return CRC
  data.push_back(0);
  data.push_back(0);
  ser.write(data);
}

bytebuf_t BLDCControllerClient::readResponse(uint8_t server_id, uint8_t func_code, unsigned int num_tries, unsigned int try_interval) {
  uint8_t lb = 0;
  for (size_t i = 0; i < num_tries && lb == 0; i++) {
    ser.read(&lb, 1);
    sleep(try_interval);
  }
  bytebuf_t empty(0);
  if (lb == 0) {
    std::cout << "lb was zero\n";
    ser.flushInput();
    return empty;
  }

  //TODO: confirm endianness
  uint8_t message_server_id = 0;
  uint8_t message_func_code = 0;
  uint8_t errorsh = 0;
  uint8_t errorsl = 0;

  ser.read(&message_server_id, 1);
  ser.read(&message_func_code, 1);
  ser.read(&errorsl, 1);
  ser.read(&errorsh, 1);
  uint16_t errors = errorsl + (errorsh << 8);
  if (message_server_id != server_id || message_func_code != func_code) {
    std::cout << "Received unexpected server ID or function code\n";
    throw std::string("Received unexpected server ID or function code");
  }
  if (errors) {
    if (errors & COMM_ERRORS_OP_FAILED) {
      std::cout << "operation failed\n";
    }
    if (errors & COMM_ERRORS_MALFORMED) {
      std::cout << "malformed request\n";
    }
    if (errors & COMM_ERRORS_INVALID_FC) {
      std::cout << "invalid function code\n";
    }
    if (errors & COMM_ERRORS_INVALID_ARGS) {
      std::cout << "invalid arguments\n";
    }
    if (errors & COMM_ERRORS_BUF_LEN_MISMATCH) {
      std::cout << "buffer length mismatch\n";
    }
    return empty;
  }

  /** DEBUG **/
  std::cout << "message server id: " << (unsigned int) message_server_id << "\n";
  std::cout << "message func code: " << (unsigned int) message_func_code << "\n";
  std::cout << "errorsh: " << (unsigned int) errorsh << "\n";
  std::cout << "errorsl: " << (unsigned int) errorsl << "\n";
  /** DEBUG **/

  bytebuf_t message;
  message.reserve(lb - 4);
  if (lb - 4 == 0 || ser.read(message, lb - 4) < (size_t) (lb - 4)) {
    std::cout << "message length was not long enough\n";
    ser.flushInput();
    return empty;
  }

  /* uint8_t crc_bytesh = 0; */
  /* uint8_t crc_bytesl = 0; */
  uint16_t crc_bytes = 0;
  if (ser.read(reinterpret_cast<uint8_t*>(&crc_bytes), 2) < 2) {
    std::cout << "crc bytes not read properly\n";
    ser.flushInput();
    return empty;
  }

  // TODO: CRC check
  // Add more errors


  return message;
}

bytebuf_t BLDCControllerClient::doTransaction(uint8_t server_id, uint8_t func_code, bytebuf_t& data) {
  BLDCControllerClient::writeRequest(server_id, func_code, data);
  ser.flushInput();
  return BLDCControllerClient::readResponse(server_id, func_code, 10, 10);
}

bool BLDCControllerClient::writeRegisters(uint8_t server_id, uint16_t start_addr, uint8_t count, uint8_t* data, size_t size) {
  bytebuf_t buffer;
  buffer.reserve(size + 8);
  buffer.push_back(0); buffer.push_back(0); buffer.push_back(0);
  //TODO: confirm endianness
  buffer.push_back(start_addr & 0xFF);
  buffer.push_back(start_addr >> 8);
  buffer.push_back(count);
  for (size_t i = 0; i < size; i++) {
    buffer.push_back(data[i]);
  }
  bytebuf_t message = BLDCControllerClient::doTransaction(server_id, COMM_FC_WRITE_REGS, buffer);
  if (message.size() == 0) {
    return false;
  }
  return true;
}

bytebuf_t BLDCControllerClient::readRegisters(uint8_t server_id, uint16_t start_addr, uint8_t count) {
  bytebuf_t buffer;
  buffer.reserve(8);
  buffer.push_back(0); buffer.push_back(0); buffer.push_back(0);
  //TODO: confirm endianness
  buffer.push_back(start_addr & 0xFF);
  buffer.push_back(start_addr >> 8);
  buffer.push_back(count);

  bytebuf_t data = BLDCControllerClient::doTransaction(server_id, COMM_FC_READ_REGS, buffer);
  if (data.size() == 0) {
    std::cout << "Register read failed\n";
    throw std::string("Register read failed");
  }
  return data;
}

uint16_t BLDCControllerClient::getEncoder(uint8_t id) {
  bytebuf_t message = BLDCControllerClient::readRegisters(id, 0x100, 1);
  uint16_t angle = message.at(0) + (message.at(1) << 8);
  return angle;
}

bool BLDCControllerClient::leaveBootloader(uint8_t server_id, unsigned int jump_addr) {
  uint8_t* src = (uint8_t*) &jump_addr;
  bytebuf_t buffer;
  buffer.reserve(9);
  buffer.push_back(0); buffer.push_back(0); buffer.push_back(0);
  buffer.push_back(src[0]);
  buffer.push_back(src[1]);
  buffer.push_back(src[2]);
  buffer.push_back(src[3]);
  BLDCControllerClient::writeRequest(server_id, COMM_FC_LEAVE_BOOTLOADER, buffer);
  return true;
}

bool BLDCControllerClient::setDuty(uint8_t id, float value) {
  uint8_t* src = (uint8_t*) &value;
  return BLDCControllerClient::writeRegisters(id, 0x0106, 1, src, 4);
}