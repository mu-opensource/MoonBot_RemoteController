/*
 * MoonBot_RemotPotocolAnalysis.cpp
 *
 *  Created on: 2019年4月22日
 *      Author: ysq
 */

#include <MoonBot_RemotePotocolAnalysis.h>

MoonBotRemoteProtocolAnalysis::MoonBotRemoteProtocolAnalysis(MuVsUart* uart,
                                                             uint32_t address,
                                                             bool response_enable)
    : MuVisionSensorUart(uart, address),
      response_enable_(response_enable){
}

MoonBotRemoteProtocolAnalysis::~MoonBotRemoteProtocolAnalysis(void) {}

void MoonBotRemoteProtocolAnalysis::responseEnable(bool state) {
  response_enable_ = state;
}

int MoonBotRemoteProtocolAnalysis::capture(void) {
  if (uart_->available() == 0) {
    return MU_ERROR_FAIL;
  }
  mu_err_t err;
  err = GetProtocolHead(buffer_);
  if (err != MU_OK) {
    EPRINTF("GetProtocolHead error:0x%02x\n", err);
    return err;
  } else if (buffer_[2] != mu_address_) {
    EPRINTF("GetProtocolHead error:0x%02x\n", MU_ERROR_FAIL);
    return MU_ERROR_FAIL;
  }
  err = GetProtocolBody(buffer_);
  if (err != MU_OK) {
    EPRINTF("GetProtocolBody error:0x%02x\n", err);
    return err;
  }
  IPRINTF("Get cmd:0x%02x, sub-cmd:0x%02x\n", cmd_, sub_cmd_);
  cmdParameterTest();

  return MU_OK;
}

void MoonBotRemoteProtocolAnalysis::responseError(uint8_t err) {
  if (response_enable_) {
    uint8_t err_buf[] = {MU_PROTOCOL_START,0,(uint8_t)mu_address_,err,0,MU_PROTOCOL_END};
    err_buf[1] = sizeof(err_buf);
    err_buf[err_buf[1]-2] = SumCheck(err_buf, err_buf[1]-2);
    UartWrite(err_buf, err_buf[1]);
#if MR_DEBUG
    printf("\e[0;31mE:ackError:0x%x: ", err);
    for (int i = 0; i < err_buf[1]; ++i) {
      printf("0x%x,", err_buf[i]);
    }
    printf("\n\e[0m");
#endif
  }
}

void MoonBotRemoteProtocolAnalysis::response(uint8_t cmd, uint8_t sub_cmd) {
    if (response_enable_) {
    buffer_[0] = MU_PROTOCOL_START;
    buffer_[1] = 8+parameter_offset_;
    buffer_[2] = (uint8_t)mu_address_;
    buffer_[3] = MU_ERROR_OK;
    buffer_[4] = cmd;
    buffer_[5] = sub_cmd;
    buffer_[6+parameter_offset_] = SumCheck(buffer_, buffer_[1]-2);
    buffer_[7+parameter_offset_] = MU_PROTOCOL_END;
    UartWrite(buffer_, buffer_[1]);
#if MR_DEBUG
    printf("\e[0;32mI:ackOK[0x%x, 0x%x] ", cmd, sub_cmd);
    for (int i = 0; i < buffer_[1]; ++i) {
      printf("0x%x,", buffer_[i]);
    }
    printf("\n\e[0m");
#endif
  }
}

void MoonBotRemoteProtocolAnalysis::cmdParameterTest(void) {
#if MR_DEBUG
  if (buffer_[1] < 8) {
    printf("Error command:");
    for (uint8_t i = 0; i < buffer_[1]; ++i) {
      printf("0x%02x ", buffer_[i]);
    }
    printf("\n");
    return;
  }
  uint8_t parameter_len = buffer_[1]-8;
  printf("paramter length: %hhu byte, id: 0x%02x\n",
         parameter_len, id_);
  if (parameter_len == 0) return;
  printf("paramter(uint8_t): ");
  resetParameterIndex();
  for (uint8_t i = 0; i < parameter_len-sizeof(uint8_t)+1; ++i) {
    printf("%hu,", getParameter<uint8_t>());
  }
  printf("\nparamter(uint16_t): ");
  resetParameterIndex();
  for (uint8_t i = 0; i < parameter_len-sizeof(uint16_t)+1; ++i) {
    printf("%u,", getParameter<uint16_t>());
    parameter_offset_ = parameter_offset_+1-sizeof(uint16_t);
  }
  printf("\nparamter(int8_t): ");
  resetParameterIndex();
  for (uint8_t i = 0; i < parameter_len-sizeof(int8_t)+1; ++i) {
    printf("%hd,", getParameter<int8_t>());
  }
  printf("\nparamter(int16_t): ");
  resetParameterIndex();
  for (uint8_t i = 0; i < parameter_len-sizeof(int16_t)+1; ++i) {
    printf("%d,", getParameter<int16_t>());
    parameter_offset_ = parameter_offset_+1-sizeof(int16_t);
  }
  printf("\n");
#endif
}
