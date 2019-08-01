/*
 * MoonBot_RemotPotocolAnalysis.h
 *
 *  Created on: 2019年4月22日
 *      Author: ysq
 */

#ifndef MOONBOT_REMOTCONTROLLER_MOONBOT_REMOTEPOTOCOLANALYSIS_H_
#define MOONBOT_REMOTCONTROLLER_MOONBOT_REMOTEPOTOCOLANALYSIS_H_


#include <MoonBot_RemoteTypes.h>
#include "mu_vision_sensor_uart_hw_interface.h"

class MoonBotRemoteProtocolAnalysis : public MuVisionSensorUart {
 public:
  MoonBotRemoteProtocolAnalysis(MuVsUart* uart,
                                uint32_t address,
                                bool response_enable = true);
  virtual ~MoonBotRemoteProtocolAnalysis(void);
  MoonBotRemoteProtocolAnalysis& operator=(const MoonBotRemoteProtocolAnalysis &) = delete;
  MoonBotRemoteProtocolAnalysis(const MoonBotRemoteProtocolAnalysis&) = delete;

  void responseEnable(bool state);

  /*
   * Protocol format:
   * No.    |   0   |    1   |    2    |    3    |      4     | 5  |     5+N      |  6+N  | 7+N  |
   * Brief  | Start | Length | Address | Command | SubCommand | ID | Parameter    | Check | End  |
   * Value  | 0xFF  | 8+N    | Address | Command | SubCommand | ID | Data1..DataN | Check | 0xED |
   */
  int capture(void);
  void responseError(uint8_t err);
  void response(uint8_t cmd, uint8_t sub_cmd);

  /*
   * @brief: get/set value from transmit buffer
   * example:
   *  uint8_t appButtonDrag() {
   *    resetParameterIndex();                        // reset parameter pointer to first parameter address
   *    uint16_t angle = getParameter<uint16_t>();    // get first parameter
   *    uint16_t speed = getParameter<uint16_t>();    // get second parameter
   *    ...
   *    resetParameterIndex();                        // reset parameter pointer to first parameter address
   *    // add parameter here if need response parameters
   *    // setParameter<[parameter_type]>([parameter]); // set parameter
   *    return MU_OK;                                 // response MU_OK with no parameters
   *  }
   */
  void resetParameterIndex(void) {
    parameter_offset_ = 0;
  }
  int getUnuseParamterLen(void) {
    return (int)buffer_[1]-8-parameter_offset_;
  }
  template<class T>
  T getParameter(void) {
    size_t len = sizeof(T);
    // Little-Endian
    uint32_t data = 0;
    for (size_t i = len; i != 0; --i) {
      data |= parameter_[parameter_offset_++]<<((i-1)*8);
    }
    T* ret = (T*)&data;
    // Big-Endian
//    T* ret = (T*)&parameter_[parameter_offset_];
//    parameter_offset_ += len;
    return *ret;
  }
  template<class T>
  void setParameter(const T parameter) {
    size_t len = sizeof(T);
    for (size_t i = len; i != 0; --i) {
      parameter_[parameter_offset_++] = (parameter>>((i-1)*8)) & 0xFF;
    }
  }
  uint8_t getCMD(void) {return cmd_;}
  uint8_t getSubCMD(void) {return sub_cmd_;}
  uint8_t getID(void) {return id_;}

  inline void cmdParameterTest(void);

 private:
 protected:
  uint8_t buffer_[MR_CMD_BUFFER_LEN];
  const uint8_t& cmd_ = buffer_[3];
  const uint8_t& sub_cmd_ = buffer_[4];
  const uint8_t& id_ = buffer_[5];
  uint8_t* const parameter_ = &buffer_[6];
  int parameter_offset_ = 0;
  bool response_enable_ = true;
};


#endif /* MOONBOT_REMOTCONTROLLER_MOONBOT_REMOTEPOTOCOLANALYSIS_H_ */
