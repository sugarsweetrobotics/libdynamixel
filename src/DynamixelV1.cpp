/***************************************************
 * Dynamixel.cpp
 *
 * created 2010/04/20
 * author Yuki Suga (Revast Co., Ltd.)
 * email  ysuga@revast.co.jp
 * copyright Revast Co., Ltd. 2010 all rights reserved.
 ***************************************************/

#include "DynamixelV1.h"

using namespace ssr;
using namespace ssr::dynamixel;

Dynamixel::Dynamixel(const char* comport, const long baudrate) : m_SerialPort(comport, baudrate)
{
}

Dynamixel::~Dynamixel(void)
{
}

void Dynamixel::WritePacket (uint8_t cID, TInstruction cInst,  uint8_t *pParam, int32_t iLength) {
  int32_t sum; 
  uint8_t   buf[200]; 
  unsigned long   wlen; 


  if ((cID > PACKET_MAX_ID) || (iLength > PACKET_MAX_DATASIZE)) {
    throw WrongArgException();
  }

  buf[0] = PACKET_HEADER_FIXED_VALUE0; // fixed
  buf[1] = PACKET_HEADER_FIXED_VALUE1; // fixed     
  buf[2] = sum = cID; // ID
  buf[3] = (uint8_t)(iLength + 2); // data length
  sum += buf[3];
  buf[4] = cInst; // Instruction code.
  sum += buf[4];
  for (int32_t i = 0; i < iLength; i++) { // setting parameter.
    sum += (buf[i + 5] = pParam[i]);
  }
  buf[iLength + 5] = (uint8_t)(~(sum) & 0xFF); // summation check.

  //m_SerialPort.PurgeRxClear();
  m_SerialPort.FlushRxBuffer();
  wlen = m_SerialPort.Write(buf, iLength + 6);

  if(wlen != iLength+6) {
    throw WritePacketException();
  }

}



void Dynamixel::ReceivePacket (uint8_t *pRcv, int32_t *pLength, int32_t mask, int32_t timeout /* [ms] */)
{
  int32_t ind;
  uint8_t           sum;
  uint8_t dataLength;

  *pLength = 0;

  // Wait for receive header 4 bytes.
  m_Timer.tick();
  while (1) {
    TimeSpec time;
    m_Timer.tack(&time);

    if (m_SerialPort.GetSizeInRxBuffer() >= PACKET_HEADER_SIZE) {
      break;
    }
		
    if (time.getUsec() > timeout * 1000) {
      throw TimeOutException();
    }
  }

  // Receiving Header 4 bytes.
  if(m_SerialPort.Read(pRcv, PACKET_HEADER_SIZE) != PACKET_HEADER_SIZE) {
    throw ReceivePacketException();
  }
  *pLength = PACKET_HEADER_SIZE;

  // Confirming valid header.
  if((pRcv[0] == PACKET_HEADER_FIXED_VALUE0) && (pRcv[1] == PACKET_HEADER_FIXED_VALUE1) 
     && (pRcv[2] <= PACKET_MAX_ID) && (pRcv[3] <= PACKET_MAX_DATASIZE)) {
    ind = PACKET_HEADER_SIZE;
    sum = pRcv[2] + pRcv[3];
    dataLength = pRcv[3];
  } else {
    // invalid header.
    throw ReceivePacketException();
  }


  // Waiting for receiving data.
  while (1) {
    TimeSpec time;
    m_Timer.tack(&time);

    if (m_SerialPort.GetSizeInRxBuffer() >= dataLength) {
      break;
    }
    if (time.getUsec() > timeout*1000) {
      throw TimeOutException();
    }
  }

  // Receive data.
  if(m_SerialPort.Read(&pRcv[PACKET_HEADER_SIZE], dataLength) != dataLength) {
    throw ReceivePacketException();
  }

  uint8_t error = pRcv[PACKET_HEADER_SIZE];
  if(error & INPUT_VOLTAGE_ERROR_FLAG & mask) {
    throw InputVoltageException();
  } else if (error & ANGLE_LIMIT_ERROR_FLAG & mask) {
    throw AngleLimitException();
  } else if (error & OVERHEATING_ERROR_FLAG & mask) {
    throw OverheatingException();
  } else if (error & RANGE_ERROR_FLAG & mask) {
    throw RangeException();
  } else if (error & CHECKSUM_ERROR_FLAG & mask) {
    throw ChecksumException();
  } else if (error & OVERLOAD_ERROR_FLAG & mask) {
    throw OverloadException();
  } else if (error & INSTRUCTION_ERROR_FLAG & mask) {
    throw InstructionException();
  }


  for(int32_t i = 0;i < dataLength-1;i++) {
    sum += pRcv[PACKET_HEADER_SIZE + i];
  }
  if(((~sum) & 0xFFU) != pRcv[PACKET_HEADER_SIZE + dataLength -1]) {
    throw ReceivePacketException();
  }
}


void Dynamixel::WriteByteData (uint8_t id, uint8_t adr, uint8_t dat, int32_t mask, int32_t timeout) {
  int32_t l1;
  uint8_t param[10];
  uint8_t rbuf[10];

  param[0] = adr;
  param[1] = dat;

  ssr::MutexBinder m(m_Mutex);

  int32_t i = 0;
 write_byte_data_try:
  try {
    WritePacket (id, INST_WRITE, param, 2);
    if (id != BROADCASTING_ID) 
      ReceivePacket (rbuf, &l1, mask, timeout);
  } catch (TimeOutException &e) {
    i++;
    if(i < DYNAMIXEL_RECEIVEPACKET_TRYCYCLE) {
      goto write_byte_data_try;
    } else {
      throw e;
    }
  }
}


void Dynamixel::ReadByteData (uint8_t id, uint8_t adr, uint8_t *result, int32_t mask, int32_t timeout) {
  int32_t             l1;
  uint8_t           param[10];
  uint8_t           rbuf[10];

  if (id == BROADCASTING_ID) {
    throw new WrongArgException();
  }

  ssr::MutexBinder m(m_Mutex);

  param[0] = adr;
  param[1] = 1;
  int32_t i = 0;
 read_byte_data_try:
  try {
    WritePacket (id, INST_READ, param, 2);
    ReceivePacket (rbuf, &l1, mask, timeout);
  } catch (TimeOutException &e) {
    i++;
    if(i < DYNAMIXEL_RECEIVEPACKET_TRYCYCLE) {
      goto read_byte_data_try;
    } else {
      throw e;
    }
  }
  *result = rbuf[5];	
}


void Dynamixel::ReadWordData (uint8_t id, uint8_t adr, uint16_t *result, int32_t mask, int32_t timeout)
{
  int32_t             l1;
  uint8_t           param[10];
  uint8_t           rbuf[10];

  if (id == BROADCASTING_ID) {
    throw new WrongArgException();
  }

  ssr::MutexBinder m(m_Mutex);
  param[0] = adr;
  param[1] = 2;
  int32_t i = 0;
 read_word_data_try:
  try {
    WritePacket (id, INST_READ, param, 2);
    ReceivePacket (rbuf, &l1, mask, timeout);
  } catch (TimeOutException &e) {
    i++;
    if(i < DYNAMIXEL_RECEIVEPACKET_TRYCYCLE) {
      goto read_word_data_try;
    } else { 
      throw e;
    }
  }

#if __BYTE_ORDER == __LITTLE_ENDIAN
  *result = rbuf[5] + (rbuf[6] << 8);
#elif __BYTE_ORDER == __BIG_ENDIAN
  *result = rbuf[6] + (rbuf[5] << 8);
#endif
}



void Dynamixel::WriteWordData (uint8_t id, uint8_t adr, uint16_t dat, int32_t mask, int32_t timeout)
{
  int32_t             l1;
  uint8_t           param[10];
  uint8_t           rbuf[10];

  param[0] = adr;
#if __BYTE_ORDER == __LITTLE_ENDIAN
  param[1] = dat & 0xff;
  param[2] = dat >> 8;
#elif __BYTE_ORDER == __BIG_ENDIAN
  param[2] = dat & 0xff;
  param[1] = dat >> 8;
#endif

  ssr::MutexBinder m(m_Mutex);

  int32_t i = 0;
 write_word_data_try:
  try {
    WritePacket (id, INST_WRITE, param, 3);
    if(id != BROADCASTING_ID)
      ReceivePacket (rbuf, &l1, mask, timeout);
  } catch (TimeOutException &e) {
    i++;
    if(i < DYNAMIXEL_RECEIVEPACKET_TRYCYCLE) {
      goto write_word_data_try;
    } else {
      throw e;
    }
  }
}


void Dynamixel::LockItem (int32_t mask) {
  WriteByteData(BROADCASTING_ID, ADDRESS_LOCK, 1, mask, DEFAULT_RESPONSE_TIME);
}

void Dynamixel::SetCWAngleLimit(uint8_t id, uint16_t position, int32_t mask, int32_t timeout) {
  WriteWordData (id, ADDRESS_CW_ANGLE_LIMIT, position, mask, timeout);
}

void Dynamixel::SetCCWAngleLimit(uint8_t id, uint16_t position, int32_t mask, int32_t timeout) {
  WriteWordData (id, ADDRESS_CCW_ANGLE_LIMIT, position, mask, timeout);
}

uint16_t Dynamixel::GetCWAngleLimit(uint8_t id, int32_t mask, int32_t timeout) {
  uint16_t  result;

  ReadWordData (id, ADDRESS_CW_ANGLE_LIMIT, &result, mask, timeout);
  return result;
}

uint16_t Dynamixel::GetCCWAngleLimit(uint8_t id, int32_t mask, int32_t timeout) {
  uint16_t result;

  ReadWordData (id, ADDRESS_CCW_ANGLE_LIMIT, &result, mask, timeout);
  return result;
}


void Dynamixel::MovePosition (uint8_t id, uint16_t position, int32_t mask, int32_t timeout) {
  WriteWordData (id, ADDRESS_GOAL_POSITION, position, mask, timeout);
}

void Dynamixel::SetCompliant (uint8_t id, bool on, int32_t mask, int32_t timeout) {
  WriteByteData (id, ADDRESS_TORQUE_ENABLE, !on, mask, timeout);
}

uint16_t Dynamixel::GetModelNumber (uint8_t id, int32_t mask,
					   int32_t timeout/* = DEFAULT_RESPONSE_TIME*/) 
{
  uint16_t result;
  ReadWordData(id, ADDRESS_MODEL_NUMBER, &result, mask, timeout);
  return result;
}



uint16_t Dynamixel::GetCurrentPosition (uint8_t id, int32_t mask, int32_t timeout) {
  uint16_t  result;
  ReadWordData(id, ADDRESS_PRESENT_POSITION, &result, mask, timeout);
  return result;
}

uint16_t Dynamixel::GetTargetPosition (uint8_t id, int32_t mask, int32_t timeout) {
  uint16_t  result;

  ReadWordData(id, ADDRESS_GOAL_POSITION, &result, mask, timeout);
  return result;
}


void Dynamixel::SendSyncPosition(SyncPosData *pos, int32_t num) {
  uint8_t   param[150];

  param[0] = ADDRESS_GOAL_POSITION;
  param[1] = 2;
  for (int32_t i = 0; i < num; i++) {
    if (pos->ID < BROADCASTING_ID) {
      param[i * 3 + 2] = pos->ID;
#if __BYTE_ORDER == __LITTLE_ENDIAN
      param[i * 3 + 3] = pos->GoalPosition & 0xff;
      param[i * 3 + 4] = (pos->GoalPosition >> 8) & 0xff;
#elif __BYTE_ORDER == __BIG_ENDIAN
      param[i * 3 + 4] = pos->GoalPosition & 0xff;
      param[i * 3 + 3] = (pos->GoalPosition >> 8) & 0xff;
#endif
      pos++;
    } else {
      throw WrongArgException();
    }
  }
  WritePacket (BROADCASTING_ID, INST_SYNC_WRITE, param, 2 + 3 * num);
}

//void Dynamixel::MoveVelocity(uint8_t id, uint16_t velocity) {
//    if(velocity == 0){
//      WriteWordData (id, ADDRESS_GOAL_SPEED, 1);
//      WriteWordData (id, ADDRESS_GOAL_POSITION, GetCurrentPosition(id));
//    }else if(velocity < 0){
//      WriteWordData (id, ADDRESS_GOAL_SPEED, -velocity);
//      WriteWordData (id, ADDRESS_GOAL_POSITION, 1);
//    }else if(velocity > 0){
//      WriteWordData (id, ADDRESS_GOAL_SPEED, velocity);            
//      WriteWordData (id, ADDRESS_GOAL_POSITION, 1023);
//    }
//}

/*----------------------------------------------------------------------------
  uint16_t GetTargetVelocity (uint8_t id, int32_t timeout)
  ----------------------------------------------------------------------------
  DESCRIPTION:
  目標速度取得
  ----------------------------------------------------------------------------*/
uint16_t Dynamixel::GetTargetVelocity (uint8_t id, int32_t mask, int32_t timeout) {
  uint16_t  result;

  ReadWordData(id, ADDRESS_GOAL_SPEED, &result, mask, timeout);
  return result;
}

/*----------------------------------------------------------------------------
  void SetTargetVelocity (uint8_t id, uint16_t velocity)
  ----------------------------------------------------------------------------
  DESCRIPTION:
  目標速度設定
  ----------------------------------------------------------------------------*/
void Dynamixel::SetTargetVelocity (uint8_t id, uint16_t velocity, int32_t mask,  int32_t timeout) {
  WriteWordData (id, ADDRESS_GOAL_SPEED, velocity, mask,  timeout);
}

/*----------------------------------------------------------------------------
  void SetTorqueLimit (uint8_t id, uint16_t velocity)
  ----------------------------------------------------------------------------
  DESCRIPTION:
  トルク制限設定
  ----------------------------------------------------------------------------*/
void Dynamixel::SetTorqueLimit (uint8_t id,uint16_t torque, int32_t mask, int32_t timeout){
  WriteWordData (id, ADDRESS_TORQUE_LIMIT, torque, mask, timeout);
}


short Dynamixel::GetCurrentTorque(uint8_t id, int32_t mask, int32_t timeout) 
{
  uint16_t result;
  ReadWordData(id, ADDRESS_PRESENT_LOAD, &result, mask, timeout);
  if(result >= 1024) {
    return - (result -1024);
  } else {
    return result;
  }
}


short Dynamixel::GetCurrentVelocity(uint8_t id, int32_t mask, int32_t timeout) {
  short speed;
  uint16_t  result;

  ReadWordData(id, ADDRESS_PRESENT_SPEED, &result, mask, timeout);
  if(result > 1024) {
    speed = -(result-1024);
  } else {
    speed = result;
  }
  return speed;
}

uint16_t Dynamixel::GetCurrentTemperature(uint8_t id, int32_t mask, int32_t timeout)
{
  uint16_t result;
  ReadWordData(id, ADDRESS_PRESENT_TEMP, &result, mask, timeout);
  return result;
}

void Dynamixel::SetLED(uint8_t id, int32_t flag, int32_t mask, int32_t timeout) {
  if(flag) {
    WriteByteData(id, ADDRESS_LED, 1, mask, timeout);
  } else {
    WriteByteData(id, ADDRESS_LED, 0, mask, timeout);
  }

}

void Dynamixel::SetID(uint8_t id, uint8_t newID, int32_t mask, int32_t timeout)
{
  WriteByteData(id, ADDRESS_ID, newID, mask, timeout);
}

void Dynamixel::SetBaudRate(uint8_t id, uint8_t baudrate, int32_t mask, int32_t timeout)
{
  WriteByteData(id, ADDRESS_BAUDRATE, baudrate, mask, timeout);
}

void Dynamixel::SetHighestLimitTemperature(uint8_t id, uint8_t temperature, int32_t mask, int32_t timeout)
{
  WriteByteData(id, ADDRESS_HIGHEST_LIMIT_TEMPERATURE, temperature, mask,  timeout);
}


void Dynamixel::SetLowestLimitVoltage(uint8_t id, uint8_t voltage, int32_t mask, int32_t timeout)
{
  WriteByteData(id, ADDRESS_LOWEST_LIMIT_VOLTAGE, voltage, mask, timeout);
}

void Dynamixel::SetHighestLimitVoltage(uint8_t id, uint8_t voltage, int32_t mask, int32_t timeout)
{
  WriteByteData(id, ADDRESS_HIGHEST_LIMIT_VOLTAGE, voltage, mask, timeout);
}

void Dynamixel::SetAlarmShutdownFlag(uint8_t id, uint8_t flag, int32_t mask, int32_t timeout)
{
  WriteByteData(id, ADDRESS_ALARM_SHUTDOWN, flag, mask, timeout);
}

uint8_t Dynamixel::GetAlarmShutdownFlag(uint8_t id, int32_t mask, int32_t timeout)
{
  uint8_t result;
  ReadByteData(id, ADDRESS_ALARM_SHUTDOWN, &result, mask, timeout);
  return result;
}

void Dynamixel::SetAlarmLEDFlag(uint8_t id, uint8_t flag, int32_t mask, int32_t timeout)
{
  WriteByteData(id, ADDRESS_ALARM_LED, flag, mask, timeout);
}

uint8_t Dynamixel::GetAlarmLEDFlag(uint8_t id, int32_t mask, int32_t timeout)
{
  uint8_t result;
  ReadByteData(id, ADDRESS_ALARM_LED, &result, mask,  timeout);
  return result;
}

void Dynamixel::SetComplianceSlope(uint8_t id, uint8_t slope,  int32_t mask, int32_t timeout)
{
  WriteByteData(id, ADDRESS_CW_COMP_SLOPE, slope, mask, timeout);
  WriteByteData(id, ADDRESS_CCW_COMP_SLOPE, slope, mask,  timeout);
}

void Dynamixel::SetComplianceMargin(uint8_t id, uint8_t margin,  int32_t mask, int32_t timeout)
{
  WriteByteData(id, ADDRESS_CW_COMP_MARGIN, margin, mask, timeout);
  WriteByteData(id, ADDRESS_CCW_COMP_MARGIN, margin, mask, timeout);
}

void Dynamixel::SetPunch(uint8_t id, uint16_t punch,  int32_t mask, int32_t timeout)
{
  WriteWordData(id, ADDRESS_PUNCH, punch, mask, timeout);
}

void Dynamixel::TorqueEnable(uint8_t id, int32_t mask, int32_t timeout)
{
  WriteByteData(id, ADDRESS_TORQUE_ENABLE, 1, mask, timeout);
}
