/***************************************************
 * Dynamixel.cpp
 *
 * created 2010/04/20
 * author Yuki Suga (Revast Co., Ltd.)
 * email  ysuga@revast.co.jp
 * copyright Revast Co., Ltd. 2010 all rights reserved.
 ***************************************************/

#include <iostream>
#include "DynamixelV2.h"

using namespace ssr;
using namespace ssr::dynamixel;
//#define DEBUG


#ifndef DEBUG
#define debug( fmt, ... ) ((void)0)
#define dput(fmt, ...) ((void)0)
#else 

#include <stdio.h>
#define debug( fmt, ... ) \
  fprintf( stderr, "%s:%u # " fmt "\n", \
	   __FILE__, __LINE__, ##__VA_ARGS__ )

#define dputd(fmt, v) \
  fprintf(stderr, fmt , v)
#endif



static uint16_t update_crc(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size);

DynamixelV2::DynamixelV2(const char* comport, const long baudrate) : m_SerialPort(comport, baudrate)
{
}

DynamixelV2::~DynamixelV2(void)
{
}

void DynamixelV2::WritePacket (uint8_t cID, TInstruction cInst,  uint8_t *pParam, int32_t iLength) {
  int32_t sum; 
  uint8_t   buf[200]; 
  unsigned long   wlen; 
  uint8_t pattern[3] = {0, 0, 0};
  uint16_t length = 0;
  if ((cID > PACKET_MAX_ID) || (iLength > PACKET_MAX_DATASIZE)) {
    throw WrongArgException();
  }

  buf[0] = PACKET_HEADER_FIXED_VALUE0; // fixed
  buf[1] = PACKET_HEADER_FIXED_VALUE1; // fixed     
  buf[2] = PACKET_HEADER_FIXED_VALUE2; // fixed     
  buf[3] = PACKET_HEADER_RESERVED;
  buf[4] = cID; // ID
  buf[5] = 0;
  buf[6] = 0;
  buf[7] = cInst; // Instruction code.


  pattern[2] = cInst;

  for (int32_t i = 0; i < iLength; i++) { // setting parameter.
    buf[8 + length] = pParam[i];
    length++;

    // Header Pattern Escape Check
    pattern[0] = pattern[1];
    pattern[1] = pattern[2];
    pattern[2] = pParam[i];
    if (pattern[0] == PACKET_HEADER_FIXED_VALUE0 && pattern[1] == PACKET_HEADER_FIXED_VALUE1 && pattern[2] == PACKET_HEADER_FIXED_VALUE2) {
      buf[8 + length] = PACKET_ESCAPE_CHARACTER;
      length++;
    }
  }

  length += 1; // for inst
  length += 2; // FOR CRC

#if __BYTE_ORDER == __LITTLE_ENDIAN
  buf[5] = (uint8_t)(length & 0x00FF);
  buf[6] = (uint8_t)((length >> 8) & 0x00FF);
#elif __BYTE_ORDER == __BIG_ENDIAN
  buf[6] = (uint8_t)(length & 0x00FF);
  buf[5] = (uint8_t)((length & 0xFF00) >> 8);
#endif

  uint16_t crc = update_crc(0, buf, length+5);

#if __BYTE_ORDER == __LITTLE_ENDIAN
  buf[7+length-2] = (uint8_t)(crc & 0x00FF);
  buf[7+length-2+1] = (uint8_t)((crc >> 8) & 0x00FF);
#elif __BYTE_ORDER == __BIG_ENDIAN
  buf[7+length-2+1] = (uint8_t)(crc & 0x00FF);
  buf[7+length-2] = (uint8_t)((crc >> 8) & 0x00FF);
#endif

  
#ifdef DEBUG
  std::cout << "WritePacket(";
  for(int i = 0;i < 7+length;i++) {
    std::cout << (int)buf[i] << " ";
  }
  std::cout << ")" << std::endl;
#endif

  m_SerialPort.FlushRxBuffer();
  wlen = m_SerialPort.Write(buf, 7+length);

  if(wlen != length+7) {
    throw WritePacketException();
  }
}



void DynamixelV2::ReceivePacket (uint8_t *pRcv, int32_t *pLength, int32_t mask, int32_t timeout /* [ms] */)
{
  int32_t ind;
  uint8_t sum;
  uint8_t dataLength;
  TimeSpec time;
  *pLength = 0;

  // Wait for receive header 4 bytes.
  m_Timer.tick();

#ifdef DEBUG
  std::cout << "ReceivePacket:";
#endif
  while (1) {
    if (m_SerialPort.GetSizeInRxBuffer() >= 1) {

      if(m_SerialPort.Read(pRcv, 1) != 1) throw ReceivePacketException();
      if (pRcv[0] == PACKET_HEADER_FIXED_VALUE0) break;
    }
    m_Timer.tack(&time);
    if (time.getUsec() > timeout * 1000) throw TimeOutException();
  }

  while (1) {
    if (m_SerialPort.GetSizeInRxBuffer() >= 1) {
      if(m_SerialPort.Read(pRcv+1, 1) != 1) throw ReceivePacketException();
      if (pRcv[1] == PACKET_HEADER_FIXED_VALUE1) break;
    }
    m_Timer.tack(&time);
    if (time.getUsec() > timeout * 1000) throw TimeOutException();
  }

  while (1) {
    if (m_SerialPort.GetSizeInRxBuffer() >= 1) {
      if(m_SerialPort.Read(pRcv+2, 1) != 1) throw ReceivePacketException();
      if (pRcv[2] == PACKET_HEADER_FIXED_VALUE2) break;
    }
    m_Timer.tack(&time);
    if (time.getUsec() > timeout * 1000) throw TimeOutException();
  }

  while (1) {
    if (m_SerialPort.GetSizeInRxBuffer() >= 1) {
      if(m_SerialPort.Read(pRcv+3, 1) != 1) throw ReceivePacketException();
      break;
    }
    m_Timer.tack(&time);
    if (time.getUsec() > timeout * 1000) throw TimeOutException();
  }
#ifdef DEBUG
  std::cout << "Header Detected: " << std::endl;
#endif

  *pLength = PACKET_HEADER_SIZE;

  uint8_t id = 0;
  while (1) {
    if (m_SerialPort.GetSizeInRxBuffer() >= 1) {
      if(m_SerialPort.Read(pRcv+4, 1) != 1) throw ReceivePacketException();
      id = pRcv[4];
      break;
    }
    
    m_Timer.tack(&time);
    if (time.getUsec() > timeout*1000) throw TimeOutException();
  }

  uint16_t length = 0;
  while (1) {
    if (m_SerialPort.GetSizeInRxBuffer() >= 2) {
      if(m_SerialPort.Read(pRcv+5, 2) != 2) throw ReceivePacketException();
#if __BYTE_ORDER == __LITTLE_ENDIAN
      length = (((uint16_t)pRcv[6])<< 8) | pRcv[5];

#elif __BYTE_ORDER == __BIG_ENDIAN
      length = pRcv[5];
      length = (length << 8) | pRcv[6];
#endif
      break;
    }


    m_Timer.tack(&time);
    if (time.getUsec() > timeout*1000) throw TimeOutException();
  }
#ifdef DEBUG
  std::cout << "id=" << (int)id << ", length=" << (int)length << ", ";
#endif

  while (1) {
    if (m_SerialPort.GetSizeInRxBuffer() >= length) {
      if(m_SerialPort.Read(pRcv+7, length) != length) throw ReceivePacketException();
      break;
    }
    
    m_Timer.tack(&time);
    if (time.getUsec() > timeout*1000) throw TimeOutException();
  }

#ifdef DEBUG
  for(int i = 0;i < length;i++) {
     std::cout << (int)(pRcv[7 + i]) << " ";
  }
  std::cout << ")" << std::endl;;
#endif

  uint16_t crc_low = pRcv[5+length];
  uint16_t crc_high = pRcv[5+length+1];
#if __BYTE_ORDER == __LITTLE_ENDIAN
  uint16_t crc = (crc_high << 8) | crc_low;
#elif __BYTE_ORDER == __BIG_ENDIAN
  uint16_t crc = (crc_low << 8) | crc_high;
#endif

  uint8_t error = pRcv[8];

  switch(error & STATUS_ERROR_NUMBER_MASK) {
  case STATUS_ERROR_NUMBER_RESULT_FAIL:
    throw ResultFailException();
  case STATUS_ERROR_NUMBER_INSTRUCTION_ERROR:
    throw InstructionException();
  case STATUS_ERROR_NUMBER_CRC_ERROR:
#ifdef DEBUG
    std::cout << "crc error returned" << std::endl;
#endif
    throw CRCException();
  case STATUS_ERROR_NUMBER_DATA_RANGE_ERROR:
    throw RangeException();
  case STATUS_ERROR_NUMBER_DATA_LENGTH_ERROR:
    throw LengthException();
  case STATUS_ERROR_NUMBER_DATA_LIMIT_ERROR:
    throw LimitException();
  case STATUS_ERROR_NUMBER_ACCESS_ERROR:
    throw AccessException();
  case 0:
    break;
  default:
    throw UnknownException();
  }


  if(error & STATUS_ERROR_ALERT_FLAG) {
    uint8_t buffer;
    //ReadByteData(id, ADDRESS_HARDWARE_ERROR_STATUS, buffer, mask, timeout);
    ReadData<uint8_t>(id, ADDRESS_HARDWARE_ERROR_STATUS, &buffer, timeout);
    //TODO: Do something
    if (buffer & HARDWARE_INPUTVOLTAGE_ERROR_FLAG) throw InputVoltageException();
    else if (buffer & HARDWARE_OVERHEAT_ERROR_FLAG) throw OverheatingException();
    else if (buffer & HARDWARE_ENCODER_ERROR_FLAG) throw EncoderException();
    else if (buffer & HARDWARE_ELECTRICSHOCK_ERROR_FLAG) throw ElectricshockException();
    else if (buffer & HARDWARE_OVERLOAD_ERROR_FLAG) throw OverloadException();

  } 


  if(update_crc(0, pRcv, length+5) != crc) {
    throw ReceivedCRCException();
  }
  
}


void DynamixelV2::WriteByteData (uint8_t id, uint16_t adr, uint8_t dat, int32_t mask, int32_t timeout) {
  int32_t l1;
  uint8_t param[10];
  uint8_t rbuf[10];

  param[0] = adr & 0x00FF;
  param[1] = ((adr & 0xFF00) >> 8);
  param[2] = dat;

  ssr::MutexBinder m(m_Mutex);

  int32_t i = 0;
 write_byte_data_try:
  try {
    WritePacket (id, INST_WRITE, param, 3);
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


void DynamixelV2::ReadByteData (uint8_t id, uint8_t adr, uint8_t *result, int32_t mask, int32_t timeout) {
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


void DynamixelV2::ReadWordData (uint8_t id, uint8_t adr, uint16_t *result, int32_t mask, int32_t timeout)
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




void DynamixelV2::WriteWordData (uint8_t id, uint8_t adr, uint16_t dat, int32_t mask, int32_t timeout)
{
  int32_t             l1;
  uint8_t           param[10];
  uint8_t           rbuf[10];

  param[0] = adr & 0x00FF;
  param[1] = (adr >> 8) & 0x00FF;
#if __BYTE_ORDER == __LITTLE_ENDIAN
  param[2] = dat & 0xff;
  param[3] = dat >> 8;
#elif __BYTE_ORDER == __BIG_ENDIAN
  param[3] = dat & 0xff;
  param[2] = dat >> 8;
#endif

  ssr::MutexBinder m(m_Mutex);

  int32_t i = 0;
 write_word_data_try:
  try {
    WritePacket (id, INST_WRITE, param, 4);
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


/*
void DynamixelV2::LockItem (int32_t mask) {
  WriteByteData(BROADCASTING_ID, ADDRESS_LOCK, 1, mask, DEFAULT_RESPONSE_TIME);
}
*/

/*
void DynamixelV2::SetCWAngleLimit(uint8_t id, uint16_t position, int32_t mask, int32_t timeout) {
  WriteWordData (id, ADDRESS_CW_ANGLE_LIMIT, position, mask, timeout);
}

void DynamixelV2::SetCCWAngleLimit(uint8_t id, uint16_t position, int32_t mask, int32_t timeout) {
  WriteWordData (id, ADDRESS_CCW_ANGLE_LIMIT, position, mask, timeout);
}
*/

/*
uint16_t DynamixelV2::GetCWAngleLimit(uint8_t id, int32_t mask, int32_t timeout) {
  uint16_t  result;

  ReadWordData (id, ADDRESS_CW_ANGLE_LIMIT, &result, mask, timeout);
  return result;
}

uint16_t DynamixelV2::GetCCWAngleLimit(uint8_t id, int32_t mask, int32_t timeout) {
  uint16_t result;

  ReadWordData (id, ADDRESS_CCW_ANGLE_LIMIT, &result, mask, timeout);
  return result;
}
*/

/*
void DynamixelV2::MovePosition (uint8_t id, uint16_t position, int32_t mask, int32_t timeout) {
  WriteWordData (id, ADDRESS_GOAL_POSITION, position, mask, timeout);
}

void DynamixelV2::SetCompliant (uint8_t id, bool on, int32_t mask, int32_t timeout) {
  WriteByteData (id, ADDRESS_TORQUE_ENABLE, !on, mask, timeout);
}

uint16_t DynamixelV2::GetModelNumber (uint8_t id, int32_t mask,
					   int32_t timeout) 
{
  uint16_t result;
  ReadWordData(id, ADDRESS_MODEL_NUMBER, &result, mask, timeout);
  return result;
}

uint16_t DynamixelV2::GetTargetPosition (uint8_t id, int32_t mask, int32_t timeout) {
  uint16_t  result;

  ReadWordData(id, ADDRESS_GOAL_POSITION, &result, mask, timeout);
  return result;
}


void DynamixelV2::SendSyncPosition(SyncPosData *pos, int32_t num) {
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

//void DynamixelV2::MoveVelocity(uint8_t id, uint16_t velocity) {
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

uint16_t DynamixelV2::GetTargetVelocity (uint8_t id, int32_t mask, int32_t timeout) {
  uint16_t  result;

  ReadWordData(id, ADDRESS_GOAL_SPEED, &result, mask, timeout);
  return result;
}

void DynamixelV2::SetTargetVelocity (uint8_t id, uint16_t velocity, int32_t mask,  int32_t timeout) {
  WriteWordData (id, ADDRESS_GOAL_SPEED, velocity, mask,  timeout);
}


void DynamixelV2::SetTorqueLimit (uint8_t id,uint16_t torque, int32_t mask, int32_t timeout){
  WriteWordData (id, ADDRESS_TORQUE_LIMIT, torque, mask, timeout);
}


short DynamixelV2::GetCurrentTorque(uint8_t id, int32_t mask, int32_t timeout) 
{
  uint16_t result;
  ReadWordData(id, ADDRESS_PRESENT_LOAD, &result, mask, timeout);
  if(result >= 1024) {
    return - (result -1024);
  } else {
    return result;
  }
}


short DynamixelV2::GetCurrentVelocity(uint8_t id, int32_t mask, int32_t timeout) {
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

uint16_t DynamixelV2::GetCurrentTemperature(uint8_t id, int32_t mask, int32_t timeout)
{
  uint16_t result;
  ReadWordData(id, ADDRESS_PRESENT_TEMP, &result, mask, timeout);
  return result;
}

void DynamixelV2::SetLED(uint8_t id, int32_t flag, int32_t mask, int32_t timeout) {
  if(flag) {
    WriteByteData(id, ADDRESS_LED, 1, mask, timeout);
  } else {
    WriteByteData(id, ADDRESS_LED, 0, mask, timeout);
  }
}

void DynamixelV2::SetID(uint8_t id, uint8_t newID, int32_t mask, int32_t timeout)
{
  WriteByteData(id, ADDRESS_ID, newID, mask, timeout);
}

void DynamixelV2::SetBaudRate(uint8_t id, uint8_t baudrate, int32_t mask, int32_t timeout)
{
  WriteByteData(id, ADDRESS_BAUDRATE, baudrate, mask, timeout);
}

void DynamixelV2::SetHighestLimitTemperature(uint8_t id, uint8_t temperature, int32_t mask, int32_t timeout)
{
  WriteByteData(id, ADDRESS_HIGHEST_LIMIT_TEMPERATURE, temperature, mask,  timeout);
}


void DynamixelV2::SetLowestLimitVoltage(uint8_t id, uint8_t voltage, int32_t mask, int32_t timeout)
{
  WriteByteData(id, ADDRESS_LOWEST_LIMIT_VOLTAGE, voltage, mask, timeout);
}

void DynamixelV2::SetHighestLimitVoltage(uint8_t id, uint8_t voltage, int32_t mask, int32_t timeout)
{
  WriteByteData(id, ADDRESS_HIGHEST_LIMIT_VOLTAGE, voltage, mask, timeout);
}

void DynamixelV2::SetAlarmShutdownFlag(uint8_t id, uint8_t flag, int32_t mask, int32_t timeout)
{
  WriteByteData(id, ADDRESS_ALARM_SHUTDOWN, flag, mask, timeout);
}

uint8_t DynamixelV2::GetAlarmShutdownFlag(uint8_t id, int32_t mask, int32_t timeout)
{
  uint8_t result;
  ReadByteData(id, ADDRESS_ALARM_SHUTDOWN, &result, mask, timeout);
  return result;
}

void DynamixelV2::SetAlarmLEDFlag(uint8_t id, uint8_t flag, int32_t mask, int32_t timeout)
{
  WriteByteData(id, ADDRESS_ALARM_LED, flag, mask, timeout);
}

uint8_t DynamixelV2::GetAlarmLEDFlag(uint8_t id, int32_t mask, int32_t timeout)
{
  uint8_t result;
  ReadByteData(id, ADDRESS_ALARM_LED, &result, mask,  timeout);
  return result;
}

void DynamixelV2::SetComplianceSlope(uint8_t id, uint8_t slope,  int32_t mask, int32_t timeout)
{
  WriteByteData(id, ADDRESS_CW_COMP_SLOPE, slope, mask, timeout);
  WriteByteData(id, ADDRESS_CCW_COMP_SLOPE, slope, mask,  timeout);
}

void DynamixelV2::SetComplianceMargin(uint8_t id, uint8_t margin,  int32_t mask, int32_t timeout)
{
  WriteByteData(id, ADDRESS_CW_COMP_MARGIN, margin, mask, timeout);
  WriteByteData(id, ADDRESS_CCW_COMP_MARGIN, margin, mask, timeout);
}

void DynamixelV2::SetPunch(uint8_t id, uint16_t punch,  int32_t mask, int32_t timeout)
{
  WriteWordData(id, ADDRESS_PUNCH, punch, mask, timeout);
}

*/

void DynamixelV2::SetTargetCurrent(uint8_t id, short digit, int32_t timeout) {
  int16_t result;  
  WriteData<int16_t>(id, ADDRESS_GOAL_CURRENT, digit, timeout);
}

short DynamixelV2::GetCurrentCurrent(uint8_t id, int32_t mask, int32_t timeout) 
{
  int16_t result;
  ReadData<int16_t>(id, ADDRESS_PRESENT_CURRENT, &result, timeout);
  return result;
}


static uint16_t update_crc(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size)
{
  uint16_t i, j;
  uint16_t crc_table[256] = {
    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
    0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
    0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
    0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
    0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
    0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
    0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
    0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
    0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
    0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
    0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
    0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
    0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
    0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
    0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
    0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
    0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
    0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
    0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
    0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
    0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
    0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
    0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
    0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
    0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
    0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
    0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
  };
 
  for(j = 0; j < data_blk_size; j++) {
    i = ((uint16_t)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
    crc_accum = (crc_accum << 8) ^ crc_table[i];
  }
 
  return crc_accum;
}
