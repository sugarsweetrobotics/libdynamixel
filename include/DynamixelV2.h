/***************************************************
 * Dynamixel.h
 *
 * created 2010/04/20
 * author Yuki Suga (SUGAR SWEET ROBOTICS)
 * email  ysuga@ysuga.net
 * copyright SUGAR SWEET ROBOTICS
 ***************************************************/

#pragma once

#ifdef WIN32
// 以下の ifdef ブロックは DLL からのエクスポートを容易にするマクロを作成するための 
// 一般的な方法です。この DLL 内のすべてのファイルは、コマンド ラインで定義された LIBDYNAMIXEL_EXPORTS
// シンボルでコンパイルされます。このシンボルは、この DLL を使うプロジェクトで定義することはできません。
// ソースファイルがこのファイルを含んでいる他のプロジェクトは、 
// LIBDXL_API 関数を DLL からインポートされたと見なすのに対し、この DLL は、このマクロで定義された
// シンボルをエクスポートされたと見なします。
#ifdef dynamixel_EXPORTS
#define LIBDXL_API __declspec(dllexport)
#else
#ifdef EXPORT_dynamixel_STATIC
#define LIBDXL_API 
#else
#define LIBDXL_API __declspec(dllimport)
#endif 

#endif

#else // ifdef WIN32
#define LIBDXL_API 

#endif // ifdef WIN32

#define _USE_MATH_DEFINES
#include <math.h>

#include "Exception.h"

#include "SerialPort.h"
#include "Thread.h"

namespace ssr {
  namespace dynamixel {

    template<typename T> T unmarshall(const uint8_t* buf) {
      T result = 0;
	for(uint32_t j = 0;j < sizeof(T);j++) {
	  result = result << 8;
#if __BYTE_ORDER == __LITTLE_ENDIAN
	  result |= buf[sizeof(T)-1-j];
#elif __BYTE_ORDER == __BIG_ENDIAN
	  result |= buf[j];
#endif
	}
	return result;
    }

    

  static const uint16_t ADDRESS_MODEL_NUMBER = 0;   //W
  static const uint16_t ADDRESS_MODEL_INFO   = 2;
  static const uint16_t ADDRESS_VERSION_FW   = 6;   //B
  static const uint16_t ADDRESS_ID           = 7;   //B
  static const uint16_t ADDRESS_BAUDRATE     = 8;   //B
  static const uint16_t ADDRESS_RETURN_DELAY_TIME = 9;   //B
  static const uint16_t ADDRESS_DRIVE_MODE = 10;   //B
  static const uint16_t ADDRESS_OPERATING_MODE = 11;   //B
  static const uint16_t ADDRESS_SECONDARY_ID = 12;   //B
  static const uint16_t ADDRESS_PROTOCOL_VERSION = 13;   //B
  static const uint16_t ADDRESS_HOMING_OFFSET = 20;   //B
  static const uint16_t ADDRESS_MOVING_THRESHOLD = 24;   //B
  static const uint16_t ADDRESS_TEMPERATURE_LIMIT = 31;  //B  
  static const uint16_t ADDRESS_MAX_VOLTAGE_LIMIT = 32;  //B
  static const uint16_t ADDRESS_MIN_VOLTAGE_LIMIT = 34;  //B
  static const uint16_t ADDRESS_PWM_LIMIT = 36;
  static const uint16_t ADDRESS_CURRENT_LIMIT = 38;
  static const uint16_t ADDRESS_ACCELERATION_LIMIT = 40;
  static const uint16_t ADDRESS_VELOCITY_LIMIT = 44;
  static const uint16_t ADDRESS_MAX_POSITION_LIMIT = 48;
  static const uint16_t ADDRESS_MIN_POSITION_LIMIT = 52;

  static const uint16_t ADDRESS_EXTERNAL_PORT_MODE1 = 56;
  static const uint16_t ADDRESS_EXTERNAL_PORT_MODE2 = 57;
  static const uint16_t ADDRESS_EXTERNAL_PORT_MODE3 = 58;

  static const uint16_t ADDRESS_SHUTDOWN = 63;

  static const uint16_t ADDRESS_TORQUE_ENABLE   =  64;  //B
  static const uint16_t ADDRESS_LED             =  65;  //B

  static const uint16_t ADDRESS_STATUS_RETURN_LEVEL  = 68;  //B
  static const uint16_t ADDRESS_REGISTERED_INSTRUCTION  = 69;  //B
  static const uint16_t ADDRESS_HARDWARE_ERROR_STATUS  = 70;  //B


  static const uint16_t ADDRESS_VELOCITY_I_GAIN = 76;
  static const uint16_t ADDRESS_VELOCITY_P_GAIN = 78;
  static const uint16_t ADDRESS_POSITION_D_GAIN = 80;
  static const uint16_t ADDRESS_POSITION_I_GAIN = 82;
  static const uint16_t ADDRESS_POSITION_P_GAIN = 84;
  static const uint16_t ADDRESS_FEEDFORWARD_ACCELERATION_GAIN = 88;
  static const uint16_t ADDRESS_FEEDFORWARD_VELOCITY_GAIN = 90;
  static const uint16_t ADDRESS_BUS_WATCHDOG = 98;
  static const uint16_t ADDRESS_GOAL_PWM = 100;
  static const uint16_t ADDRESS_GOAL_CURRENT = 102;
  static const uint16_t ADDRESS_GOAL_VELOCITY = 104;
  static const uint16_t ADDRESS_PROFILE_ACCELERATION = 108;
  static const uint16_t ADDRESS_PROFILE_VELOCITY = 112;
  static const uint16_t ADDRESS_GOAL_POSITION = 116;
  static const uint16_t ADDRESS_REALTIME_TICK = 120;
  static const uint16_t ADDRESS_MOVING = 122;
  static const uint16_t ADDRESS_MOVING_STATUS = 123;
  static const uint16_t ADDRESS_PRESENT_PWM = 124;
  static const uint16_t ADDRESS_PRESENT_CURRENT = 126;
  static const uint16_t ADDRESS_PRESENT_VELOCITY = 128;
  static const uint16_t ADDRESS_PRESENT_POSITION = 132;
  static const uint16_t ADDRESS_VELOCITY_TRAJECTORY = 136;
  static const uint16_t ADDRESS_POSITION_TRAJECTORY = 140;
  static const uint16_t ADDRESS_PRESENT_INPUT_VOLTAGE = 144;
  static const uint16_t ADDRESS_PRESENT_TEMPERATURE = 146;
  static const uint16_t ADDRESS_EXTERNAL_PORT_DATA1 = 152;
  static const uint16_t ADDRESS_EXTERNAL_PORT_DATA2 = 154;
  static const uint16_t ADDRESS_EXTERNAL_PORT_DATA3 = 156;
  static const uint16_t ADDRESS_INDIRECT_ADDRESS_OFFSET = 168;
  static const uint16_t ADDRESS_INDIRECT_ADDRESS_STROKE = 2;
  static const uint16_t ADDRESS_INDIRECT_DATA_OFFSET = 224;
  static const uint16_t ADDRESS_INDIRECT_DATA_STROKE = 1;
  static const uint16_t ADDRESS_INDIRECT_ADDRESS29_OFFSET = 578;
  static const uint16_t ADDRESS_INDIRECT_DATA29_OFFSET = 634;

  static const uint8_t HARDWARE_OVERLOAD_ERROR_FLAG = 0x20;
  static const uint8_t HARDWARE_ELECTRICSHOCK_ERROR_FLAG = 0x10;
  static const uint8_t HARDWARE_ENCODER_ERROR_FLAG = 0x08;
  static const uint8_t HARDWARE_OVERHEAT_ERROR_FLAG = 0x04;
  static const uint8_t HARDWARE_INPUTVOLTAGE_ERROR_FLAG = 0x01;

  static const uint8_t MOVINGSTATUS_INPOSITION = 0x01;
  static const uint8_t MOVINGSTATUS_PROFILEONGOING = 0x02;
  static const uint8_t MOVINGSTATUS_FOLLOWINGERROR = 0x04;
  static const uint8_t MOVINGSTATUS_PROFILETYPE_MASK = 0x30;
  static const uint8_t MOVINGSTATUS_STEPSPEEDPROFILE = 0x00;
  static const uint8_t MOVINGSTATUS_RECTANGULARPROFILE = 0x10;
  static const uint8_t MOVINGSTATUS_TRIANGULARPROFILE = 0x20;
  static const uint8_t MOVINGSTATUS_TRAPEZOIDALPROFILE = 0x30;
    /** 
     * @if jp
     * @brief Dynamixelモータの同期制御に使う構造体
     *
     * @else
     *
     * @brief Struct used when synchronized motor motion
     *
     * @endif
     * @author Yuki Suga, Hirotaka Hachiya (Revast Co., Ltd.)
     * @see CDynamixel#SendSyncPosition(SyncPosData*, int)
     */
    struct LIBDXL_API SyncPosData{
      /**
       * @if jp
       * @brief DynamixelモータのID番号
       * @else
       * @endif
       */
      uint8_t  ID;
      /**
       * @if jp
       * @brief Dynamixelモータの目標位置
       * @else
       * @endif
       */
      uint16_t GoalPosition;
    };
    
    /** 
     * @if jp
     * @brief Dynamixelモータの制御クラス
     *
     * シリアルポートクラスであるCSerialPortを継承している。
     * ひとつのシリアルバスにつき、Dynamixelクラスをひとつ必要とする。
     *
     * @else
     * @brief Dynamixel RC Servo Motor Controlling Class.
     *
     * @endif
     * @author Yuki Suga (Revast Co., Ltd.)
     * @see CSerialPort
     */
    class LIBDXL_API DynamixelV2 {
    private:

    private:
      ssr::SerialPort m_SerialPort;
      ssr::Mutex m_Mutex;
      ssr::Timer m_Timer;
      
      // Instruction
      enum TInstruction{
	INST_PING=1,
	INST_READ=2,
	INST_WRITE=3,
	INST_REG_WRITE=4,
	INST_ACTION=5,
	INST_FACTORY_RESET=6,
	INST_REBOOT=8,
	INST_SYNC_READ=0x82,
	INST_SYNC_WRITE=0x83,
	INST_BULK_READ=0x92,
	INST_BULK_WRITE=0x93,
      } ;
      
      static const int32_t PACKET_HEADER_SIZE = 4;
      static const uint8_t PACKET_HEADER_FIXED_VALUE0 = 0xFFU;
      static const uint8_t PACKET_HEADER_FIXED_VALUE1 = 0xFFU;
      static const uint8_t PACKET_HEADER_FIXED_VALUE2 = 0xFDU;
      static const uint8_t PACKET_HEADER_RESERVED     = 0x00U;
      static const uint8_t PACKET_ESCAPE_CHARACTER    = 0xFDU;

      static const uint8_t PACKET_MAX_ID = 254U;
      static const uint8_t PACKET_MAX_DATASIZE = 120U;
      static const int32_t DYNAMIXEL_RECEIVEPACKET_TRYCYCLE = 10;
      
    private:
      
      static const uint8_t STATUS_ERROR_ALERT_FLAG = 0x80;
      static const uint8_t STATUS_ERROR_NUMBER_MASK = 0x7F;
      static const uint8_t STATUS_ERROR_NUMBER_RESULT_FAIL = 1;
      static const uint8_t STATUS_ERROR_NUMBER_INSTRUCTION_ERROR = 2;
      static const uint8_t STATUS_ERROR_NUMBER_CRC_ERROR = 3;
      static const uint8_t STATUS_ERROR_NUMBER_DATA_RANGE_ERROR = 4;
      static const uint8_t STATUS_ERROR_NUMBER_DATA_LENGTH_ERROR = 5;
      static const uint8_t STATUS_ERROR_NUMBER_DATA_LIMIT_ERROR = 6;
      static const uint8_t STATUS_ERROR_NUMBER_ACCESS_ERROR = 7;

      
    public:
      
      static const uint8_t OVERLOAD_ERROR_FLAG = 0x20;
      static const uint8_t ELECTRICAL_SHOCK_ERROR_FLAG = 0x10;
      static const uint8_t MOTOR_ENCODER_ERROR_FLAG = 0x08;
      static const uint8_t OVERHEATING_ERROR_FLAG = 0x04;      
      static const uint8_t INPUT_VOLTAGE_ERROR_FLAG = 0x01;

      /** 
       * @if jp
       * @brief デフォルトの待ち時間。これ以上待つ場合はCTimeOutExceptionがスローされる。
       *
       * @else
       * @brief Default Timeout value. [ms]
       * 
       * @endif
       * @see CTimeOutException
       *
       */
      static const int32_t DEFAULT_RESPONSE_TIME = 50;
      
      /** 
       * @if jp
       * @brief ブロードキャスティングID
       *
       * すべてのモータに同時に命令を送る場合のID番号。ブロードキャストパケットを送信した場合は、モータからステータスパケットが返らない。
       *
       * @else
       *
       * @brief Broadcast ID used when all servo receive commands.
       * @endif
       *
       */
      static const uint8_t BROADCASTING_ID = 254U;

    public:
      /** 
       * @if jp
       * @brief コンストラクタ
       *
       * @param com_prot COMポート名
       * @param baudrate ボーレート。通信速度。デフォルトで54713。
       *
       * @else
       *
       * @brief Constructor
       * @param com_port Serial port file name.
       * @param baudrate baudrate.
       * @endif
       *
       * @exception CComOpenException
       * @exception CComStateException
       */
      DynamixelV2(const char* com_port, const long baudrate=54713);
	
      /** 
       * @if jp
       * @brief デストラクタ
       *
       * @else
       *
       * @brief Destructor
       * @endif
       *
       */
      virtual ~DynamixelV2(void);
		
    private:
      /** 
       * @if jp
       * @brief COMポートにパケット送信
       *
       * @else
       *
       * @brief Write data packet to COM port.
       * @param cID ID number of RC servo.
       * @param cInst instruction code.
       * @param pParam data value.
       * @param iLength length of packet.
       * @endif
       *
       * @exception CComAccessException
       * @exception CWritePacketException
       */
      void WritePacket (uint8_t cID, 
			TInstruction cInst, 
			uint8_t *pParam, 
			int32_t iLength);

      /** 
       * @if jp
       * @brief COMポートからパケット受信
       *
       * @else
       *
       * @brief Receive data packet to COM port.
       * @param pRcv pointer to received data buffer.
       * @param pLength length of packet.
       * @param timeout timeout value.
       * @endif
       *
       * @exception CComAccessException
       * @exception CReceivePacketException
       * @exception CTimeOutException
       * @exception CAngleLimitException
       * @exception CChecksumException
       * @exception CInputVoltageException
       * @exception CInstructionException
       * @exception COverheatingException
       * @exception COverloadException
       * @exception CRangeException
       */
      void ReceivePacket (uint8_t *pRcv,
			  int32_t *pLength,
			  int32_t mask,
			  int32_t timeout);
		
      template<typename T> void WriteData(uint8_t id, uint16_t adr, T dat, int32_t timeout) {
	uint8_t param[2 + sizeof(T)];
	uint8_t rbuf[256];
	param[0] = adr & 0x00FF;
	param[1] = (adr >> 8) & 0x00FF;
	
	for(uint32_t i = 0;i < sizeof(T);i++) {
#if __BYTE_ORDER == __LITTLE_ENDIAN
	  param[2 + i] = (dat >> (i*8)) & 0xff;
#elif __BYTE_ORDER == __BIG_ENDIAN
	  param[2 + sizeof(T)-1 + i] = (dat >> (i*8)) & 0xff;
#endif
	}
	
	ssr::MutexBinder m(m_Mutex);
	
	int32_t i = 0;
	int32_t l1;
	uint8_t mask = 0;
      write_word_data_try:
	try {
	  WritePacket (id, INST_WRITE, param, 2 + sizeof(T));
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
      



      template<typename T> void ReadData(uint8_t id, uint16_t adr, T* result, int32_t timeout) {
	uint8_t rbuf[64];
	ReadData(id, adr, rbuf, sizeof(T), timeout);
	/*
	int32_t             l1;
	uint8_t           param[4];
	uint8_t           rbuf[10];
	
	if (id == BROADCASTING_ID) {
	  throw new WrongArgException();
	}
	
	ssr::MutexBinder m(m_Mutex);
	param[0] = adr & 0x00FF;
	param[1] = (adr >> 8) & 0x00ff;
	param[2] = sizeof(T);
	param[3] = (sizeof(T) >> 8) & 0x00ff;
	int32_t i = 0;
	uint8_t mask = 0;
      read_word_data_try:
	try {
	  WritePacket (id, INST_READ, param, 4);
	  ReceivePacket (rbuf, &l1, mask, timeout);
	} catch (TimeOutException &e) {
	  i++;
	  if(i < DYNAMIXEL_RECEIVEPACKET_TRYCYCLE) {
	    goto read_word_data_try;
	  } else { 
	    throw e;
	  }
	}
	*/
	/*
	*result = 0;
	for(uint32_t j = 0;j < sizeof(T);j++) {
#if __BYTE_ORDER == __LITTLE_ENDIAN
	  *result |= rbuf[7+j + sizeof(T)-1];
#elif __BYTE_ORDER == __BIG_ENDIAN
	  *result |= rbuf[7+j];
#endif
}*/
	*result = unmarshall<T>(rbuf+9);
      }

      void ReadData(uint8_t id, uint16_t adr, uint8_t* buf, uint32_t size, int32_t timeout) {
	int32_t             l1;
	uint8_t           param[4];
	uint8_t           rbuf[10];
	
	if (id == BROADCASTING_ID) {
	  throw new WrongArgException();
	}
	
	ssr::MutexBinder m(m_Mutex);
	param[0] = adr & 0x00FF;
	param[1] = (adr >> 8) & 0x00ff;
	param[2] = size & 0x00FF;
	param[3] = (size >> 8) & 0x00FF;
	int32_t i = 0;
	uint8_t mask = 0;
      read_word_data_try:
	try {
	  WritePacket (id, INST_READ, param, 4);
	  ReceivePacket (buf, &l1, mask, timeout);
	} catch (TimeOutException &e) {
	  i++;
	  if(i < DYNAMIXEL_RECEIVEPACKET_TRYCYCLE) {
	    goto read_word_data_try;
	  } else { 
	    throw e;
	  }
	}
      }
      
      
      /** 
       * @if jp
       * @brief
       *
       * @else
       *
       * @brief Write data packet to COM port.
       * @param id ID number of RC servo.
       * @param adr address value.
       * @param dat data value.
       * @param timeout timeout value.
       * @endif
       *
       * @exception revast::system::CComAccessException
       * @exception revast::dynamixel::ReceivePacketException
       * @exception revast::dynamixel::WritePacketException
       * @exception revast::dynamixel::TimeOutException
       * @exception revast::dynamixel::AngleLimitException
       * @exception revast::dynamixel::ChecksumException
       * @exception revast::dynamixel::InputVoltageException
       * @exception revast::dynamixel::InstructionException
       * @exception revast::dynamixel::OverheatingException
       * @exception revast::dynamixel::OverloadException
       * @exception revast::dynamixel::RangeException
       */
      void WriteByteData (uint8_t id,
			  uint16_t adr, 
			  uint8_t dat, 
			  int32_t mask,
			  int32_t timeout);

      /** 
       * @if jp
       * @brief
       *
       * @else
       *
       * @brief Write data packet to COM port.
       * @param id ID number of RC servo.
       * @param adr address value.
       * @param dat data value.
       * @param timeout timeout value.
       * @endif
       *
       * @exception revast::system::CComAccessException
       * @exception revast::dynamixel::ReceivePacketException
       * @exception revast::dynamixel::WritePacketException
       * @exception revast::dynamixel::TimeOutException
       * @exception revast::dynamixel::AngleLimitException
       * @exception revast::dynamixel::ChecksumException
       * @exception revast::dynamixel::InputVoltageException
       * @exception revast::dynamixel::InstructionException
       * @exception revast::dynamixel::OverheatingException
       * @exception revast::dynamixel::OverloadException
       * @exception revast::dynamixel::RangeException
       */
      void ReadByteData (uint8_t id,
			 uint8_t adr, 
			 uint8_t *result,
			 int32_t mask,
			 int32_t timeout);

      /** 
       * @if jp
       * @brief
       *
       * @else
       *
       * @brief Write data packet to COM port.
       * @param id ID number of RC servo.
       * @param adr address value.
       * @param dat data value.
       * @param timeout timeout value.
       * @endif
       *
       * @exception revast::system::CComAccessException
       * @exception revast::dynamixel::ReceivePacketException
       * @exception revast::dynamixel::WritePacketException
       * @exception revast::dynamixel::TimeOutException
       * @exception revast::dynamixel::AngleLimitException
       * @exception revast::dynamixel::ChecksumException
       * @exception revast::dynamixel::InputVoltageException
       * @exception revast::dynamixel::InstructionException
       * @exception revast::dynamixel::OverheatingException
       * @exception revast::dynamixel::OverloadException
       * @exception revast::dynamixel::RangeException
       */
      void WriteWordData (uint8_t id,
			  uint8_t adr,
			  uint16_t dat, 
			  int32_t mask,
			  int32_t timeout);

      /** 
       * @if jp
       * @brief
       *
       * @else
       *
       * @brief Read data packet from COM port.
       * @param id ID number of RC servo.
       * @param adr address value.
       * @param result data value buffer.
       * @param timeout timeout value.
       * @endif
       *
       * @exception revast::system::CComAccessException
       * @exception revast::dynamixel::ReceivePacketException
       * @exception revast::dynamixel::WritePacketException
       * @exception revast::dynamixel::TimeOutException
       * @exception revast::dynamixel::AngleLimitException
       * @exception revast::dynamixel::ChecksumException
       * @exception revast::dynamixel::InputVoltageException
       * @exception revast::dynamixel::InstructionException
       * @exception revast::dynamixel::OverheatingException
       * @exception revast::dynamixel::OverloadException
       * @exception revast::dynamixel::RangeException
       */
      void ReadWordData (uint8_t id, 
			 uint8_t adr, 
			 uint16_t *result,
			 int32_t mask, 
			 int32_t timeout );

    public:

      /** 
       * @if jp
       * @brief サーボをON/OFFします。
       *
       * モータにかかるトルクのON/OFFを制御します。 
       *
       * @param id モータのID番号
       * @param on トルクのON/OFFフラグ。TRUEならばトルクON。FALSEならばトルクOFF。
       * @param mask スローされる例外のマスク。例外フラグの対応ビットが1ならば例外がスローされる。
       * @param timeout タイムアウト時間。単位ms。これ以上の時間を待ってもパケットが戻らない場合はCTimeOutExceptionがスローされる。
       * @else
       *
       * @brief Set Compliant mode 
       *
       * When this function is called, the motor is off. <BR />
       * CAUTION!!!! <BR />
       * When the motor is off, the joint32_t of robot is free.
       * If the robot fall down onto the floor, that might destroy the robot. 
       *
       * @param id RC servo id.
       * @param on flag. if TRUE, servo will be compliant (free). else, servo will be on (rigid).
       * @endif
       *
       * @exception ssr::system::CComAccessException
       * @exception ssr::dynamixel::ReceivePacketException
       * @exception ssr::dynamixel::WritePacketException
       * @exception ssr::dynamixel::TimeOutException
       * @exception ssr::dynamixel::AngleLimitException
       * @exception ssr::dynamixel::ChecksumException
       * @exception ssr::dynamixel::InputVoltageException
       * @exception ssr::dynamixel::InstructionException
       * @exception ssr::dynamixel::OverheatingException
       * @exception ssr::dynamixel::OverloadException
       * @exception ssr::dynamixel::RangeException
       */
      void SetCompliant (uint8_t id, bool on, int32_t mask=0x7F, int32_t timeout=DEFAULT_RESPONSE_TIME);

      /** 
       * @if jp
       * @brief ID番号などの変更をロックします。
       *
       * @param mask スローされる例外のマスク。例外フラグの対応ビットが1ならば例外がスローされる。
       * @else
       *
       * @brief Lock ID number of servo motor. 
       *
       * <BR />
       * After calling this function, the ID of motor cannot be modified.
       * @endif
       *
       * @exception ssr::system::CComAccessException
       * @exception ssr::dynamixel::ReceivePacketException
       * @exception ssr::dynamixel::WritePacketException
       * @exception ssr::dynamixel::TimeOutException
       * @exception ssr::dynamixel::AngleLimitException
       * @exception ssr::dynamixel::ChecksumException
       * @exception ssr::dynamixel::InputVoltageException
       * @exception ssr::dynamixel::InstructionException
       * @exception ssr::dynamixel::OverheatingException
       * @exception ssr::dynamixel::OverloadException
       * @exception ssr::dynamixel::RangeException
       */
      void LockItem (int32_t mask=0x7F);

      /** 
       * @if jp
       * @brief 目標位置を書き込みます。
       * @param id モータのID番号
       * @param position モータの目標位置。[0, 1023]で、サーボホーンの±75度に対応。
       * @param mask スローされる例外のマスク。例外フラグの対応ビットが1ならば例外がスローされる。
       * @param timeout タイムアウト時間。単位ms。これ以上の時間を待ってもパケットが戻らない場合はCTimeOutExceptionがスローされる。
       * @else
       *
       * @brief Send target position to the servo motor.
       * 
       * @param id ID number of RC servo.
       * @param position target position. [0-1023]
       * @endif
       *
       * @exception ssr::system::CComAccessException
       * @exception ssr::dynamixel::ReceivePacketException
       * @exception ssr::dynamixel::WritePacketException
       * @exception ssr::dynamixel::TimeOutException
       * @exception ssr::dynamixel::AngleLimitException
       * @exception ssr::dynamixel::ChecksumException
       * @exception ssr::dynamixel::InputVoltageException
       * @exception ssr::dynamixel::InstructionException
       * @exception ssr::dynamixel::OverheatingException
       * @exception ssr::dynamixel::OverloadException
       * @exception ssr::dynamixel::RangeException
       */
      void MovePosition (uint8_t id, int32_t position, int32_t timeout=DEFAULT_RESPONSE_TIME) {
	WriteData<int32_t>(id, ADDRESS_GOAL_POSITION, position, timeout);
      }


      /** 
       * @if jp
       * @brief 現在のモータのモデル番号を取得。
       *
       * @param id モータのID番号
       * @param timeout タイムアウト時間。単位ms。これ以上の時間を待ってもパケットが戻らない場合はCTimeOutExceptionがスローされる。
       * @return モータのモデル番号。
       *
       * @else
       *
       * @brief Get target position of RC servo motor.
       * 
       * @param id ID number of RC servo.
       * @param mask スローされる例外のマスク。例外フラグの対応ビットが1ならば例外がスローされる。
       * @param timeout timeout value.
       * @return model number
       * @endif
       *
       * @exception ssr::system::CComAccessException
       * @exception ssr::dynamixel::ReceivePacketException
       * @exception ssr::dynamixel::WritePacketException
       * @exception ssr::dynamixel::TimeOutException
       * @exception ssr::dynamixel::AngleLimitException
       * @exception ssr::dynamixel::ChecksumException
       * @exception ssr::dynamixel::InputVoltageException
       * @exception ssr::dynamixel::InstructionException
       * @exception ssr::dynamixel::OverheatingException
       * @exception ssr::dynamixel::OverloadException
       * @exception ssr::dynamixel::RangeException
       */
      uint16_t GetModelNumber (uint8_t id, int32_t mask=0x7F,
				     int32_t timeout = DEFAULT_RESPONSE_TIME) ;



      /** 
       * @if jp
       * @brief 現在のモータの位置を取得。
       *
       * @param id モータのID番号
       * @param timeout タイムアウト時間。単位ms。これ以上の時間を待ってもパケットが戻らない場合はCTimeOutExceptionがスローされる。
       * @return 現在位置。[0, 1023]の値が、±75度に対応。
       *
       * @else
       *
       * @brief Get target position of RC servo motor.
       * 
       * @param id ID number of RC servo.
       * @param mask スローされる例外のマスク。例外フラグの対応ビットが1ならば例外がスローされる。
       * @param timeout timeout value.
       * @return target position [0-1023]
       * @endif
       *
       * @exception ssr::system::CComAccessException
       * @exception ssr::dynamixel::ReceivePacketException
       * @exception ssr::dynamixel::WritePacketException
       * @exception ssr::dynamixel::TimeOutException
       * @exception ssr::dynamixel::AngleLimitException
       * @exception ssr::dynamixel::ChecksumException
       * @exception ssr::dynamixel::InputVoltageException
       * @exception ssr::dynamixel::InstructionException
       * @exception ssr::dynamixel::OverheatingException
       * @exception ssr::dynamixel::OverloadException
       * @exception ssr::dynamixel::RangeException
       */
      int32_t GetCurrentPosition (uint8_t id, int32_t mask=0x7F, int32_t timeout = DEFAULT_RESPONSE_TIME) {
	int32_t result;
	ReadData<int32_t>(id, ADDRESS_PRESENT_POSITION, &result, timeout);
	return result;
      }

      /** 
       * @if jp
       * @brief 現在の目標位置を取得
       *
       * @param id モータのID番号
       * @param timeout タイムアウト時間。単位ms。これ以上の時間を待ってもパケットが戻らない場合はCTimeOutExceptionがスローされる。
       * @return 現在の目標位置。[0, 1023]が±75度に対応
       * 
       * @else
       *
       * @brief Get target position of RC servo motor.
       * 
       * @param id ID number of RC servo.
       * @param mask スローされる例外のマスク。例外フラグの対応ビットが1ならば例外がスローされる。
       * @param timeout timeout value.
       * @return target position [0-1023]
       * @endif
       *
       * @exception ssr::system::CComAccessException
       * @exception ssr::dynamixel::ReceivePacketException
       * @exception ssr::dynamixel::WritePacketException
       * @exception ssr::dynamixel::TimeOutException
       * @exception ssr::dynamixel::AngleLimitException
       * @exception ssr::dynamixel::ChecksumException
       * @exception ssr::dynamixel::InputVoltageException
       * @exception ssr::dynamixel::InstructionException
       * @exception ssr::dynamixel::OverheatingException
       * @exception ssr::dynamixel::OverloadException
       * @exception ssr::dynamixel::RangeException
       */
      uint16_t GetTargetPosition (uint8_t id,
					int32_t mask=0x7F, int32_t timeout = DEFAULT_RESPONSE_TIME);

      /** 
       * @if jp
       * @brief 現在の目標速度を取得
       *
       * @param id モータのID番号
       * @param velocity 現在の目標速度。[0-1023]。
       * @param mask スローされる例外のマスク。例外フラグの対応ビットが1ならば例外がスローされる。
       * @param timeout タイムアウト時間。単位ms。これ以上の時間を待ってもパケットが戻らない場合はCTimeOutExceptionがスローされる。
       * @else
       *
       * @brief Send target velocity to the servo motor.
       *
       * The velocity will be active when the MoveImmediately command is called next.
       * @param id ID number of RC servo.
       * @param velocity target velocity. [0-1023]
       * @endif
       *
       * @see #MoveImmediately(uint8_t, uint16_t)
       * @exception ssr::system::CComAccessException
       * @exception ssr::dynamixel::ReceivePacketException
       * @exception ssr::dynamixel::WritePacketException
       * @exception ssr::dynamixel::TimeOutException
       * @exception ssr::dynamixel::AngleLimitException
       * @exception ssr::dynamixel::ChecksumException
       * @exception ssr::dynamixel::InputVoltageException
       * @exception ssr::dynamixel::InstructionException
       * @exception ssr::dynamixel::OverheatingException
       * @exception ssr::dynamixel::OverloadException
       * @exception ssr::dynamixel::RangeException
       */
      void SetTargetVelocity (uint8_t id,uint16_t velocity, int32_t mask=0x7F, int32_t timeout=DEFAULT_RESPONSE_TIME);

      /** 
       * @if jp
       * @brief 現在の目標速度を取得
       *
       * @param id モータのID番号
       * @param timeout タイムアウト時間。単位ms。これ以上の時間を待ってもパケットが戻らない場合はCTimeOutExceptionがスローされる。
       * @param mask スローされる例外のマスク。例外フラグの対応ビットが1ならば例外がスローされる。
       * @return 現在の目標速度。[0-1023]。
       * @else
       *
       * @brief Get target velocity.
       * @param id ID number of RC servo.
       * @param timeout timeout value.
       * @return target velocity [0-1023]
       * @endif
       *
       * @exception ssr::system::CComAccessException
       * @exception ssr::dynamixel::ReceivePacketException
       * @exception ssr::dynamixel::WritePacketException
       * @exception ssr::dynamixel::TimeOutException
       * @exception ssr::dynamixel::AngleLimitException
       * @exception ssr::dynamixel::ChecksumException
       * @exception ssr::dynamixel::InputVoltageException
       * @exception ssr::dynamixel::InstructionException
       * @exception ssr::dynamixel::OverheatingException
       * @exception ssr::dynamixel::OverloadException
       * @exception ssr::dynamixel::RangeException
       */
      uint16_t GetTargetVelocity (uint8_t id,
					int32_t mask=0x7F, int32_t timeout = DEFAULT_RESPONSE_TIME);

      /** 
       * @if jp
       * @brief 現在速度を取得
       *
       * @param id モータのID番号
       * @param timeout タイムアウト時間。単位ms。これ以上の時間を待ってもパケットが戻らない場合はCTimeOutExceptionがスローされる。
       * @param mask スローされる例外のマスク。例外フラグの対応ビットが1ならば例外がスローされる。
       * @return 現在の回転速度。[0-1023]。
       * @else
       * 
       * @brief Get current velocity. 
       * @param id ID number of RC servo.
       * @param timeout timeout value.
       * @return current velocity [-1023, +1023]
       * @endif
       *
       * @exception ssr::system::CComAccessException
       * @exception ssr::dynamixel::ReceivePacketException
       * @exception ssr::dynamixel::WritePacketException
       * @exception ssr::dynamixel::TimeOutException
       * @exception ssr::dynamixel::AngleLimitException
       * @exception ssr::dynamixel::ChecksumException
       * @exception ssr::dynamixel::InputVoltageException
       * @exception ssr::dynamixel::InstructionException
       * @exception ssr::dynamixel::OverheatingException
       * @exception ssr::dynamixel::OverloadException
       * @exception ssr::dynamixel::RangeException
       */
      short GetCurrentVelocity(uint8_t id, int32_t mask=0x7F, int32_t timeout = DEFAULT_RESPONSE_TIME);


      // obsolute
      //void MoveVelocity (uint8_t id, uint16_t velocity);

      /** 
       * @if jp
       * @brief トルク制限値の設定。
       *
       * トルク制限値を超えるトルクの発生を抑えられる。COverloadExceptionは、モータの機能限界の場合にスローされる例外なので、
       * この設定値とは関係ない。
       * @param id モータのID番号
       * @param torque トルクの制限値。[0, 1023]。
       * @param mask スローされる例外のマスク。例外フラグの対応ビットが1ならば例外がスローされる。
       * @param timeout タイムアウト時間。単位ms。これ以上の時間を待ってもパケットが戻らない場合はCTimeOutExceptionがスローされる。
       * @else
       *
       * @brief Send torque limitation value to the servo motor.
       * 
       * @param id ID number of RC servo.
       * @param torque torque limit. [0, 1023]. 0 = free. 1023 = maximum.
       * @endif
       *
       * @exception ssr::system::CComAccessException
       * @exception ssr::dynamixel::ReceivePacketException
       * @exception ssr::dynamixel::WritePacketException
       * @exception ssr::dynamixel::TimeOutException
       * @exception ssr::dynamixel::AngleLimitException
       * @exception ssr::dynamixel::ChecksumException
       * @exception ssr::dynamixel::InputVoltageException
       * @exception ssr::dynamixel::InstructionException
       * @exception ssr::dynamixel::OverheatingException
       * @exception ssr::dynamixel::OverloadException
       * @exception ssr::dynamixel::RangeException
       */
      void SetTorqueLimit (uint8_t id, uint16_t torque, int32_t mask=0x7F, int32_t timeout=DEFAULT_RESPONSE_TIME);

      /** 
       * @if jp
       * @brief 現在の負荷トルク値を取得
       *
       * @param id モータのID番号
       * @param timeout タイムアウト時間。単位ms。これ以上の時間を待ってもパケットが戻らない場合はCTimeOutExceptionがスローされる。
       * @param mask スローされる例外のマスク。例外フラグの対応ビットが1ならば例外がスローされる。
       * @return 現在のトルク。
       * @else
       *
       * @brief Get Current Torque.
       * @param id ID number of RC servo.
       * @return current torque (-1023, +1023).
       * @endif
       *
       * @exception ssr::system::CComAccessException
       * @exception ssr::dynamixel::ReceivePacketException
       * @exception ssr::dynamixel::WritePacketException
       * @exception ssr::dynamixel::TimeOutException
       * @exception ssr::dynamixel::AngleLimitException
       * @exception ssr::dynamixel::ChecksumException
       * @exception ssr::dynamixel::InputVoltageException
       * @exception ssr::dynamixel::InstructionException
       * @exception ssr::dynamixel::OverheatingException
       * @exception ssr::dynamixel::OverloadException
       * @exception ssr::dynamixel::RangeException
       */
      short GetCurrentTorque(uint8_t id, int32_t mask=0x7F,  int32_t timeout = DEFAULT_RESPONSE_TIME);


      /** 
       * @if jp
       * @brief 現在のモータ温度を取得。
       *
       * @param id モータのID番号
       * @param mask スローされる例外のマスク。例外フラグの対応ビットが1ならば例外がスローされる。
       * @param timeout タイムアウト時間。単位ms。これ以上の時間を待ってもパケットが戻らない場合はCTimeOutExceptionがスローされる。
       * @return 現在の温度。単位は度。
       * @else
       *
       * @brief Get Current Temperatur.
       * @param id ID number of RC servo.
       * @return current temperature (0, 1023).
       * @endif
       *
       * @exception ssr::system::CComAccessException
       * @exception ssr::dynamixel::ReceivePacketException
       * @exception ssr::dynamixel::WritePacketException
       * @exception ssr::dynamixel::TimeOutException
       * @exception ssr::dynamixel::AngleLimitException
       * @exception ssr::dynamixel::ChecksumException
       * @exception ssr::dynamixel::InputVoltageException
       * @exception ssr::dynamixel::InstructionException
       * @exception ssr::dynamixel::OverheatingException
       * @exception ssr::dynamixel::OverloadException
       * @exception ssr::dynamixel::RangeException
       */
      uint16_t GetCurrentTemperature(uint8_t id, int32_t mask=0x7F, int32_t timeout = DEFAULT_RESPONSE_TIME) {
	uint16_t result;
	ReadData<uint16_t>(id, ADDRESS_PRESENT_TEMPERATURE, &result, timeout);
	return result;
      }

      /** 
       * @if jp
       * @brief 現在のモータへの供給電圧を取得
       *
       * @param id モータのID番号
       * @param mask スローされる例外のマスク。例外フラグの対応ビットが1ならば例外がスローされる。0ならばTimeOut以外の例外はスローされない。
       * @param timeout タイムアウト時間。単位ms。これ以上の時間を待ってもパケットが戻らない場合はCTimeOutExceptionがスローされる。
       * @return 現在の供給電圧値。単位は[V]だが、10倍の値が出ます。
       * @else
       *
       * @brief Get Supply Voltage.
       * @param id ID number of RC servo.
       * @return current supply Voltage (0, 26).
       * @endif
       *
       * @exception ssr::system::CComAccessException
       * @exception ssr::dynamixel::ReceivePacketException
       * @exception ssr::dynamixel::WritePacketException
       * @exception ssr::dynamixel::TimeOutException
       * @exception ssr::dynamixel::AngleLimitException
       * @exception ssr::dynamixel::ChecksumException
       * @exception ssr::dynamixel::InputVoltageException
       * @exception ssr::dynamixel::InstructionException
       * @exception ssr::dynamixel::OverheatingException
       * @exception ssr::dynamixel::OverloadException
       * @exception ssr::dynamixel::RangeException
       */
      uint16_t GetSupplyVoltage(uint8_t id, int32_t mask=0x7F, int32_t timeout = DEFAULT_RESPONSE_TIME);

      /** 
       * @if jp
       * @brief LEDの点灯。
       *
       * @param id モータのID番号
       * @param flag LEDの点灯/消灯。デフォルトでTRUE（点灯）。FALSEを与えると消灯。
       * @param mask スローされる例外のマスク。例外フラグの対応ビットが1ならば例外がスローされる。0ならばTimeOut以外の例外はスローされない。
       * @param timeout タイムアウト時間。単位ms。これ以上の時間を待ってもパケットが戻らない場合はCTimeOutExceptionがスローされる。
       * @else
       *
       * @brief Set LED
       * @param id ID number of RC Servo.
       * @param flag if TRUE, LED is turned on. if FALSE, LED is turned off.
       * @endif
       *
       * @see #ClrLED(uint8_t, int)
       * @exception ssr::system::CComAccessException
       * @exception ssr::dynamixel::ReceivePacketException
       * @exception ssr::dynamixel::WritePacketException
       * @exception ssr::dynamixel::TimeOutException
       * @exception ssr::dynamixel::AngleLimitException
       * @exception ssr::dynamixel::ChecksumException
       * @exception ssr::dynamixel::InputVoltageException
       * @exception ssr::dynamixel::InstructionException
       * @exception ssr::dynamixel::OverheatingException
       * @exception ssr::dynamixel::OverloadException
       * @exception ssr::dynamixel::RangeException
       */
      void SetLED(uint8_t id, int32_t flag = true, int32_t timeout=DEFAULT_RESPONSE_TIME) {
	//	WriteByteData(id, ADDRESS_LED, flag ? 1 : 0, 0, timeout);
	WriteData<uint8_t>(id, ADDRESS_LED, flag ? 1: 0, timeout);
      }


      /** 
       * @if jp
       * @brief LEDの消灯
       *
       * @param id モータのID番号
       * @param mask スローされる例外のマスク。例外フラグの対応ビットが1ならば例外がスローされる。0ならばTimeOut以外の例外はスローされない。
       * @param timeout タイムアウト時間。単位ms。これ以上の時間を待ってもパケットが戻らない場合はCTimeOutExceptionがスローされる。
       * @else
       *
       * @brief Clear LED
       * @param id ID number of RC Servo.
       * @endif
       *
       * @see #SetLED(uint8_t, int, int)
       * @exception ssr::system::CComAccessException
       * @exception ssr::dynamixel::ReceivePacketException
       * @exception ssr::dynamixel::WritePacketException
       * @exception ssr::dynamixel::TimeOutException
       * @exception ssr::dynamixel::AngleLimitException
       * @exception ssr::dynamixel::ChecksumException
       * @exception ssr::dynamixel::InputVoltageException
       * @exception ssr::dynamixel::InstructionException
       * @exception ssr::dynamixel::OverheatingException
       * @exception ssr::dynamixel::OverloadException
       * @exception ssr::dynamixel::RangeException
       */
      void ClearLED(uint8_t id, int32_t timeout=DEFAULT_RESPONSE_TIME) {
	SetLED(id, false, timeout);
      }


      /** 
       * @if jp
       * @brief モータの同期制御命令。
       *
       * 通常は下記のように使います。 <BR />
       * ex:<BR />
       * > SyncPosData sp[2];<BR />
       * > sp[0].ID = 0;<BR />
       * > sp[0].GoalPosition = 100;<BR />
       * > sp[1].ID = 1;<BR />
       * > sp[1].GoalPosition = 100;<BR />
       * > SendSyncPosition(sp, 2); // move 2 motors synchronizingly.<BR />
       * 
       * @param pos 目標データとID番号の入ったSyncPosData構造体。
       * @param num モータの個数。
       * @else
       *
       * @brief Send synchronizing motion packet.
       * ex:<BR />
       * > SyncPosData sp[2];<BR />
       * > sp[0].ID = 0;<BR />
       * > sp[0].GoalPosition = 100;<BR />
       * > sp[1].ID = 1;<BR />
       * > sp[1].GoalPosition = 100;<BR />
       * > SendSyncPosition(sp, 2); // move 2 motors synchronizingly.<BR />
       *
       * @param pos target data and id pair.
       * @param num the number of RC servo motor.
       * @endif
       * @see SyncPosData
       *
       * @exception ssr::system::CComAccessException
       * @exception ssr::dynamixel::ReceivePacketException
       * @exception ssr::dynamixel::WritePacketException
       * @exception ssr::dynamixel::TimeOutException
       * @exception ssr::dynamixel::AngleLimitException
       * @exception ssr::dynamixel::ChecksumException
       * @exception ssr::dynamixel::InputVoltageException
       * @exception ssr::dynamixel::InstructionException
       * @exception ssr::dynamixel::OverheatingException
       * @exception ssr::dynamixel::OverloadException
       * @exception ssr::dynamixel::RangeException
       */
      void SendSyncPosition(SyncPosData *pos, int32_t num);


      /** 
       * @if jp
       * @brief CW回転方向の制限値設定
       *
       * @param id モータのID番号
       * @param position 制限位置。[0, 1023]。CW < CCWでなくてはならない。
       * @param mask スローされる例外のマスク。例外フラグの対応ビットが1ならば例外がスローされる。0ならばTimeOut以外の例外はスローされない。
       * @param timeout タイムアウト時間。単位ms。これ以上の時間を待ってもパケットが戻らない場合はCTimeOutExceptionがスローされる。
       * @else
       * 
       * @brief Set CW Angle Limit Value
       * @param id ID of Motor
       * @param position Angle Limit (0-1023)
       * @endif
       *
       * @see ssr::dynamixel::AngleLimitException
       * @exception ssr::system::CComAccessException
       * @exception ssr::dynamixel::ReceivePacketException
       * @exception ssr::dynamixel::WritePacketException
       * @exception ssr::dynamixel::TimeOutException
       * @exception ssr::dynamixel::AngleLimitException
       * @exception ssr::dynamixel::ChecksumException
       * @exception ssr::dynamixel::InputVoltageException
       * @exception ssr::dynamixel::InstructionException
       * @exception ssr::dynamixel::OverheatingException
       * @exception ssr::dynamixel::OverloadException
       * @exception ssr::dynamixel::RangeException
       */
      void SetCWAngleLimit(uint8_t id, uint16_t position, int32_t mask=0x7F, int32_t timeout=DEFAULT_RESPONSE_TIME) ;

      /** 
       * @if jp
       * @brief CCW回転方向の制限値設定。
       *
       * @param id モータのID番号
       * @param position 制限位置。[0, 1023]。 CW < CCWでなくてはならない。
       * @param mask スローされる例外のマスク。例外フラグの対応ビットが1ならば例外がスローされる。0ならばTimeOut以外の例外はスローされない。
       * @param timeout タイムアウト時間。単位ms。これ以上の時間を待ってもパケットが戻らない場合はCTimeOutExceptionがスローされる。
       * @else
       *
       * @brief Set CCW Angle Limit Value
       * @param id ID of Motor
       * @param position Angle Limit (0-1023)
       * @endif
       *
       * @see ssr::dynamixel::AngleLimitException
       * @exception ssr::system::CComAccessException
       * @exception ssr::dynamixel::ReceivePacketException
       * @exception ssr::dynamixel::WritePacketException
       * @exception ssr::dynamixel::TimeOutException
       * @exception ssr::dynamixel::AngleLimitException
       * @exception ssr::dynamixel::ChecksumException
       * @exception ssr::dynamixel::InputVoltageException
       * @exception ssr::dynamixel::InstructionException
       * @exception ssr::dynamixel::OverheatingException
       * @exception ssr::dynamixel::OverloadException
       * @exception ssr::dynamixel::RangeException
       */
      void SetCCWAngleLimit(uint8_t id, uint16_t position, int32_t mask=0x7F, int32_t timeout=DEFAULT_RESPONSE_TIME) ;
			
      /** 
       * @if jp
       * @brief モータの制限位置取得
       *
       * @param id モータのID番号
       * @param mask スローされる例外のマスク。例外フラグの対応ビットが1ならば例外がスローされる。0ならばTimeOut以外の例外はスローされない。
       * @param timeout タイムアウト時間。単位ms。これ以上の時間を待ってもパケットが戻らない場合はCTimeOutExceptionがスローされる。
       * @return 現在のモータのCW方向制限位置を取得。[0, 1023]。
       * @else
       *
       * @brief Get CW Angle Limit Value
       * @param id ID of Motor
       * @return Angle Limit Value
       * @endif
       *
       * @exception ssr::system::CComAccessException
       * @exception ssr::dynamixel::ReceivePacketException
       * @exception ssr::dynamixel::WritePacketException
       * @exception ssr::dynamixel::TimeOutException
       * @exception ssr::dynamixel::AngleLimitException
       * @exception ssr::dynamixel::ChecksumException
       * @exception ssr::dynamixel::InputVoltageException
       * @exception ssr::dynamixel::InstructionException
       * @exception ssr::dynamixel::OverheatingException
       * @exception ssr::dynamixel::OverloadException
       * @exception ssr::dynamixel::RangeException
       */
      uint16_t GetCWAngleLimit(uint8_t id, int32_t mask=0x7F, int32_t timeout=DEFAULT_RESPONSE_TIME);

      /** 
       * @if jp
       * @brief モータのCCW方向制限位置を取得
       *
       * @param id モータのID番号
       * @param mask スローされる例外のマスク。例外フラグの対応ビットが1ならば例外がスローされる。0ならばTimeOut以外の例外はスローされない。
       * @param timeout タイムアウト時間。単位ms。これ以上の時間を待ってもパケットが戻らない場合はCTimeOutExceptionがスローされる。
       * @return 現在のモータのCCW方向制限位置を取得。[0, 1023]
       * @else
       *
       * @brief Get CCW Angle Limit Value
       * @param id ID of Motor
       * @return Angle Limit Value
       * @endif
       *
       * @exception ssr::system::CComAccessException
       * @exception ssr::dynamixel::ReceivePacketException
       * @exception ssr::dynamixel::WritePacketException
       * @exception ssr::dynamixel::TimeOutException
       * @exception ssr::dynamixel::AngleLimitException
       * @exception ssr::dynamixel::ChecksumException
       * @exception ssr::dynamixel::InputVoltageException
       * @exception ssr::dynamixel::InstructionException
       * @exception ssr::dynamixel::OverheatingException
       * @exception ssr::dynamixel::OverloadException
       * @exception ssr::dynamixel::RangeException
       */
      uint16_t GetCCWAngleLimit(uint8_t id, int32_t mask=0x7F, int32_t timeout=DEFAULT_RESPONSE_TIME);

      /** 
       * @if jp
       * @brief モータのID番号変更
       *
       * @param id 設定先のモータのID番号
       * @param newID 新しいID番号
       * @param mask スローされる例外のマスク。例外フラグの対応ビットが1ならば例外がスローされる。0ならばTimeOut以外の例外はスローされない。
       * @param timeout タイムアウト時間。単位ms。これ以上の時間を待ってもパケットが戻らない場合はCTimeOutExceptionがスローされる。
       * @else
       *
       * @brief Set ID Number
       * @param id ID of Motor
       * @return Angle Limit Value
       * @endif
       *
       * @exception ssr::system::CComAccessException
       * @exception ssr::dynamixel::ReceivePacketException
       * @exception ssr::dynamixel::WritePacketException
       * @exception ssr::dynamixel::TimeOutException
       * @exception ssr::dynamixel::AngleLimitException
       * @exception ssr::dynamixel::ChecksumException
       * @exception ssr::dynamixel::InputVoltageException
       * @exception ssr::dynamixel::InstructionException
       * @exception ssr::dynamixel::OverheatingException
       * @exception ssr::dynamixel::OverloadException
       * @exception ssr::dynamixel::RangeException
       */
      void SetID(uint8_t id, uint8_t newID, int32_t mask=0x7F, int32_t timeout=DEFAULT_RESPONSE_TIME);


      /** 
       * @if jp
       * @brief モータの温度限界値設定
       *
       * この設定値を超えると COverheatingExceptionがスローされる。
       *
       * @param id モータのID番号
       * @param temperature 温度限界値。単位は度。デフォルトで80度。
       * @param mask スローされる例外のマスク。例外フラグの対応ビットが1ならば例外がスローされる。0ならばTimeOut以外の例外はスローされない。
       * @param timeout タイムアウト時間。単位ms。これ以上の時間を待ってもパケットが戻らない場合はCTimeOutExceptionがスローされる。
       * @else
       *
       * @brief Set ID Number
       * @param id ID of Motor
       * @return Angle Limit Value
       * @endif
       *
       * @see COverheatingException
       * @exception ssr::system::CComAccessException
       * @exception ssr::dynamixel::ReceivePacketException
       * @exception ssr::dynamixel::WritePacketException
       * @exception ssr::dynamixel::TimeOutException
       * @exception ssr::dynamixel::AngleLimitException
       * @exception ssr::dynamixel::ChecksumException
       * @exception ssr::dynamixel::InputVoltageException
       * @exception ssr::dynamixel::InstructionException
       * @exception ssr::dynamixel::OverheatingException
       * @exception ssr::dynamixel::OverloadException
       * @exception ssr::dynamixel::RangeException
       */
      void SetHighestLimitTemperature(uint8_t id, uint8_t temperature, int32_t mask=0x7F, int32_t timeout=DEFAULT_RESPONSE_TIME);




      /** 
       * @if jp
       * @brief モータの通信速度設定
       *
       * @param id モータのID番号
       * @param baudrate 通信速度。通信速度は baudrate[bps] = 2000000 / (value + 1)で計算される。
       * @param mask スローされる例外のマスク。例外フラグの対応ビットが1ならば例外がスローされる。0ならばTimeOut以外の例外はスローされない。
       * @param timeout タイムアウト時間。単位ms。これ以上の時間を待ってもパケットが戻らない場合はCTimeOutExceptionがスローされる。
       * @else
       *
       * @brief Set ID Number
       * @param id ID of Motor
       * @return Angle Limit Value
       * @endif
       *
       * @exception ssr::system::CComAccessException
       * @exception ssr::dynamixel::ReceivePacketException
       * @exception ssr::dynamixel::WritePacketException
       * @exception ssr::dynamixel::TimeOutException
       * @exception ssr::dynamixel::AngleLimitException
       * @exception ssr::dynamixel::ChecksumException
       * @exception ssr::dynamixel::InputVoltageException
       * @exception ssr::dynamixel::InstructionException
       * @exception ssr::dynamixel::OverheatingException
       * @exception ssr::dynamixel::OverloadException
       * @exception ssr::dynamixel::RangeException
       */
      void SetBaudRate(uint8_t id, uint8_t baudrate, int32_t mask=0x7F, int32_t timeout=DEFAULT_RESPONSE_TIME);


      /** 
       * @if jp
       * @brief モータの最低供給電圧設定
       *
       * 最低供給電圧を下回ると CInputVoltageExceptin がスローされる。
       * @param id モータのID番号
       * @param voltage 最低供給電圧。これを下回ると CInputVoltageExceptionがスローされる。
       * @param mask スローされる例外のマスク。例外フラグの対応ビットが1ならば例外がスローされる。0ならばTimeOut以外の例外はスローされない。
       * @param timeout タイムアウト時間。単位ms。これ以上の時間を待ってもパケットが戻らない場合はCTimeOutExceptionがスローされる。
       * @else
       *
       * @brief Set ID Number
       * @param id ID of Motor
       * @return Angle Limit Value
       * @endif
       *
       * @see CInputVoltageException
       * @exception ssr::system::CComAccessException
       * @exception ssr::dynamixel::ReceivePacketException
       * @exception ssr::dynamixel::WritePacketException
       * @exception ssr::dynamixel::TimeOutException
       * @exception ssr::dynamixel::AngleLimitException
       * @exception ssr::dynamixel::ChecksumException
       * @exception ssr::dynamixel::InputVoltageException
       * @exception ssr::dynamixel::InstructionException
       * @exception ssr::dynamixel::OverheatingException
       * @exception ssr::dynamixel::OverloadException
       * @exception ssr::dynamixel::RangeException
       */
      void SetLowestLimitVoltage(uint8_t id, uint8_t voltage, int32_t mask=0x7F, int32_t timeout=DEFAULT_RESPONSE_TIME);

      /** 
       * @if jp
       * @brief モータの最大供給電圧設定
       *
       * 最大供給電圧を超えると CInputVoltageExceptin がスローされる。
       * @param id モータのID番号
       * @param voltage 最大供給電圧。これを超えると CInputVoltageExceptionがスローされる。
       * @param mask スローされる例外のマスク。例外フラグの対応ビットが1ならば例外がスローされる。0ならばTimeOut以外の例外はスローされない。
       * @param timeout タイムアウト時間。単位ms。これ以上の時間を待ってもパケットが戻らない場合はCTimeOutExceptionがスローされる。
       * @else
       *
       * @brief Set ID Number
       * @param id ID of Motor
       * @return Angle Limit Value
       * @endif
       *
       * @see CInputVoltageException
       * @exception ssr::system::CComAccessException
       * @exception ssr::dynamixel::ReceivePacketException
       * @exception ssr::dynamixel::WritePacketException
       * @exception ssr::dynamixel::TimeOutException
       * @exception ssr::dynamixel::AngleLimitException
       * @exception ssr::dynamixel::ChecksumException
       * @exception ssr::dynamixel::InputVoltageException
       * @exception ssr::dynamixel::InstructionException
       * @exception ssr::dynamixel::OverheatingException
       * @exception ssr::dynamixel::OverloadException
       * @exception ssr::dynamixel::RangeException
       */
      void SetHighestLimitVoltage(uint8_t id, uint8_t voltage, int32_t mask=0x7F, int32_t timeout=DEFAULT_RESPONSE_TIME);

      /** 
       * @if jp
       * @brief アラームがおき多々場合の自動トルクオフ設定
       *
       * 対応するフラグがONになっていると、エラーが発生した場合にトルクオフになる。
       * デフォルトで0x24 ( Overload, Range Error )
       *
       * <BR />
       * ex: <BR />
       * >> SetAlarmShutdownFlag(0, OVERLOAD_ERROR_FLAG | RANGE_ERROR_FLAG | OVERHEATING_ERROR_FLAG);<BR />
       * >> // 過負荷、温度範囲外と設定値範囲外の場合に自動的にトルクオフ。
       *
       * @param id モータのID番号
       * @param flag エラーフラグ。
       * @param mask スローされる例外のマスク。例外フラグの対応ビットが1ならば例外がスローされる。0ならばTimeOut以外の例外はスローされない。
       * @param timeout タイムアウト時間。単位ms。これ以上の時間を待ってもパケットが戻らない場合はCTimeOutExceptionがスローされる。
       * @else
       *
       * @brief Set ID Number
       * @param id ID of Motor
       * @return Angle Limit Value
       * @endif
       *
       * @see INSTRUCTION_ERROR_FLAG
       * @see OVERLOAD_ERROR_FLAG
       * @see CHECKSUM_ERROR_FLAG
       * @see RANGE_ERROR_FLAG
       * @see OVERHEATING_ERROR_FLAG
       * @see ANGLE_LIMIT_ERROR_FLAG
       * @see INPUT_VOLTAGE_ERROR_FLAG
       * @exception ssr::system::CComAccessException
       * @exception ssr::dynamixel::ReceivePacketException
       * @exception ssr::dynamixel::WritePacketException
       * @exception ssr::dynamixel::TimeOutException
       * @exception ssr::dynamixel::AngleLimitException
       * @exception ssr::dynamixel::ChecksumException
       * @exception ssr::dynamixel::InputVoltageException
       * @exception ssr::dynamixel::InstructionException
       * @exception ssr::dynamixel::OverheatingException
       * @exception ssr::dynamixel::OverloadException
       * @exception ssr::dynamixel::RangeException
       */
      void SetAlarmShutdownFlag(uint8_t id, uint8_t flag, int32_t mask=0x7F,  int32_t timeout=DEFAULT_RESPONSE_TIME);

      /** 
       * @if jp
       * @brief アラームがおきた場合の自動LED点灯設定
       *
       * 対応するフラグがONになっていると、エラーが発生した場合にLED点灯。
       * デフォルトで0x24 ( Overload, Range Error )
       *
       * <BR />
       * ex: <BR />
       * >> SetAlarmLEDFlag(0, OVERLOAD_ERROR_FLAG | RANGE_ERROR_FLAG | OVERHEATING_ERROR_FLAG);<BR />
       * >> // 過負荷、温度範囲外と設定値範囲外の場合に自動的にLED点灯。
       *
       * @param id モータのID番号
       * @param flag エラーフラグ。
       * @param mask スローされる例外のマスク。例外フラグの対応ビットが1ならば例外がスローされる。0ならばTimeOut以外の例外はスローされない。
       * @param timeout タイムアウト時間。単位ms。これ以上の時間を待ってもパケットが戻らない場合はCTimeOutExceptionがスローされる。
       * @else
       *
       * @brief Set ID Number
       * @param id ID of Motor
       * @endif
       *
       * @see INSTRUCTION_ERROR_FLAG
       * @see OVERLOAD_ERROR_FLAG
       * @see CHECKSUM_ERROR_FLAG
       * @see RANGE_ERROR_FLAG
       * @see OVERHEATING_ERROR_FLAG
       * @see ANGLE_LIMIT_ERROR_FLAG
       * @see INPUT_VOLTAGE_ERROR_FLAG
       * @exception ssr::system::CComAccessException
       * @exception ssr::dynamixel::ReceivePacketException
       * @exception ssr::dynamixel::WritePacketException
       * @exception ssr::dynamixel::TimeOutException
       * @exception ssr::dynamixel::AngleLimitException
       * @exception ssr::dynamixel::ChecksumException
       * @exception ssr::dynamixel::InputVoltageException
       * @exception ssr::dynamixel::InstructionException
       * @exception ssr::dynamixel::OverheatingException
       * @exception ssr::dynamixel::OverloadException
       * @exception ssr::dynamixel::RangeException
       */
      void SetAlarmLEDFlag(uint8_t id, uint8_t flag, int32_t mask=0x7F, int32_t timeout=DEFAULT_RESPONSE_TIME);

      /** 
       * @if jp
       * @brief アラームがおきた場合の自動トルクオフ設定を取得
       *
       * 対応するフラグがONになっていると、エラーが発生した場合にトルクオフになる。
       * デフォルトで0x24 ( Overload, Range Error )
       *
       * <BR />
       * ex: <BR />
       * >> uint8_t buf = GetAlarmShutdownFlag(0);<BR />
       * >> if (buf & OVERLOAD_ERROR_FLAG) { <BR />
       * >> // 過負荷、温度範囲外と設定値範囲外の場合に自動的にトルクオフならば
       *
       * @param id モータのID番号
       * @param mask スローされる例外のマスク。例外フラグの対応ビットが1ならば例外がスローされる。0ならばTimeOut以外の例外はスローされない。
       * @param timeout タイムアウト時間。単位ms。これ以上の時間を待ってもパケットが戻らない場合はCTimeOutExceptionがスローされる。
       * @return エラーフラグ。
       * @else
       *
       * @brief Set ID Number
       * @param id ID of Motor
       * @return Angle Limit Value
       * @endif
       *
       * @see INSTRUCTION_ERROR_FLAG
       * @see OVERLOAD_ERROR_FLAG
       * @see CHECKSUM_ERROR_FLAG
       * @see RANGE_ERROR_FLAG
       * @see OVERHEATING_ERROR_FLAG
       * @see ANGLE_LIMIT_ERROR_FLAG
       * @see INPUT_VOLTAGE_ERROR_FLAG
       * @exception ssr::system::CComAccessException
       * @exception ssr::dynamixel::ReceivePacketException
       * @exception ssr::dynamixel::WritePacketException
       * @exception ssr::dynamixel::TimeOutException
       * @exception ssr::dynamixel::AngleLimitException
       * @exception ssr::dynamixel::ChecksumException
       * @exception ssr::dynamixel::InputVoltageException
       * @exception ssr::dynamixel::InstructionException
       * @exception ssr::dynamixel::OverheatingException
       * @exception ssr::dynamixel::OverloadException
       * @exception ssr::dynamixel::RangeException
       */
      uint8_t GetAlarmShutdownFlag(uint8_t id, int32_t mask=0x7F, int32_t timeout=DEFAULT_RESPONSE_TIME);

      /** 
       * @if jp
       * @brief アラームがおきた場合の自動LED点灯設定を取得
       *
       * 対応するフラグがONになっていると、エラーが発生した場合にLED点灯になる。
       * デフォルトで0x24 ( Overload, Range Error )
       *
       * <BR />
       * ex: <BR />
       * >> uint8_t buf = GetAlarmShutdownFlag(0);<BR />
       * >> if (buf & OVERLOAD_ERROR_FLAG) { <BR />
       * >> // 過負荷、温度範囲外と設定値範囲外の場合に自動的にLED点灯ならば
       *
       * @param id モータのID番号
       * @param mask スローされる例外のマスク。例外フラグの対応ビットが1ならば例外がスローされる。0ならばTimeOut以外の例外はスローされない。
       * @param timeout タイムアウト時間。単位ms。これ以上の時間を待ってもパケットが戻らない場合はCTimeOutExceptionがスローされる。
       * @return エラーフラグ。
       * @else
       *
       * @brief Set ID Number
       * @param id ID of Motor
       * @return Angle Limit Value
       * @endif
       *
       * @see INSTRUCTION_ERROR_FLAG
       * @see OVERLOAD_ERROR_FLAG
       * @see CHECKSUM_ERROR_FLAG
       * @see RANGE_ERROR_FLAG
       * @see OVERHEATING_ERROR_FLAG
       * @see ANGLE_LIMIT_ERROR_FLAG
       * @see INPUT_VOLTAGE_ERROR_FLAG
       * @exception ssr::system::CComAccessException
       * @exception ssr::dynamixel::ReceivePacketException
       * @exception ssr::dynamixel::WritePacketException
       * @exception ssr::dynamixel::TimeOutException
       * @exception ssr::dynamixel::AngleLimitException
       * @exception ssr::dynamixel::ChecksumException
       * @exception ssr::dynamixel::InputVoltageException
       * @exception ssr::dynamixel::InstructionException
       * @exception ssr::dynamixel::OverheatingException
       * @exception ssr::dynamixel::OverloadException
       * @exception ssr::dynamixel::RangeException
       */
      uint8_t  GetAlarmLEDFlag(uint8_t id, int32_t mask=0x7F, int32_t timeout=DEFAULT_RESPONSE_TIME);



      /** 
       * @if jp
       * @brief コンプライアンスの傾きを設定する
       *
       * @param id モータのID番号
       * @param slope スロープ
       * @param mask スローされる例外のマスク。例外フラグの対応ビットが1ならば例外がスローされる。0ならばTimeOut以外の例外はスローされない。
       * @param timeout タイムアウト時間。単位ms。これ以上の時間を待ってもパケットが戻らない場合はCTimeOutExceptionがスローされる。
       * @else
       *
       * @brief Set ID Number
       * @param id ID of Motor
       * @return Angle Limit Value
       * @endif
       *
       * @exception ssr::system::CComAccessException
       * @exception ssr::dynamixel::ReceivePacketException
       * @exception ssr::dynamixel::WritePacketException
       * @exception ssr::dynamixel::TimeOutException
       * @exception ssr::dynamixel::AngleLimitException
       * @exception ssr::dynamixel::ChecksumException
       * @exception ssr::dynamixel::InputVoltageException
       * @exception ssr::dynamixel::InstructionException
       * @exception ssr::dynamixel::OverheatingException
       * @exception ssr::dynamixel::OverloadException
       * @exception ssr::dynamixel::RangeException
       */
      void SetComplianceSlope(uint8_t id, uint8_t slope,  int32_t mask=0x7F, int32_t timeout=DEFAULT_RESPONSE_TIME);



      /** 
       * @if jp
       * @brief コンプライアンスの幅（不感帯）を設定する
       *
       * @param id モータのID番号
       * @param margin 幅。デフォルトは1。
       * @param mask スローされる例外のマスク。例外フラグの対応ビットが1ならば例外がスローされる。0ならばTimeOut以外の例外はスローされない。
       * @param timeout タイムアウト時間。単位ms。これ以上の時間を待ってもパケットが戻らない場合はCTimeOutExceptionがスローされる。
       * @else
       *
       * @brief Set ID Number
       * @param id ID of Motor
       * @return Angle Limit Value
       * @endif
       *
       * @exception ssr::system::CComAccessException
       * @exception ssr::dynamixel::ReceivePacketException
       * @exception ssr::dynamixel::WritePacketException
       * @exception ssr::dynamixel::TimeOutException
       * @exception ssr::dynamixel::AngleLimitException
       * @exception ssr::dynamixel::ChecksumException
       * @exception ssr::dynamixel::InputVoltageException
       * @exception ssr::dynamixel::InstructionException
       * @exception ssr::dynamixel::OverheatingException
       * @exception ssr::dynamixel::OverloadException
       * @exception ssr::dynamixel::RangeException
       */
      void SetComplianceMargin(uint8_t id, uint8_t margin, int32_t mask=0x7F,  int32_t timeout=DEFAULT_RESPONSE_TIME);



      /** 
       * @if jp
       * @brief モータに流す最小の電流
       *
       * @param id モータのID番号
       * @param punch モータに流す電流。[0, 1024]
       * @param mask スローされる例外のマスク。例外フラグの対応ビットが1ならば例外がスローされる。0ならばTimeOut以外の例外はスローされない。
       * @param timeout タイムアウト時間。単位ms。これ以上の時間を待ってもパケットが戻らない場合はCTimeOutExceptionがスローされる。
       * @else
       *
       * @brief Set ID Number
       * @param id ID of Motor
       * @return Angle Limit Value
       * @endif
       *
       * @exception ssr::system::CComAccessException
       * @exception ssr::dynamixel::ReceivePacketException
       * @exception ssr::dynamixel::WritePacketException
       * @exception ssr::dynamixel::TimeOutException
       * @exception ssr::dynamixel::AngleLimitException
       * @exception ssr::dynamixel::ChecksumException
       * @exception ssr::dynamixel::InputVoltageException
       * @exception ssr::dynamixel::InstructionException
       * @exception ssr::dynamixel::OverheatingException
       * @exception ssr::dynamixel::OverloadException
       * @exception ssr::dynamixel::RangeException
       */
      void SetPunch(uint8_t id, uint16_t punch,  int32_t mask=0x7F, int32_t timeout=DEFAULT_RESPONSE_TIME);


      void TorqueEnable(uint8_t id, int32_t timeout=DEFAULT_RESPONSE_TIME) {
	WriteData<uint8_t>(id, ADDRESS_TORQUE_ENABLE, 1, timeout);
      }

      void TorqueDisable(uint8_t id, int32_t timeout=DEFAULT_RESPONSE_TIME) {
	WriteData<uint8_t>(id, ADDRESS_TORQUE_ENABLE, 0, timeout);
      }

      void SetProfileAcceleration(uint8_t id, uint32_t acc, int32_t timeout=DEFAULT_RESPONSE_TIME) {
	WriteData<uint32_t>(id, ADDRESS_PROFILE_ACCELERATION, acc, timeout);
      }

      void SetProfileVelocity(uint8_t id, uint32_t vel, int32_t timeout=DEFAULT_RESPONSE_TIME) {
	WriteData<uint32_t>(id, ADDRESS_PROFILE_VELOCITY, vel, timeout);
      }

      uint8_t GetMovingStatus(uint8_t id, int32_t timeout=DEFAULT_RESPONSE_TIME) {
	uint8_t result;
	ReadData<uint8_t>(id, ADDRESS_MOVING_STATUS, &result, timeout);
	return result;
      }

      uint32_t GetVelocityLimit(uint8_t id, int32_t timeout=DEFAULT_RESPONSE_TIME) {
	uint32_t result;
	ReadData<uint32_t>(id, ADDRESS_VELOCITY_LIMIT, &result, timeout);
	return result;
      }
    };

    inline double pos_to_rad(int32_t pos) {
      return pos * 2 * M_PI / 4096;
    }
    
    inline int32_t rad_to_pos(double rad) {
      return rad * 4096 / 2 / M_PI;
    }

    inline double vel_to_rpm(int32_t vel) {
      return vel * 0.229;
    }

    inline int32_t rpm_to_vel(double& rpm) {
      return rpm / 0.229;
    }

    inline double rpm_to_rad_per_sec(double& rpm) {
      return rpm  * M_PI * 2 / 60;
    }

    inline double rad_per_sec_to_rpm(double& rad) {
      return rad * 60 / 2 / M_PI;
    }

    inline double acc_to_rpm2(int32_t acc) {
      return acc * 60 * 60 / 2 / M_PI;
    }

    inline int32_t rpm2_to_acc(double rpm2) {
      return rpm2 * M_PI * 2 / 60 / 60;
    }


  }
  
}
