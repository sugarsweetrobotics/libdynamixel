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
// 一般的な方法です。この DLL 内のすべてのファイルは、コマンド ラインで定義された LIBARM_EXPORTS
// シンボルでコンパイルされます。このシンボルは、この DLL を使うプロジェクトで定義することはできません。
// ソースファイルがこのファイルを含んでいる他のプロジェクトは、 
// LIBDXL_API 関数を DLL からインポートされたと見なすのに対し、この DLL は、このマクロで定義された
// シンボルをエクスポートされたと見なします。
#ifdef dynamixel_EXPORTS
#define LIBDXL_API __declspec(dllexport)
#else
#define LIBDXL_API __declspec(dllimport)
#endif

#else // ifdef WIN32
#define LIBDXL_API 

#endif // ifdef WIN32

#include "Exception.h"

#include "SerialPort.h"
#include "Thread.h"

namespace ssr {
  namespace dynamixel {

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
    class LIBDXL_API Dynamixel {
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
	INST_RESET=6,
	INST_SYNC_WRITE=0x83,
	INST_SYNG_REG_WRITE=0x85
      } ;
      
      static const int32_t PACKET_HEADER_SIZE = 4;
      static const uint8_t PACKET_HEADER_FIXED_VALUE0 = 0xFFU;
      static const uint8_t PACKET_HEADER_FIXED_VALUE1 = 0xFFU;
      static const uint8_t PACKET_MAX_ID = 254U;
      static const uint8_t PACKET_MAX_DATASIZE = 120U;
      static const int32_t DYNAMIXEL_RECEIVEPACKET_TRYCYCLE = 10;
      
    private:
      
      
    public:
      /**
       * @if jp
       * @brief 入力電圧範囲外エラーフラグ
       * @endif
       * @see #SetAlarmShutdownFlag
       * @see #SetAlarmLEDFlag
       * @see #GetAlarmShutdownFlag
       * @see #GetAlarmLEDFlag
       */
      static const uint8_t INPUT_VOLTAGE_ERROR_FLAG = 0x01;
      
      /**
       * @if jp
       * @brief 角度命令範囲外エラーフラグ
       * @endif
       * @see #SetAlarmShutdownFlag
       * @see #SetAlarmLEDFlag
       * @see #GetAlarmShutdownFlag
       * @see #GetAlarmLEDFlag
       */
      static const uint8_t ANGLE_LIMIT_ERROR_FLAG = 0x02;
      
      /**
       * @if jp
       * @brief モータ温度範囲外エラーフラグ
       * @endif
       * @see #SetAlarmShutdownFlag
       * @see #SetAlarmLEDFlag
       * @see #GetAlarmShutdownFlag
       * @see #GetAlarmLEDFlag
       */
      static const uint8_t OVERHEATING_ERROR_FLAG = 0x04;
      
      /**
       * @if jp
       * @brief 設定値範囲外エラーフラグ
       * @endif
       * @see #SetAlarmShutdownFlag
       * @see #SetAlarmLEDFlag
       * @see #GetAlarmShutdownFlag
       * @see #GetAlarmLEDFlag
       */
      static const uint8_t RANGE_ERROR_FLAG = 0x08;
      
      /**
       * @if jp
       * @brief 通信パケットチェックサムエラーフラグ
       * @endif
       * @see #SetAlarmShutdownFlag
       * @see #SetAlarmLEDFlag
       * @see #GetAlarmShutdownFlag
       * @see #GetAlarmLEDFlag
       */
      static const uint8_t CHECKSUM_ERROR_FLAG = 0x10;
      
      /**
       * @if jp
       * @brief 過負荷エラーフラグ
       * @endif
       * @see #SetAlarmShutdownFlag
       * @see #SetAlarmLEDFlag
       * @see #GetAlarmShutdownFlag
       * @see #GetAlarmLEDFlag
       */
      static const uint8_t OVERLOAD_ERROR_FLAG = 0x20;
      
      /**
       * @if jp
       * @brief 不明な命令で起こるエラーフラグ
       * @endif
       * @see #SetAlarmShutdownFlag
       * @see #SetAlarmLEDFlag
       * @see #GetAlarmShutdownFlag
       * @see #GetAlarmLEDFlag
       */
      static const uint8_t INSTRUCTION_ERROR_FLAG = 0x40;
      
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
      Dynamixel(const char* com_port, const long baudrate=54713);
	
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
      virtual ~Dynamixel(void);
		
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
			  uint8_t adr, 
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
      void MovePosition (uint8_t id, uint16_t position, int32_t mask=0x7F, int32_t timeout=DEFAULT_RESPONSE_TIME);


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
      uint16_t GetCurrentPosition (uint8_t id,
					 int32_t mask=0x7F, int32_t timeout = DEFAULT_RESPONSE_TIME) ;

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
      uint16_t GetCurrentTemperature(uint8_t id, int32_t mask=0x7F, int32_t timeout = DEFAULT_RESPONSE_TIME);

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
      void SetLED(uint8_t id, int32_t flag = true,int32_t mask=0x7F,  int32_t timeout=DEFAULT_RESPONSE_TIME);


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
      void ClearLED(uint8_t id, int32_t mask=0x7F, int32_t timeout=DEFAULT_RESPONSE_TIME) {
	SetLED(id, false, mask, timeout);
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


      void TorqueEnable(uint8_t id, int32_t mask=0x7F, int32_t timeout=DEFAULT_RESPONSE_TIME);

    };

  }


#define ADDRESS_MODEL_NUMBER      0   //W
#define ADDRESS_VERSION_FW        2   //B
#define ADDRESS_ID                3   //B
#define ADDRESS_BAUDRATE          4   //B
#define ADDRESS_RETURN_DELAY_TIME 5   //B
#define ADDRESS_CW_ANGLE_LIMIT    6   //W
#define ADDRESS_CCW_ANGLE_LIMIT   8   //W
#define ADDRESS_HIGHEST_LIMIT_TEMPERATURE 11 //B
#define ADDRESS_LOWEST_LIMIT_VOLTAGE 12  //B
#define ADDRESS_HIGHEST_LIMIT_VOLTAGE 13 //B
#define ADDRESS_STATUS_RET_LEVEL  16  //B
#define ADDRESS_ALARM_LED	      17
#define ADDRESS_ALARM_SHUTDOWN    18

#define ADDRESS_TORQUE_ENABLE     24  //B
#define ADDRESS_LED               25  //B

#define ADDRESS_CW_COMP_MARGIN    26  //B
#define ADDRESS_CCW_COMP_MARGIN   27  //B
#define ADDRESS_CW_COMP_SLOPE     28  //B
#define ADDRESS_CCW_COMP_SLOPE    29  //B

#define ADDRESS_GOAL_POSITION     30  //W
#define ADDRESS_GOAL_SPEED        32  //W
#define ADDRESS_TORQUE_LIMIT      34  //W
#define ADDRESS_PRESENT_POSITION  36  //W
#define ADDRESS_PRESENT_SPEED     38  //W
#define ADDRESS_PRESENT_LOAD      40  //W
#define ADDRESS_PRESENT_VOLTAGE   42  //B
#define ADDRESS_PRESENT_TEMP      43  //B
#define ADDRESS_MOVING            46  //B
#define ADDRESS_LOCK              47  //B
#define ADDRESS_PUNCH             48  //W

}
