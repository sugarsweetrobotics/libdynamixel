/***************************************************
 * Exception.h
 *
 * created 2010/04/20
 * author Yuki Suga (SUGAR SWEET ROBOTICS)
 * email  ysuga@ssr.tokyo
 * copyright SUGAR SWEET ROBOTICS CO., LTD. 2018
 ***************************************************/

#pragma once
#include <exception>

namespace ssr {
  namespace dynamixel {

    class DynamixelException : std::exception {
    public:
      DynamixelException(void) {}
      virtual ~DynamixelException(void) throw() {}
    public:
      const char* what() const throw() {
	return "Dynamixel Execption";
      }
    };
    
    /**
     * @if jp
     * @brief モータ通信例外
     *
     * モータとの通信パケットのチェックサムが不正な場合にスローされます。
     * @else
     * @brief
     *
     * @endif
     * This exception is thrown when Dynamixel motor receives wrong data.
     *
     * @author Yuki Suga (SUGAR SWEET ROBOTICS CO., LTD.)
     */
    class ChecksumException : public DynamixelException {
    public:
      ChecksumException(void) {}
      virtual ~ChecksumException() throw() {}
    public:
      const char* what() const throw() {
	return "ChecksumError";
      }
    };


    /**
     * @if jp
     * @brief 過負荷例外
     *
     * 過負荷のときに投げられる例外。
     * @else
     * @brief
     *
     * @endif
     * This exception is thrown when Dynamixel motor is overload.
     *
     * @author Yuki Suga (SUGAR SWEET ROBOTICS CO., LTD.)
     */
    class OverloadException : public DynamixelException {
    public:
      OverloadException(void) {}
      virtual ~OverloadException(void) throw() {}
    public:
      const char* what() const throw() {
	return "Overload";
      }
    };

    
    /**
     * @if jp
     * @brief パケット受信エラー
     *
     * パケットの受信失敗時にスローされる。
     * @else
     * @brief
     *
     * This exception is thrown when Receive packet is failed.
     * @endif
     *
     * @author Yuki Suga (SUGAR SWEET ROBOTICS CO., LTD.)
     */
    class ReceivePacketException : public DynamixelException {
    public:
      ReceivePacketException(void) {}
      virtual ~ReceivePacketException(void) throw() {}
    public:
      const char* what() const throw() {
	return "ReceivePacketError";
      }
    };


    /**
     * @if jp
     * @brief パケット受信タイムアウト例外
     *
     * コマンドを送信した後に、レスポンスが帰ってこない場合にスローされる。
     * この例外の場合、ノイズなどによる通信エラーの場合もあるが、ケーブルの断線、またはDynamixelモータの破損などでもスローされる可能性がある。
     * @else
     * @brief
     *
     * This exception is thrown when Dynamixel motor do not reply.<BR />
     * This exception may imply that corresponding Dynamixel motor is not connected or broken.
     * @endif
     *
     * @author Yuki Suga (SUGAR SWEET ROBOTICS CO., LTD.)
     */
    class TimeOutException : public DynamixelException {
    public:
      TimeOutException(void) {}
      virtual ~TimeOutException(void) throw() {}
    public:
      const char* what() const throw() { 
	return "TimeOut";
      }
    };


    /**
     * @if jp
     * @brief モータのモデルの非対応例外
     *
     * モータのモデル番号が対応モータの番号（107 (EX-106+), 64 (RX-64), 28 (RX-28)）ではない。
     * @else
     * @brief
     *
     * @endif
     * This exception is thrown when Dynamixel motor receives wrong data.
     *
     * @author Yuki Suga (SUGAR SWEET ROBOTICS CO., LTD.)
     */
    class UnknownModelException : public DynamixelException {
    public:
      UnknownModelException(void) {}
      virtual ~UnknownModelException(void) throw() {}
    public:
      const char* what() const throw() {
	return "Unknown Model";
      }
    };


    /**
     * @if jp
     * @brief 不正引数例外
     *
     * Dynamixelクラスのメソッドに不正な引数を与えた場合にスローされる。
     * @else
     * @brief
     *
     * @endif
     * This exception is thrown when Write/Read Packet is called with wrong arguments.
     *
     * @author Yuki Suga (SUGAR SWEET ROBOTICS CO., LTD.)
     */
    class WrongArgException : public DynamixelException {
    public:
      WrongArgException(void) {}
      virtual ~WrongArgException(void) throw() {}
    public:
      const char* what() const throw() {
	return "Wrong Argument";
      }
    };


    /**
     * @if jp
     * @brief 最大関節角度に関する例外
     *
     * 関節角度指定値が最大角度を超えている場合スローされます。
     *
     * @else
     * @brief
     *
     * This exception is thrown when Dynamixel motor receives target position data which is out of range.
     * @endif
     *
     * @author Yuki Suga (SUGAR SWEET ROBOTICS CO., LTD.)
     */
    class AngleLimitException : public DynamixelException {
    public:
      AngleLimitException(void) {}
      virtual ~AngleLimitException(void) throw() {}
    public:
      const char* what() const throw() {
	return "Angle Limit";
      }
    };


    /**
     * @if jp
     * @brief モータインストラクション例外
     * 
     * モータに送信する命令パケットが、不明なインストラクションコードを持っている場合にスローされます。
     * @else
     * @brief
     *
     * @endif
     * This exception is thrown when Dynamixel motor receives invalid command.
     *
     * @author Yuki Suga (SUGAR SWEET ROBOTICS CO., LTD.)
     */
    class InstructionException : public DynamixelException {
    public:
      InstructionException(void) {}
      virtual ~InstructionException(void) throw() {}
    public:
      const char* what() const throw() {
	return "Instruction Error";
      }
    };

    /**
     * @if jp
     * @brief オーバーヒート例外
     *
     * モータの温度が最大温度を超えた場合にスローされます。
     * この例外がスローされた場合は、使用を一時中断し、電源供給をストップしてしばらく待つなどして、
     * モータの温度が十分に下がるまで待ってください。
     * @else
     * @brief
     *
     * This exception is thrown when Dynamixel motor is overheating.
     * @endif
     *
     * @author Yuki Suga (SUGAR SWEET ROBOTICS CO., LTD.)
     */
    class OverheatingException : public DynamixelException {
    public:
      OverheatingException(void) {}
      virtual ~OverheatingException(void) throw() {}
    public:
      const char* what() const throw() {
	return "Overheat";
      }
    };


    /**
     * @if jp
     * @brief 設定値範囲外例外
     *
     * 設定値が設定可能値範囲外の場合に投げられる。
     * @else
     * @brief
     *
     * This exception is thrown when Dynamixel motor receives target position data which is out of range.
     * @endif
     *
     * @author Yuki Suga (SUGAR SWEET ROBOTICS CO., LTD.)
     */
    class RangeException : public DynamixelException {
    public:
      RangeException(void) {}
      virtual ~RangeException(void) throw() {}
    public:
      const char* what() const throw() {
	return "Parameter Out of Range";
      }
    };
    
    /**
     * @if jp
     * @brief パケット送信例外
     *
     * パケットの送信に失敗した場合にスローされる。
     * @else
     * @brief
     *
     * @endif
     * This exception is thrown when Write packet is failed.
     *
     * @author Yuki Suga (SUGAR SWEET ROBOTICS CO., LTD.)
     */
    class WritePacketException : public DynamixelException {
    public:
      WritePacketException(void) {}
      virtual ~WritePacketException(void) throw() {}
    public:
      const char* what() const throw() {
	return "Write Packet Error";
      }
    };
    

    /**
     * @if jp
     * @brief 入力電圧例外
     *
     * モータに供給される電圧が最大電圧を超える場合にスローされます。
     * @else
     * @brief
     *
     * @endif
     * This exception is thrown when Invalid voltage is supplied to Dynamixel motor.
     *
     * @author Yuki Suga (SUGAR SWEET ROBOTICS CO., LTD.)
     */
    class InputVoltageException : public DynamixelException {
    public:
      InputVoltageException(void) {}
      virtual ~InputVoltageException(void) throw() {}
    public:
      const char* what() const throw() {
	return "Input Voltage Error";
      }
    };



    class CRCException : public DynamixelException {
    public:
      CRCException(void) {}
      virtual ~CRCException() throw() {}
    public:
      const char* what() const throw() {
	return "CRC Error";
      }
    };

    class ResultFailException : public DynamixelException {
    public:
      ResultFailException(void) {}
      virtual ~ResultFailException() throw() {}
    public:
      const char* what() const throw() {
	return "Result Failed";
      }
    };

    class LengthException : public DynamixelException {
    public:
      LengthException(void) {}
      virtual ~LengthException() throw() {}
    public:
      const char* what() const throw() {
	return "Package Length Error";
      }
    };

    class LimitException : public DynamixelException {
    public:
      LimitException(void) {}
      virtual ~LimitException(void) throw() {}
    public:
      const char* what() const throw() {
	return "Parameter Limit Error";
      }
    };

    class AccessException : public DynamixelException {
    public:
      AccessException(void) {}
      virtual ~AccessException(void) throw() {}
    public:
      const char* what() const throw() {
	return "Access Error";
      }
    };

    class UnknownException : public DynamixelException {
    public:
      UnknownException(void) {}
      virtual ~UnknownException(void) throw() {}
    public:
      const char* what() const throw() {
	return "Unknown Error";
      }
    };


    
  }
};

