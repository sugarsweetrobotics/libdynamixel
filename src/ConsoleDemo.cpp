/*
 * ConsoleDemo.cpp
 *
 * コンソールプログラムの例。
 *
 * author: Hirotaka Hachiya, Yuki Suga (RevatCo., Ltd.)
 *
 * copyright: Revast Co., Ltd. 2010
 * 
 */


#include <signal.h>
#include <string>
#include <iostream>
#include <sstream>

#ifdef WIN32
#include <conio.h>
#else
#include <unistd.h>
#endif


#include <math.h>
#include "Tsurugi.h"

using namespace std;
using namespace revast;
using namespace revast::arm;



/*
 * CConsoleDemoクラス。
 *
 * コンソールデモ用クラス。コンストラクタにほとんどの処理が記述してある。
 *
 * author: Hirotaka Hachiya, Yuki Suga (Revast Co., Ltd.)
 */
class CConsoleDemo {
public:

	/*
	 * Tsurugiクラスのインスタンス。
	 */
	CTsurugi myArm;

	/*
	 * 各関節の角度指令を行う場合の、動作ステップ。初期値5.0。
	 */
	double step;


	/*
	 * 関節の数
	 */
	static const int NUM_JOINT = 6;


	/*
	 * ティーチングで記憶可能なバッファの数。
	 */
	static const int NUM_BUFFER = 1024;

	/*
	 * ティーチングで関節角度を保存するためのバッファ。
	 */
	double PositionBuffer[NUM_JOINT][NUM_BUFFER];

	/*
	 * ティーチングで保存したポジションの数。
	 */
	int CurrentBufferPosition;
public:
	
	/*
	 * コンストラクタ
	 *
	 * メイン関数のほとんどの処理はここに記述してある。
	 *  argc main関数の引数1．コマンドライン引数の数＋１
	 *  argv main関数の引数2．コマンドライン引数をあらわす文字列。0番目はコマンド自体そのもの。
	 */
	CConsoleDemo(int argc, char* argv[]): CurrentBufferPosition(0), step(5.0) {

		string comportName; // 通信ポートのファイル名
		ostringstream oss;  // ファイル名を作成するためのストリーム。 


		cout << "Input COM Port Name.... >> ";
		std::string lineBuffer, dummy;
		std::getline(std::cin, lineBuffer);
		//oss << "Port = " << lineBuffer;
		comportName = lineBuffer;
		
		// 通信ポートを開く
		cout << "COM PORT OPEN [" << comportName << "]" << endl;
		try {
			myArm.Connect(comportName.c_str(), 57600);
			Initialize(); // 初期化。
		} catch (CException &e) {
			cout << "Exception occurred." << endl;
			cout << ">> " << e.what() << endl;
			cout << "Aborted." << endl;
			exit(-1);
		}

		cout << "Input Command ( 'h'=help ) >> " << ends;
		int endflag = FALSE; // 終了フラグ。TRUEでWhileを抜ける。
		char ch;
		while(!endflag) {

			try {
				  std::cout << "Input Command and Enter." << std::endl;
				  std::getline(std::cin, lineBuffer);
				  ch = lineBuffer[0];
					switch(ch){
						case 'h': // ヘルプ表示
							Help();
							break;
						case 0x1B: // "ESC"は終了
							endflag = TRUE;
							break;
						case 'p': // 現在位置表示
							ShowCurrentPosition();
							break;
						case 'v': // 現在速度表示
							ShowCurrentVelocity();
							break;
						case 'V': // 目標速度変更
							ChangeTargetVelocity();
							break;
						case 'F': // トルクリミット変更
							ChangeLimitTorque();
							break;
						case '1': //1: increase motor 1 position
							MoveRelatively(0, step);
							break;
						case '!': //Shift+1: decrese motor 1 position
							MoveRelatively(0, -step);
							break;
						case '2': //2: increase motor 2 position
							MoveRelatively(1, step);
							break;
						case '"': //Shift+2: decrese motor 2 position
							MoveRelatively(1, -step);
							break;
						case '3': //3: increase motor 1 position
							MoveRelatively(2, step);
							break;
						case '#': //Shift+3: decrese motor 1 position
							MoveRelatively(2, -step);
							break;
						case '4': //4: increase motor 1 position
							MoveRelatively(3, step);
							break;
						case '$': //Shift+4: decrese motor 1 position
							MoveRelatively(3, -step);
							break;
						case '5': //5: increase motor 1 position
							MoveRelatively(4, step);
							break;
						case '%': //Shift+5: decrese motor 1 position
							MoveRelatively(4, -step);
							break;
						case '6': //6: increase motor 1 position
							MoveRelatively(5, step);
							break;
						case '&': //Shift+6: decrese motor 1 position
							MoveRelatively(5, -step);
							break;
						case '7': // Close Hand;
							myArm.Hand->Close();
							break;
						case '8': // Open Hand;
							myArm.Hand->Open();
							break;
						case 's': // 加算的な動作のステップ数変更
							ChangeStep();
							break;
						case 't':  //Motor torque on
							cout << "Motor torque ON" << endl;
							myArm.ServoOn();
							break;
						case 'T': // Motor torque off
							cout << "Motor torque OFF" << endl;
							myArm.ServoOff();
							break;
						case 'a': // add current position
							ShowCurrentPosition();
							RecordCurrentPosition();
							break;
						case 'c': // clear position buffer;
							ClearPositionBuffer();
							break;
						case 'r': // 保存された位置情報表示。
							ShowRecordedPosition();
							break;
						case 'g': // プレイバック動作。
							PlayRecordedPosition();
							break;
						case 'o': // 初期位置へ移動
							myArm.Joint[0]->MoveImmediately(0);
							myArm.Joint[1]->MoveImmediately(0);
							myArm.Joint[2]->MoveImmediately(0);
							myArm.Joint[3]->MoveImmediately(0);
							myArm.Joint[4]->MoveImmediately(0);
							myArm.Joint[5]->MoveImmediately(0);
							break;
						default:
							cout << "Unknown Command." << endl;
							Help();
							break;
					} // switch
					cout << "Input Command ( 'h'=help ) >> " << ends;

			} catch (CException &e) {
				cout << "Exception occurred." << endl;
				cout << ">> " << e.what() << endl;
			}
		} // while
	}
		

	/*
	 * デストラクタ
	 */
	virtual ~CConsoleDemo() {}


public:

	/*
	 * 初期化処理
	 *
	 * myArmのトルクをオンにする。
	 */
	void Initialize() {
		myArm.ServoOn();
	}

	/*
	 * プレイバック動作。
	 *
	 * 記録したポジションの最初から最後までアームを動かす。
	 * 各関節が5度以内の誤差に入るまで動作を続けるので、負荷状況によっては終了しない可能性がある。
	 */
	void PlayRecordedPosition() {
		cout << "Are you ready for replaying? (Yes: y or No:n) [n]";

		std::string lineBuffer;
		std::getline(std::cin, lineBuffer);
		char c = lineBuffer.c_str()[0];
		if(c == 'y'){
			for(int i = 0;i < CurrentBufferPosition;i++) {
				for(int j = 0;j < myArm.GetNumJoint();j++) {
					myArm.Joint[j]->MoveImmediately(PositionBuffer[j][i]);
				}

				for(int j = 0;j < myArm.GetNumJoint();j++) {
					while(1) {
#ifdef WIN32
						Sleep(10); // Wait 10 [ms]
#else
						usleep(10*1000);
#endif
						double delta = fabs(myArm.Joint[j]->GetCurrentPosition() - PositionBuffer[j][i]);
						if(delta < 5) {
							break;
						}
					}
				}
			}
		}
		cout <<("Done.\n");
	}

	/*
	 * 保存したポジションデータの表示
	 */
	void ShowRecordedPosition() {
		if(CurrentBufferPosition > 0){
			for(int i = 0;i < CurrentBufferPosition;i++) {
				cout << "Position[" << i << "]:";
				for(int j = 0;j < myArm.GetNumJoint();j++) {
					cout << PositionBuffer[j][i];
					if(j != myArm.GetNumJoint()-1) {
						cout << ", ";
					} else {
						cout << endl;
					}
				}
			}
		}else{
			cout << "No Record." << endl;
		}
	}

	/*
	 * 現在位置をポジションバッファに保存する。
	 */
	void RecordCurrentPosition() {
		for(int i = 0;i < myArm.GetNumJoint();i++) {
			PositionBuffer[i][CurrentBufferPosition] = myArm.Joint[i]->GetCurrentPosition();
		}
		cout << "Recorded." << endl;
		CurrentBufferPosition++;
	}

	/*
	 * ポジションバッファをクリア
	 */
	void ClearPositionBuffer() {
		cout << "Do you really want to clear registered positions? (Yes: y or No:n) [n] >> ";

		std::string lineBuffer;
		std::getline(std::cin, lineBuffer);
		char c = lineBuffer.c_str()[0];
		if(c == 'y'){
			CurrentBufferPosition = 0;
			cout << "Positions are cleared." << endl;
		}
   	}


	/*
	 * 現在位置からstepだけ関節を動かす
	 *  joint 関節番号。0～5
	 *  step 動作角度
	 */
	void MoveRelatively(int joint, double step) {
		myArm.Joint[joint]->MoveImmediately(myArm.Joint[joint]->GetCurrentPosition() + step);
	}

	/*
	 * stepの値を変更する。
	 *
	 */
	void ChangeStep() {
		cout << "Incremental/decremental step: " << step << endl;
		cout <<"Do you want to change? (Yes: y or No:n) [n]";

		std::string lineBuffer;
		std::getline(std::cin, lineBuffer);
		char c = lineBuffer.c_str()[0];
		if(c == 'y'){
			cout <<"Incremental/decremental step (0, 10]>>";
			std::getline(std::cin, lineBuffer);
			double buf = atof(lineBuffer.c_str());
			if(buf > 0, buf <= 10) {
				step = buf;
			} else {
				cout << "Wrong Value." << endl;
			}
		}
	}

	/*
	 * トルク制限値の変更
	 */
	void ChangeLimitTorque() {
		ShowLimitTorque();

		cout << "Do you want to change? (Yes: y or No:n) [n]";

		std::string lineBuffer;
		std::getline(std::cin, lineBuffer);
		char c = lineBuffer.c_str()[0];
		if(c == 'y'){
			cout << "Limit Torque [0, 100] [unit: %] : >> ";
			std::getline(std::cin, lineBuffer);
			double limit = atof(lineBuffer.c_str());
			if(limit >= 0 && limit <= 100) {
				for(int i = 0;i < myArm.GetNumJoint();i++) {
					myArm.Joint[i]->SetTorqueLimit(limit);
				}
			} else {
				cout << "Wrong Limit Value." << endl;
			}
		}
	}

	/*
	 * 速度制限値の変更
	 */
	void ChangeTargetVelocity() {
		ShowTargetVelocity();

		cout << "Do you want to change? (Yes: y or No:n) [n]";

		std::string lineBuffer;
		std::getline(std::cin, lineBuffer);
		char c = lineBuffer.c_str()[0];
		if(c == 'y'){
			cout << "Target speed (0, 30) [unit: rpm]: >> ";
			std::getline(std::cin, lineBuffer);
			double vel = atof(lineBuffer.c_str());
			if(vel > 0 && vel < 30) {
				for(int i = 0;i < myArm.GetNumJoint();i++) {
					myArm.Joint[i]->SetTargetVelocity(vel);
				}
			} else {
				cout << "Wrong Speed." << endl;
			}
		}
	}

	/*
	 * トルク制限値を表示。単位は最大のモータ出力に対する割合[％]。
	 */
	void ShowLimitTorque() {
		cout << "Limit Torque: ";
		for(int i = 0;i < myArm.GetNumJoint();i++){
			cout << myArm.Joint[i]->GetCurrentPosition();
			if(i != myArm.GetNumJoint()-1) {
				cout << ", ";
			} else {
				cout << endl;
			}
		}
	}

	/*
	 * 現在のモータの位置表示
	 */
	void ShowCurrentPosition() {
		cout << "Current Position: ";
		for(int i = 0;i < myArm.GetNumJoint();i++){
			cout << myArm.Joint[i]->GetCurrentPosition();
			if(i != myArm.GetNumJoint()-1) {
				cout << ", ";
			} else {
				cout << endl;
			}
		}
	}

	/*
	 * 現在のモータの速度表示
	 */
	void ShowCurrentVelocity() {
		cout << "Current Velocity: ";
		for(int i = 0;i < myArm.GetNumJoint();i++){
			cout << myArm.Joint[i]->GetCurrentVelocity();
			if(i != myArm.GetNumJoint()-1) {
				cout << ", ";
			} else {
				cout << endl;
			}
		}
	}

	/*
	 * 現在のモータの目標速度表示
	 */
	void ShowTargetVelocity() {
		cout << "Target Velocity: ";
		for(int i = 0;i < myArm.GetNumJoint();i++){
			cout << myArm.Joint[i]->GetTargetVelocity();
			if(i != myArm.GetNumJoint()-1) {
				cout << ", ";
			} else {
				cout << endl;
			}
		}
	}



	/*
	 * プログラム内のキーボードコマンドの表示（ヘルプ）
	 */
	void Help() {
        cout << endl << "-----------------------------------------------" << endl;
        cout << "* Show this menu (h)" << endl;            
        cout << "* Read current positions of motors (p)" << endl;
        cout << "* Read current speed of motors (v)" << endl;
        cout << "* Change speed of motors (V)" << endl;
        cout << "* Change torque-limit of motors (F)" << endl;        
        cout << "* Open gripper (o)" << endl;
        cout << "* Close gripper (c)" << endl;        
        cout << "* Finish this demo program (Esc)" << endl;        
        cout << endl;        
        cout << "* Motor position control" << endl;
        cout << " -Increase motor 1 (1)" << endl;
        cout << " -Decrease motor 1 (Shift+1)" << endl;
        cout << " -Increase motor 2 (2)" << endl;
        cout << " -Decrease motor 2 (Shift+2)" << endl;
        cout << " -Increase motor 3 (3)" << endl;
        cout << " -Decrease motor 3 (Shift+3)" << endl;
        cout << " -Increase motor 4 (4)" << endl;
        cout << " -Decrease motor 4 (Shift+4)" << endl;
        cout << " -Increase motor 5 (5)" << endl;
        cout << " -Decrease motor 5 (Shift+5)" << endl;
        cout << " -Increase motor 6 (6)" << endl;
        cout << " -Decrease motor 6 (Shift+6)" << endl;
        cout << " -Hand Close (7)" << endl;
        cout << " -Hand Open  (8)" << endl;        
        cout << " -Read and change step size (s)" << endl;
        cout << endl;
        cout << "* Teaching and replay" << endl;
        cout << " -Motor torque on (t)" << endl;
        cout << " -Motor torque off (T)" << endl;
        cout << " -Add current position (a)" << endl;
        cout << " -Clear registerd positions (c)" << endl;
        cout << " -Show registered positions (r)" << endl;
        cout << " -Replay registered positions (g)" << endl;
		cout << " -Move to initial position (o)" << endl;
        cout << "-----------------------------------------------" << endl;
	}

};




/*
 * main関数
 *
 * CConsoleDemoクラスのインスタンスを作成するのみ。
 * 主な処理はすべてCConsoleDemoクラスのコンストラクタに記述してある。
 *
 * @see CConsoleDemo
 */
int main(int argc, char* argv[])
{
	CConsoleDemo(argc, argv);
	return 0;
}
