#include "ControllerEvent.h"  
#include "Controller.h"  
#include "Logger.h"  
#include <time.h>
#include <sys/time.h>
#include <algorithm>
#include <string> 
#include <math.h> 
#include <map>
#include <string>

using namespace std;

#define REPLY_MESS_FILENAME "reply_msg.txt"
#define RECV_MESS_FILENAME	"recv_msg.txt"
#define LOG_FILENAME		"log.txt"

#define PI 3.1415926535
#define ARM_RADIUS 18
#define TRUCK_RADIUS 60
#define ROTATE_ANG 0
#define FIND_OBJ_BY_ID_MODE false
#define UPDATE_INTERVAL 0.05

// ロボットの状態
#define INIT_STATE 0			// 初期状態
#define ROT_TO_OBJ 1			// サービスからの認識結果を待っている状態
#define GO_TO_OBJ 21			// ゴミがある方向に回転している状態
#define TURN_JOINT_UP 2			// 関節を曲げてゴミを取りに行っている状態
#define GRAB_OBJ 3				// ゴミを持ってゴミ箱の方向に回転している状態
#define GRAB_FAIL 31			// ゴミを持ってゴミ箱に向かっている状態
#define GRAB_SUCC 4				// ゴミを捨てて関節角度を元に戻している状態
#define GO_TO_TRASH_BOX 5		// 元に場所に戻る方向に回転している状態
#define TURN_JOINT_DOWN 6		// 元の場所に向かっている状態
#define GO_TO_LAST_POS 7		// 元の向きに回転している状態
#define STOP_ROBOT 8	

//角度からラジアンに変換します
#define DEG2RAD(DEG) ( (PI) * (DEG) / 180.0 )   
#define RAD2DEG(RAD) ( (RAD) * 180.0 / (PI) )


class Utility {
public:
	void AppendString2File(string msg, string _filename);
};

void Utility::AppendString2File(string msg, string _filename) {
	char filename[256];
	sprintf(filename, "%s", _filename.c_str());
	
	FILE *fw = fopen(filename, "a");
	fprintf(fw, "%s\n", msg.c_str());
	fclose(fw);
	return;
}


class MyController : public Controller {  
public:  
	void onInit(InitEvent &evt);  
	double onAction(ActionEvent&);  
	void onRecvMsg(RecvMsgEvent &evt); 
	void onCollision(CollisionEvent &evt); 

	char* sendSceneInfo(std::string header = "AskRandomRoute", int CamID = 1);
	void setCameraPosition(double angle, int camID);
	void setRobotHeadingAngle(double angle);
	void setRobotPosition(double x, double z);

	/* @brief  位置を指定しその方向に回転を開始し、回転終了時間を返します
	* @param  pos 回転したい方向の位置
	* @param  vel 回転速度
	* @param  now 現在時間
	* @return 回転終了時間
	*/
	double rotateTowardObj(Vector3d pos, double vel, double now); 

	double calcHeadingAngle();

	/* @brief  位置を指定しその方向に進みます
	* @param  pos   行きたい場所
	* @param  vel   移動速度
	* @param  range 半径range以内まで移動
	* @param  now   現在時間
	* @return 到着時間
	*/
	double goToObj(Vector3d pos, double vel, double range, double now);
	
private:
	RobotObj *m_my;

	int m_CamID;
	// ゴミの場所
	Vector3d m_tpos;  
	// 捨てたゴミの座標
	Vector3d m_threwPos;
	// ゴミ箱の場所
	Vector3d m_trashBoxPos;  
	// 目的地の座標
	Vector3d nextPos;
	Vector3d m_lookingPos;


	// ゴミの名前
	std::string m_tname;  
	std::string m_lastFailedTrash;
	// ゴミ箱の名前
	std::string m_trashBoxName;  

	int m_trashNum;

	// ゴミ候補オブジェクト
	std::vector<std::string> m_trashes;

	// ゴミ箱候補オブジェクト
	std::vector<std::string> m_trashBoxes;

	// ゴミのタイプ及び入れるべきゴミ箱
	std::map<std::string, std::string> m_trashTypeMap;


	/* ロボットの状態
	 * 0 初期状態
	 * 1 サービスからの認識結果を待っている状態
	 * 2 ゴミがある方向に回転している状態
	 * 3 関節を曲げてゴミを取りに行っている状態
	 * 4 ゴミを持ってゴミ箱の方向に回転している状態
	 * 5 ゴミを持ってゴミ箱に向かっている状態
	 * 6 ゴミを捨てて関節角度を元に戻している状態
	 * 7 元に場所に戻る方向に回転している状態
	 * 8 元の場所に向かっている状態
	 * 9 元の向きに回転している状態
	 */
	int m_state; 

	// 車輪の角速度
	double m_vel;
	double m_rotateVel;

	// 関節の回転速度
	double m_jvel;

	// 車輪半径
	double m_radius;

	// 車輪間距離
	double m_distance;

	// 移動終了時間
	double m_time;

	// 初期位置
	Vector3d m_inipos;

	// grasp中かどうか
	bool m_grasp;

	// ゴミ認識サービス
	BaseService *m_srv;

	// サービスへのリクエスト複数回送信を防ぐ
	bool m_sended;
	// 動作を実行したかどうかのフラグ、動作を複数回実行を防ぐ
	bool m_executed;
	double m_range;
	double m_lookObjFlg;

	Utility	m_util;
};  


void MyController::setCameraPosition(double angle, int camID) {
	double xDir = sin(DEG2RAD(angle));
	double zDir = cos(DEG2RAD(angle));
	m_my->setCamDir(Vector3d(xDir, 0.0, zDir), camID);
	printf("camera Dir converted \n");
	return;
}

void MyController::setRobotHeadingAngle(double angle) {
	m_my->setAxisAndAngle(0, 1.0, 0, DEG2RAD(angle));
	return;
}

void MyController::setRobotPosition(double x, double z) {
	Vector3d myPos;
	m_my->getPosition(myPos);
	double y = myPos.y();

	m_my->setPosition(x, y, z);
	return;
}

char* MyController::sendSceneInfo(std::string header, int camID) {
	
	m_my->setWheelVelocity(0.0, 0.0);		
	Vector3d myPos;
	m_my->getPosition(myPos);
	double x = myPos.x();
	double z = myPos.z();
	double theta = calcHeadingAngle();			// y方向の回転は無しと考える	

	// カメラがついているリンク名取得
	std::string link = ""; //"WAIST_LINK0"; //m_my->getCameraLinkName(3);
	if (camID < 3) {
		link = m_my->getCameraLinkName(camID);
	} else {
		link = "WAIST_LINK0";
	}

	//const dReal *lpos = m_my->getParts(link.c_str())->getPosition();
	Vector3d lpos;
	m_my->getParts(link.c_str())->getPosition(lpos);

	// カメラの位置取得(リンク座標系)
	Vector3d cpos;
	m_my->getCamPos(cpos, camID);

	// カメラの位置(絶対座標系, ロボットの回転はないものとする)
	//printf("linkpos: %lf %lf %lf \n", lpos.x(), lpos.y(), lpos.z());
	//printf("camerapos: %lf %lf %lf \n", cpos.x(), cpos.y(), cpos.z());
	Vector3d campos(lpos.x() + cpos.z() * sin(DEG2RAD(theta)), 
									lpos.y() + cpos.y(), 
									lpos.z() + cpos.z() * cos(DEG2RAD(theta)));
	//Vector3d campos(cpos.x(), cpos.y(), cpos.z());

	// カメラの方向取得(ロボットの回転,関節の回転はないものとする)
	Vector3d cdir;
	m_my->getCamDir(cdir, camID);

	char *replyMsg = new char[1024];
	sprintf(replyMsg, "%s %6.1lf %6.1lf %6.1lf %6.1lf %6.1lf %6.1lf %6.1lf %6.1lf %6.1lf", 
						header.c_str(), x, z, theta, campos.x(), campos.y(), campos.z(), cdir.x(), cdir.y(), cdir.z());
	printf("%s \n", replyMsg);

	m_srv->sendMsgToSrv(replyMsg);
	printf("make sleep\n");
	usleep(100000);	//0.1second

	m_util.AppendString2File(string(replyMsg), REPLY_MESS_FILENAME);

	return replyMsg;
}


void MyController::onInit(InitEvent &evt) 
{  
	m_my = getRobotObj(myname());

	// 初期位置取得
	m_my->getPosition(m_inipos);

	//v3の追加点
	// ゴミがある方向にカメラを向ける
	m_my->setCamDir(Vector3d(0.0, -1.0, 1.0),1);
	m_my->setCamDir(Vector3d(0.0, -1.0, 1.0),2);

	// 車輪の半径と車輪間隔距離
	m_radius = 10.0;
	m_distance = 10.0;

	m_time = 0.0;

	// 車輪の半径と車輪間距離設定
	m_my->setWheel(m_radius, m_distance);
	m_state = 0;
	srand((unsigned)time( NULL ));


	// 車輪の回転速度
	m_vel = 1.0;	//1.0;
	m_rotateVel = 0.6;	//1.0;
	// 関節の回転速度
	m_jvel = 0.6;
	m_lookObjFlg = 0.0;

	// grasp初期化
	m_grasp = false;
	m_srv = NULL;
	m_sended = false;
	m_executed = false;

}  
  


double MyController::onAction(ActionEvent &evt)
{
/*	if(!checkService("RecogTrash")){
		m_srv == NULL;
		m_state = 0;
		return UPDATE_INTERVAL;
	}
	
	if(m_srv == NULL){
		// ゴミ認識サービスが利用可能か調べる
		if(checkService("RecogTrash")){
			// ゴミ認識サービスに接続
			m_srv = connectToService("RecogTrash");
			return UPDATE_INTERVAL;
		}
	}*/


	//if(evt.time() < m_time) printf("state: %d \n", m_state);
	switch(m_state) {
		// 初期状態
		case 0: {
			if(m_srv == NULL){
				// ゴミ認識サービスが利用可能か調べる
				if(checkService("RecogTrash")){
					// ゴミ認識サービスに接続
					m_srv = connectToService("RecogTrash");
				}
			} else if(m_srv != NULL && m_executed == false){  
				//rotate toward upper
				m_my->setJointVelocity("LARM_JOINT4", -m_jvel, 0.0);
				m_my->setJointVelocity("RARM_JOINT4", -m_jvel, 0.0);
				// 50°回転
				m_time = DEG2RAD(ROTATE_ANG) / m_jvel + evt.time();
				m_state = 5;
				m_executed = false;			
			}
			break;
		}

		case 5: {
			if(evt.time() > m_time && m_executed == false) {
				m_my->setJointVelocity("LARM_JOINT4", 0.0, 0.0);
				m_my->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
				sendSceneInfo("Start");				
				printf("Started! \n");
				m_executed = true;
			}
			break;
		}


		case 800: {
			if(evt.time() > m_time && m_executed == false) {
				sendSceneInfo();
				m_executed = true;
			}
			break;
		}

		case 805: {
			if(evt.time() > m_time && m_executed == false) {
				// 送られた座標に移動する
				double range = 0;
				m_time = rotateTowardObj(nextPos, m_rotateVel, evt.time());
				m_state = 807;
				m_executed = false;
			}
			break;
	  	}

		case 807: {
			if(evt.time() > m_time && m_executed == false) {
				m_my->setWheelVelocity(0.0, 0.0);
				printf("make sleep\n");
				usleep(100000);	//0.1second			
	
				printf("移動先 x: %lf, z: %lf \n", nextPos.x(), nextPos.z());				
				m_time = goToObj(nextPos, m_vel*4, m_range, evt.time());
				
				if (m_lookObjFlg == 1.0) {
					printf("looking to Obj \n");				
					m_state = 810;
				} else {
					printf("go to next node \n");				
					m_state = 815;
				}

				m_executed = false;
			}
			break;
		}

		case 810: {
			// 送られた座標に移動中
			if(evt.time() > m_time && m_executed == false) {
				m_my->setWheelVelocity(0.0, 0.0);
				printf("make sleep\n");
				usleep(100000);	//0.1second

				m_time = rotateTowardObj(m_lookingPos, m_rotateVel, evt.time());
				m_executed = false;
				m_state = 815;
			}
			break;
		}
		
		case 815: {
			// 送られた座標に移動中
			if(evt.time() > m_time && m_executed == false) {
				m_my->setWheelVelocity(0.0, 0.0);
				printf("make sleep\n");
				usleep(100000);	//0.1second
				
				sendSceneInfo();
				printf("sent data to SIGViewer \n");				
				m_executed = true;
			}
			break;
		}

		case 920: {
			// 送られた座標に回転する
			m_time = rotateTowardObj(nextPos, m_rotateVel, evt.time());
			m_state = 921;
			m_executed = false;
			break;
		}

		case 921: {
			// ロボットが回転中
			if(evt.time() > m_time && m_executed == false) {
				m_my->setWheelVelocity(0.0, 0.0);
				printf("make sleep\n");
				usleep(100000);	//0.1second

				m_executed = false;
			}
			break;
		}

		default: {
			break;
		}

	}

	return UPDATE_INTERVAL;

}  




void MyController::onRecvMsg(RecvMsgEvent &evt)
{  
	// 送信者取得
	std::string sender = evt.getSender();
	std::cout << "sender: " << sender << std::endl;

	char *all_msg = (char*)evt.getMsg();		
	printf("all_msg: %s \n", all_msg);

	// debug
	char all_msg_bak[256];
	sprintf(all_msg_bak, "%s", all_msg);
	printf("___all_msg_bak: %s\n", all_msg_bak);

	char *delim = (char *)(" ");
	char *ctx;
	char *header = strtok_r(all_msg, delim, &ctx);
	printf("header: %s\n", header);

	if (strcmp(header, "RESET") == 0) {
		printf("Received RESET \n");
		setRobotPosition(0, -50);	
		setRobotHeadingAngle(0);
		setCameraPosition(0, 3);
		printf("Reseted RobotPosition \n");
		//char* replyMsg = sendSceneInfo();
		sendSceneInfo("Start");
		m_executed = true;
		return;
	}


	// 送信者がゴミ認識サービスの場合
	if(sender == "RecogTrash") {
		double x = 0, y = 0, z = 0; // range = 0;

		if(strcmp(header, "RandomRouteStart") == 0) {
			// debug
			printf("append msg 2 file\n");
			m_util.AppendString2File(string(all_msg_bak), RECV_MESS_FILENAME);

			m_state = 800;
			m_executed = false;
			return;
		}

		if(strcmp(header, "RandomRouteArrived") == 0) {
			//printf("onRecv RandomRouteArrived \n");
			m_state = 800;
			m_executed = false;
			return;
		}

		if(strcmp(header, "RandomRoute") == 0) {
			x = atof(strtok_r(NULL, delim, &ctx));			
			z = atof(strtok_r(NULL, delim, &ctx));	
			m_range = atof(strtok_r(NULL, delim, &ctx));	
			nextPos.set(x, y, z);

			double lookingX = atof(strtok_r(NULL, delim, &ctx));
			double lookingZ = atof(strtok_r(NULL, delim, &ctx));
			m_lookingPos.set(lookingX, 0, lookingZ);

			m_lookObjFlg = atof(strtok_r(NULL, delim, &ctx));
			printf("m_lookObjFlg: %lf \n", m_lookObjFlg);

			m_state = 805;
			m_executed = false;
			return;
		} 

		if(strcmp(header, "SetRobotPosition") == 0) {
			printf("TELEPORT: %s\n", ctx);
			double x = atof(strtok_r(NULL, delim, &ctx));		
			double z = atof(strtok_r(NULL, delim, &ctx));
			double lookX = atof(strtok_r(NULL, delim, &ctx));
			double lookZ = atof(strtok_r(NULL, delim, &ctx));
			double disX	 = lookX - x;
			double disZ  = lookZ - z;
			
			double angle = RAD2DEG(atan(disX / disZ));
			setRobotPosition(x, z);
			setRobotHeadingAngle(angle);
			
			printf("SetRobotPostion x: %lf, z: %lf, angle: %lf\n", x, z, angle);
			//printf("SetRobotPostion x: %lf, z: %lf\n", x, z);

			char* replyMsg = sendSceneInfo();
			m_executed = true;
			return;	
		}

		if(strcmp(header, "GoForwardVelocity") == 0) {
			double wheelVel = atof(strtok_r(NULL, delim, &ctx));
			printf("GoForwardVelocity coef: %lf \n", wheelVel);
			wheelVel *= m_vel;
			m_my->setWheelVelocity(wheelVel * 10., wheelVel * 10.);			
			//sendSceneInfo();				
			return;
		}

		if(strcmp(header, "RotateVelocity") == 0) {
			double wheelVel = atof(strtok_r(NULL, delim, &ctx));
			printf("RotateVelocity coef: %lf \n", wheelVel);
			wheelVel *= m_vel;
			m_my->setWheelVelocity(-wheelVel, wheelVel);				
			//sendSceneInfo();				
			return;
		}

		if(strcmp(header, "Stop") == 0) {
			printf("Stop joyStick \n");
			m_my->setWheelVelocity(0.0, 0.0);				
			sendSceneInfo();			
			return;
		}

		if (strcmp(header, "CamID") == 0) {
			printf("Setting CameraID \n");
			m_CamID = atoi(strtok_r(NULL, delim, &ctx));
			return;
		}

		if (strcmp(header, "CaptureData") == 0) {
			printf("SetRobotPosition \n");
			double x = atof(strtok_r(NULL, delim, &ctx));		
			double z = atof(strtok_r(NULL, delim, &ctx));
			setRobotPosition(x, z);
			double angle = atof(strtok_r(NULL, delim, &ctx));
			printf("setRobotHeadingAngle: %lf \n", angle);
			setRobotHeadingAngle(angle);
			char* replyMsg = sendSceneInfo();
			//m_srv->sendMsgToSrv(replyMsg);
			m_executed = true;
			return;			
		} 

	} else if (sender == "SIGViewer") {
		printf("mess from SIGViewer \n");

		if(strcmp(header, "CameraAngle") == 0) {
			double angle = atof(strtok_r(NULL, delim, &ctx));
			printf("CameraAngle: %lf \n", angle);
			setCameraPosition(angle, 3);
			char* replyMsg = sendSceneInfo();
			m_executed = true;
			return;
		} else if (strcmp(header, "RobotAngle") == 0) {
			printf("rorate robor start \n");
			double angle = atof(strtok_r(NULL, delim, &ctx));
			printf("RobotHeadingAngle: %lf \n", angle);
			setRobotHeadingAngle(angle);
			char* replyMsg = sendSceneInfo();
			//m_srv->sendMsgToSrv(replyMsg);
			m_executed = true;
			return;			
		} else if (strcmp(header, "RotateDir") == 0) {
			// 次に移動する座標を位置を取り出す			
			double x = atof(strtok_r(NULL, delim, &ctx));
			double z = atof(strtok_r(NULL, delim, &ctx));
			nextPos.set(x, 0, z);

			printf("RotateDir %lf %lf \n", nextPos.x(), nextPos.z());		
			// ロボットのステートを更新
			m_state = 920;
			m_executed = false;
			return;
		} else if(strcmp(header, "RobotPosition") == 0) {
			double x = atof(strtok_r(NULL, delim, &ctx));		
			double z = atof(strtok_r(NULL, delim, &ctx));
			setRobotPosition(x, z);
			return;
		}
	}

}  



void MyController::onCollision(CollisionEvent &evt) 
{
}


double MyController::calcHeadingAngle()
{
	// 自分の回転を得る
	Rotation myRot;
	m_my->getRotation(myRot);
      
	// y軸の回転角度を得る(x,z方向の回転は無いと仮定)
	double qw = myRot.qw();
	double qy = myRot.qy();      
	double theta = 2*acos(fabs(qw));

	if (qw * qy < 0) {   
		theta = -1 * theta;
	}

	return theta * 180.0 / PI;
}

 
/*double MyController::rotateTowardObj(Vector3d pos, double velocity, double now)
{  	
	// 自分の位置の取得
  	Vector3d myPos;
  	m_my->getPosition(myPos);
	printf("ロボットの現在位置: x: %lf, z %lf \n", myPos.x(), myPos.z());

  	// 自分の位置からターゲットを結ぶベクトル
  	Vector3d tmpPos = pos;  
	tmpPos -= myPos;

	// y方向は考えない
	tmpPos.y(0);

	// 自分の回転を得る
  	Rotation myRot;
  	m_my->getRotation(myRot);
      
	// y軸の回転角度を得る(x,z方向の回転は無いと仮定)
	double qw = myRot.qw();
	double qy = myRot.qy();
	
	double theta = 2 * acos(fabs(qw));
	
	if (qw * qy < 0) {
		theta = -1 * theta;
	}

	printf("ロボットが向いている角度 current theta: %lf(deg) \n",  theta * 180 / PI);

	//// 近すぎるなら，回転なし
	//double dis = tmpPos.x() * tmpPos.x() + tmpPos.z() * tmpPos.z();
	//if (dis < 1.0) {
	//	return 0.0;
	//}	
  
  	// z方向からの角度 
	double rate = tmpPos.x() / tmpPos.z();
	double targetAngle = atan(rate);

	if (tmpPos.z() < 0) {
		targetAngle += PI;
	}
	printf("回転する角度 targetAngle: %lf(deg) 結ぶベクトル tmpPos.x: %lf, tmpPos.z: %lf, rate: %lf \n", targetAngle*180.0/PI, tmpPos.x(), tmpPos.z(), rate);

	targetAngle -= theta;
	
	if (targetAngle > PI) {
		targetAngle = targetAngle - 2 * PI;
	}
	
	printf("targetAngle: %lf(deg) currentAngle: %lf(deg) \n", targetAngle*180.0/PI, theta * 180.0 / PI);	

	if (targetAngle == 0.0) {
		printf("donot need to rotate \n");
		return 0.0;
	} else {
		// 回転すべき円周距離
		double distance = m_distance * PI * fabs(targetAngle) / (2 * PI);
		printf("distance: %lf \n", distance);	

		// 車輪の半径から移動速度を得る
		double vel = m_radius * velocity;
		printf("radius: %lf, velocity: %lf, vel: %lf \n", m_radius, velocity, vel);

		// 回転時間(u秒)
		double time = distance / vel;
		printf("rotateTime: %lf = dis: %lf / vel: %lf\n", time, distance, vel);

		// 車輪回転開始
		if (targetAngle > 0.0) {
			m_my->setWheelVelocity(-velocity, velocity);
		} else {
			m_my->setWheelVelocity(velocity, -velocity);
		}

		return now + time;
	}
}*/




double MyController::rotateTowardObj(Vector3d pos, double velocity, double now)
{
	// 自分の位置の取得
	Vector3d myPos;
	m_my->getPosition(myPos);

	// 自分の位置からターゲットを結ぶベクトル
	Vector3d tmpp = pos;
	tmpp -= myPos;

	// y方向は考えない
	tmpp.y(0);

	// 近すぎるなら，回転なし
	double dis = tmpp.x() * tmpp.x() + tmpp.z() * tmpp.z();
	if (dis < 1.0) {
		printf("近すぎる回転手なくても良い\n");		
		return 0.0;
	}	

	// 自分の回転を得る
	Rotation myRot;
	m_my->getRotation(myRot);
	printf("ロボットの現在位置: x: %lf, z %lf \n", myPos.x(), myPos.z());
	
	// エンティティの初期方向
	Vector3d iniVec(0.0, 0.0, 1.0);
	  
	// y軸の回転角度を得る(x,z方向の回転は無いと仮定)
	double qw = myRot.qw();
	double qy = myRot.qy();
	  
	double theta = 2*acos(fabs(qw));

	if (qw*qy < 0) {
		theta = -1*theta;
	}

	printf("ロボットが向いている角度 current theta: %lf(deg) \n",  theta * 180 / PI);
	
	// z方向からの角度
	double tmp = tmpp.angle(Vector3d(0.0, 0.0, 1.0));
	double targetAngle = acos(tmp);

	// 方向
	if(tmpp.x() > 0) {
		targetAngle = -1*targetAngle;
	}
	targetAngle += theta;

	//printf("000 targetAngle: %lf(deg) currentAngle: %lf(deg) \n", targetAngle*180.0/PI, theta * 180.0 / PI);

	if (targetAngle > PI) {
		targetAngle = targetAngle - 2 * PI;
	}

	//printf("111 targetAngle: %lf(deg) currentAngle: %lf(deg) \n", targetAngle*180.0/PI, theta * 180.0 / PI);
	
	if (targetAngle < -PI) {
		targetAngle = targetAngle + 2 * PI;
	}

	//printf("222 targetAngle: %lf(deg) currentAngle: %lf(deg) \n", targetAngle*180.0/PI, theta * 180.0 / PI);

	if (targetAngle == 0.0) {
		return 0.0;
	} else {
		// 回転すべき円周距離
		double distance = m_distance*PI*fabs(targetAngle)/(2*PI);
		printf("distance: %lf \n", distance);	  

		// 車輪の半径から移動速度を得る
		double vel = m_radius*velocity;
		printf("radius: %lf, velocity: %lf, vel: %lf \n", m_radius, velocity, vel);		

		// 回転時間(u秒)
		double time = distance / vel;
		printf("rotateTime: %lf = dis: %lf / vel: %lf\n", time, distance, vel);
		
		// 車輪回転開始
		if (targetAngle > 0.0) {
			 m_my->setWheelVelocity(velocity, -velocity);
		} else {
			m_my->setWheelVelocity(-velocity, velocity);
		}

		printf("make sleep\n");
		usleep(100000);	//0.1second

		return now + time;
	}
}





// object まで移動
double MyController::goToObj(Vector3d nextPos, double velocity, double range, double now)
{
	printf("goToObj関数内　goToObj %lf %lf %lf \n", nextPos.x(), nextPos.y(), nextPos.z());	
  	// 自分の位置の取得
  	Vector3d myPos;
 	m_my->getPosition(myPos);
	printf("goToObj関数内 ロボットの現在位置: x: %lf, z %lf \n", myPos.x(), myPos.z()); 

	// 自分の位置からターゲットを結ぶベクトル
	Vector3d pos = nextPos;
	pos -= myPos;

	// y方向は考えない
	pos.y(0);

	// 距離計算
	double distance = pos.length() - range;
	printf("gotoObj distance: %lf, range: %lf\n", distance, range);

	// 車輪の半径から移動速度を得る
	double vel = m_radius*velocity;
	printf("radius: %lf, velocity: %lf, vel: %lf \n", m_radius, velocity, vel);

	// 移動開始
	m_my->setWheelVelocity(velocity, velocity);
	printf("make sleep\n");
	usleep(100000);	//0.1second
	printf("setVelocity: %lf %lf \n", velocity, velocity);

	// 到着時間取得
	double time = distance / vel;
	printf("goToObj time: %lf \n", time);

	return now + time;
}


extern "C" Controller * createController() {  
  return new MyController;  
}  


