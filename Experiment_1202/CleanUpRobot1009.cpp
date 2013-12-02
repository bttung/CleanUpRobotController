#include "ControllerEvent.h"  
#include "Controller.h"  
#include "Logger.h"  
#include <algorithm>
#include <string> 
#include <math.h> 
#include <map>
#include <string>

using namespace std;

#define PI 3.1415926535
#define ARM_RADIUS 18
#define TRUCK_RADIUS 60
#define ROTATE_ANG 0
#define FIND_OBJ_BY_ID_MODE false

// ロボットの状態
#define INIT_STATE 0			// 初期状態
#define ROT_TO_OBJ 1			// サービスからの認識結果を待っている状態
#define GO_TO_OBJ 21			// ゴミがある方向に回転している状態
#define TURN_JOINT_UP 2		// 関節を曲げてゴミを取りに行っている状態
#define GRAB_OBJ 3				// ゴミを持ってゴミ箱の方向に回転している状態
#define GRAB_FAIL 31		  // ゴミを持ってゴミ箱に向かっている状態
#define GRAB_SUCC 4				// ゴミを捨てて関節角度を元に戻している状態
#define GO_TO_TRASH_BOX 5 // 元に場所に戻る方向に回転している状態
#define TURN_JOINT_DOWN 6 // 元の場所に向かっている状態
#define GO_TO_LAST_POS 7	// 元の向きに回転している状態
#define STOP_ROBOT 8	

//角度からラジアンに変換します
#define DEG2RAD(DEG) ( (PI) * (DEG) / 180.0 )   



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
  /* @brief  ゴミを認識しゴミの位置と名前を返す
   * @return  pos ゴミの位置
   * @return  ゴミの名前
   * @return  ゴミの認識に成功した場合はtrue
   */
  bool recognizeNearestTrash(Vector3d &pos, std::string &name); 
	bool recognizeRandomTrash(Vector3d &pos, std::string &name); 
	bool recognizeNearestTrashBox(Vector3d &pos, std::string &name); 

  /* @brief  ゴミをどこに置くべきか、置くべき場所見つかったら"true"、見つからなかったら"false"が返す
	 * @param name ゴミの名前　　
   * @return pos 置くべき場所の位置
   * @return  置くべき場所をみつかった場合はtrue
   */	
	bool findPlace2PutObj(Vector3d &pos, std::string name); 
	
	void confirmThrewTrashPos(Vector3d &pos, std::string &name);


  /* @brief  位置を指定しその方向に回転を開始し、回転終了時間を返します
   * @param  pos 回転したい方向の位置
   * @param  vel 回転速度
   * @param  now 現在時間
   * @return 回転終了時間
   */
  double rotateTowardObj(Vector3d pos, double vel, double now); 
	double rotateTowardGrabPos(Vector3d pos, double vel, double now); 
	double calcHeadingAngle();

  /* @brief  位置を指定しその方向に進みます
   * @param  pos   行きたい場所
   * @param  vel   移動速度
   * @param  range 半径range以内まで移動
   * @param  now   現在時間
   * @return 到着時間
   */
  double goToObj(Vector3d pos, double vel, double range, double now);

  /* @brief  物体を掴むために向くべき方向
   * @param  pos   掴みたい座標
   * @param  robotShoulderWidth　ロボットの肩幅の半分
   * @param  grabPos	向くべき方向
   */
	bool calcGrabPos(Vector3d pos, double robotShoulderWidth, Vector3d &grabPos);

	
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
    printf("linkpos: %lf %lf %lf \n", lpos.x(), lpos.y(), lpos.z());
		printf("camerapos: %lf %lf %lf \n", cpos.x(), cpos.y(), cpos.z());
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

	m_my->setWheelVelocity(0.0, 0.0);				
	
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
  m_vel = 1.0;
  m_rotateVel = 1.0;
  // 関節の回転速度
  m_jvel = 0.6;

  // grasp初期化
  m_grasp = false;
  m_srv = NULL;
  m_sended = false;
	m_executed = false;

}  
  


double MyController::onAction(ActionEvent &evt)
{
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
				//m_my->setJointVelocity("LARM_JOINT1", 0.0, 0.0);
				m_my->setJointVelocity("LARM_JOINT4", 0.0, 0.0);
				m_my->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
				sendSceneInfo("Start");				
				//m_srv->sendMsgToSrv("Start");
				printf("Started! \n");
				m_executed = true;
			}
			break;
		}

		// 物体の方向が帰ってきた
		case 20: {
			// 送られた座標に回転する
			m_time = rotateTowardObj(nextPos, m_rotateVel, evt.time());
			//printf("debug %lf %lf \n", evt.time(), m_time);
			m_state = 21;
			m_executed = false;
			break;
		}
		
		case 21: {
			// ロボットが回転中
			//printf("debug %lf %lf \n", evt.time(), m_time);
			if(evt.time() > m_time && m_executed == false) {
				// 物体のある場所に到着したので、車輪と止め、関節を回転し始め、物体を拾う
				m_my->setWheelVelocity(0.0, 0.0);
				// 物体のある方向に回転した
				//printf("目的地の近くに移動します %lf %lf %lf \n", nextPos.x(), nextPos.y(), nextPos.z());	
				// 送られた座標に移動する
				m_time = goToObj(nextPos, m_vel*4, m_range, evt.time());
				//m_time = goToObj(nextPos, m_vel*4, 40, evt.time());
				m_state = 22;
				m_executed = false;
			}
			break;
		}

		case 22: {
			// 送られた座標に移動した
			if(evt.time() > m_time && m_executed == false) {
				m_my->setWheelVelocity(0.0, 0.0);
				printf("止める \n");
				Vector3d myPos;
				m_my->getPosition(myPos);
				double x = myPos.x();
				double z = myPos.z();
				double theta = 0;			// y方向の回転は無しと考える		
				char replyMsg[256];

				bool found = recognizeNearestTrash(m_tpos, m_tname);
				// ロボットのステートを更新
			
				if (found == true) {
					m_state = 500;				
					m_executed = false;		
					//printf("m_executed = false, state = 500 \n");		
				} else {
					//printf("Didnot found anything \n");		
					m_state = 10;
					m_executed = false;
				}

			}
			break;
		}


		case 30: {
			// 送られた座標に回転する
			m_time = rotateTowardObj(nextPos, m_rotateVel, evt.time());
			//printf("case 30 time: %lf \n", m_time);
			m_state = 31;
			m_executed = false;
			break;
		}

		// 物体を掴むために、ロボットの向く角度をズラス
		case 31: {
			if(evt.time() > m_time && m_executed == false) {
				Vector3d grabPos;
				//printf("斜めにちょっとずれた以前の時間 time: %lf \n", evt.time());
				if(calcGrabPos(nextPos, 20, grabPos)) {
					m_time = rotateTowardObj(grabPos, m_vel / 5, evt.time());
					printf("斜め grabPos :%lf %lf %lf \n", grabPos.x(), grabPos.y(), grabPos.z());
					printf("time: %lf \n", m_time);
				}
				m_state = 32;
				m_executed = false;				
			}
			break;
		}

		// 物体の方向が帰ってきた
		case 32: {
			if(evt.time() > m_time && m_executed == false) {
				// 物体のある場所に到着したので、車輪と止め、関節を回転し始め、物体を拾う
				m_my->setWheelVelocity(0.0, 0.0);
				//printf("回転を止めた evt.time %lf \n", evt.time());
				// 関節の回転を始める
				m_my->setJointVelocity("RARM_JOINT1", -m_jvel, 0.0);
				// 50°回転
				m_time = DEG2RAD(ROTATE_ANG) / m_jvel + evt.time();
				m_state = 33;
				m_executed = false;
			}
			break;
		}

		case 33: {
			// 関節回転中
			if(evt.time() > m_time && m_executed == false) {
				// 関節の回転を止める
				m_my->setJointVelocity("RARM_JOINT1", 0.0, 0.0);
				// 自分の位置の取得
				Vector3d myPos;
				m_my->getPosition(myPos);
				double x = myPos.x();
				double z = myPos.z();
				double theta = 0;									// y方向の回転は無しと考える	
				//物体を掴めるか掴めないかによって処理を分岐させる
				if(m_grasp) {											// 物体を掴んだ

					// 捨てたゴミをゴミ候補
					std::vector<std::string>::iterator it;
					// ゴミ候補を得る
					it = std::find(m_trashes.begin(), m_trashes.end(), m_tname);
					// 候補から削除する
					m_trashes.erase(it);		
					printf("erased ... \n");	

					// ゴミ箱への行き方と問い合わせする
					char replyMsg[256];

					// もっとも近いゴミ箱を探す
					//bool found = recognizeNearestTrashBox(m_trashBoxPos, m_trashBoxName);

					// ゴミを置くべき座標を探す
					bool found = findPlace2PutObj(m_trashBoxPos, m_tname); 
					if(found) {
						// ゴミ箱が検出出来た
						std::cout << "trashboxName " << m_trashBoxName << std::endl;
						sprintf(replyMsg, "AskTrashBoxRoute %6.1lf %6.1lf %6.1lf %6.1lf %6.1lf %6.1lf", 
																	x, z, theta, m_trashBoxPos.x(), m_trashBoxPos.y(), m_trashBoxPos.z());
					} else {
						sprintf(replyMsg, "AskTrashBoxPos %6.1lf %6.1lf %6.1lf", 
																	x, z, theta);
					}

					m_srv->sendMsgToSrv(replyMsg);	
					m_executed = true;	
							
				} else {					// 物体を掴めなかった、次に探す場所を問い合わせる
					// ゴミを掴めなかったもしくはゴミが無かった、次にゴミのある場所を問い合わせする
					// 逆方向に関節の回転を始める
					m_my->setJointVelocity("RARM_JOINT1", m_jvel, 0.0);
					// 50°回転
					m_time = DEG2RAD(ROTATE_ANG) / m_jvel + evt.time();
					m_state = 34;				
					m_lastFailedTrash = m_tname;
					m_executed = false;
				}		
				
			}
			break;
		}
		
		case 34: {
			if(evt.time() > m_time && m_executed == false) {
				// 関節の回転を止める
				m_my->setJointVelocity("RARM_JOINT1", 0.0, 0.0);

				// 自分の位置の取得
				Vector3d myPos;
				m_my->getPosition(myPos);
				double x = myPos.x();
				double z = myPos.z();
				double theta = 0;			
				char replyMsg[256];
				sprintf(replyMsg, "AskObjPos %6.1lf %6.1lf %6.1lf", x, z, theta);
				printf("case 34 debug %s \n", replyMsg);

				m_srv->sendMsgToSrv(replyMsg);			
				m_executed = true;
			}
			break;
		}

		case 40: {
			// 送られた座標に回転する
			m_time = rotateTowardObj(nextPos, m_rotateVel, evt.time());
			m_state = 41;
			m_executed = false;
			break;
	  }

		case 41: {
			// 送られた座標に回転中
			if(evt.time() > m_time && m_executed == false) {
				// 送られた座標に移動する
				printf("目的地の近くに移動します %lf %lf %lf \n", nextPos.x(), nextPos.y(), nextPos.z());	
				m_time = goToObj(nextPos, m_vel*4, m_range, evt.time());
				m_state = 42;
				m_executed = false;
			}
			break;
	  }

		case 42: {
			// 送られた座標に移動中
			if(evt.time() > m_time && m_executed == false) {
				// 送られた座標に到着した、 自分の位置の取得
				Vector3d myPos;
				m_my->getPosition(myPos);
				double x = myPos.x();
				double z = myPos.z();
				double theta = 0;			// y方向の回転は無しと考える	
				char replyMsg[256];

				// もっとも近いゴミ箱を探す
				bool found = recognizeNearestTrashBox(m_trashBoxPos, m_trashBoxName);
				if(found) {
					// ゴミ箱が検出出来た
					std::cout << "trashboxName " << m_trashBoxName << std::endl;
					sprintf(replyMsg, "AskTrashBoxRoute %6.1lf %6.1lf %6.1lf %6.1lf %6.1lf %6.1lf", 
																x, z, theta, m_trashBoxPos.x(), m_trashBoxPos.y(), m_trashBoxPos.z());
				} else {
					sprintf(replyMsg, "AskTrashBoxPos %6.1lf %6.1lf %6.1lf", 
																x, z, theta);
				}

				m_srv->sendMsgToSrv(replyMsg);
				m_executed = true;
			}
			break;
		}

		case 50: {
			if(evt.time() > m_time && m_executed == false) {
				Vector3d throwPos;
				//printf("斜めにちょっとずれた以前の時間 time: %lf \n", evt.time());

				// 送られた座標に到着した、 自分の位置の取得
				Vector3d myPos;
				m_my->getPosition(myPos);
				printf("robot pos %lf %lf \n", myPos.x(), myPos.z());


				// grasp中のパーツを取得します
				CParts *parts = m_my->getParts("RARM_LINK7");	
				// grasp中のパーツの座標を取得出来れば、回転する角度を逆算出来る。
				Vector3d partPos;
				if (parts->getPosition(partPos)) {
					printf("parts pos before rotate %lf %lf %lf \n", partPos.x(), partPos.y(), partPos.z());
				} 



				//if(calcGrabPos(nextPos, 20, throwPos)) {				
				if(calcGrabPos(nextPos, 20, throwPos)) {
					m_time = rotateTowardObj(throwPos, m_vel / 5, evt.time());
					printf("斜めに捨てる throwPos :%lf %lf %lf \n", throwPos.x(), throwPos.y(), throwPos.z());
					//printf("time: %lf \n", m_time);
				}
				m_state = 51;
				m_executed = false;				
			}
		
			break;
		}

		case 51: {
		  // ゴミ箱に到着したので、車輪を停止し、アームを下ろし、物体をゴミ箱に捨てる準備をする
			m_my->setWheelVelocity(0.0, 0.0);
			// grasp中のパーツを取得します
		  CParts *parts = m_my->getParts("RARM_LINK7");		
		  
			// grasp中のパーツの座標を取得出来れば、回転する角度を逆算出来る。
			Vector3d partPos;
			if (parts->getPosition(partPos)) {
				printf("parts pos after rotate %lf %lf %lf \n", partPos.x(), partPos.y(), partPos.z());
			} 

		  // releaseします
		  parts->releaseObj();		
			// ゴミが捨てられるまで少し待つ
		  sleep(1);
			// grasp終了
		  m_grasp = false;

			//confirmThrewTrashPos(m_threwPos, m_tname);
			//printf("捨てた座標：　%lf %lf %lf \n", m_threwPos.x(), m_threwPos.y(), m_threwPos.z());	
			
			// 関節の回転を始める
		  m_my->setJointVelocity("RARM_JOINT1", m_jvel, 0.0);
		  m_time = DEG2RAD(ROTATE_ANG) / m_jvel + evt.time() + 1.0;   
			m_state = 52;
			m_executed = false;
			break;
		}

		case 52: {
			// 関節が回転中
			if(evt.time() > m_time && m_executed == false) {
				// 関節が元に戻った、関節の回転を止める
				m_my->setJointVelocity("RARM_JOINT1", 0.0, 0.0);
				// 自分の位置の取得
				Vector3d myPos;
				m_my->getPosition(myPos);
				double x = myPos.x();
				double z = myPos.z();
				double theta = 0;										// y方向の回転は無しと考える	
				
				// ゴミを捨てたので、次にゴミのある場所を問い合わせする
				char replyMsg[256];
				
				if(recognizeNearestTrash(m_tpos, m_tname)) {
					m_executed = false;
					// 物体が発見された

					m_state = 500;
				} else {
					sprintf(replyMsg, "AskObjPos %6.1lf %6.1lf %6.1lf", x, z, theta);
					m_srv->sendMsgToSrv(replyMsg);
					m_executed = true;
				}
				
			}
			break;
		}
		
		case 100: {
			m_my->setJointVelocity("RARM_JOINT1", 0.0, 0.0);
			m_my->setWheelVelocity(0.0, 0.0);
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
				m_time = goToObj(nextPos, m_vel*4, m_range, evt.time());
				m_state = 810;
				m_executed = false;
			}
			break;
		}

		case 810: {
			// 送られた座標に移動中
			if(evt.time() > m_time && m_executed == false) {
				m_my->setWheelVelocity(0.0, 0.0);
				m_time = rotateTowardObj(m_lookingPos, m_rotateVel, evt.time());
				m_executed = false;
				m_state = 815;
			}
			break;
		}
		
		case 815: {
			// 送られた座標に移動中
			if(evt.time() > m_time && m_executed == false) {
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
				m_executed = false;
			}
			break;
		}

		default: {
			break;
		}

	}

  return 0.05;      
}  







void MyController::onRecvMsg(RecvMsgEvent &evt)
{  
  // 送信者取得
  std::string sender = evt.getSender();
  std::cout << "sender: " << sender << std::endl;

  // 送信者がゴミ認識サービスの場合
  if(sender == "RecogTrash")
  {

	  char *all_msg = (char*)evt.getMsg();		
		//printf("all_msg: %s \n", all_msg);
		char *delim = (char *)(" ");
		char *ctx;
		char *header = strtok_r(all_msg, delim, &ctx);

		double x = 0, y = 0, z = 0; // range = 0;
	
			
		if(strcmp(header, "ObjDir") == 0) {
			// 次に移動する座標を位置を取り出す			
			x = atof(strtok_r(NULL, delim, &ctx));
			y = atof(strtok_r(NULL, delim, &ctx));
			z = atof(strtok_r(NULL, delim, &ctx));
			m_range = atof(strtok_r(NULL, delim, &ctx));
			nextPos.set(x, y, z);
			//printf("m_range = %lf \n", m_range);
			printf("[ClientMess] ObjDir %lf %lf %lf range: %lf\n", nextPos.x(), nextPos.y(), nextPos.z(), m_range);		
			// ロボットのステートを更新

			m_state = 20;
			m_executed = false;
			return;
		}

		// 物体のある場所に到着し、アームを伸ばし、物体を掴む
		if(strcmp(header, "grab") == 0) {		
			printf("grab \n");	
			// 回転を止める
			m_my->setWheelVelocity(0.0, 0.0);
			m_state = 30;
			m_executed = false;
			return;
		}

	 if(strcmp(header, "TrashBoxDir") == 0) {
			printf("TrashBoxDir \n");
			// 次に移動する座標を位置を取り出す			
			x = atof(strtok_r(NULL, delim, &ctx));
			y = atof(strtok_r(NULL, delim, &ctx));
			z = atof(strtok_r(NULL, delim, &ctx));
			m_range = atof(strtok_r(NULL, delim, &ctx));
			//printf("range = %lf \n", m_range);
			nextPos.set(x, y, z);
			printf("[ClientMess] TrashBoxDir %lf %lf %lf range: %lf\n", nextPos.x(), nextPos.y(), nextPos.z(), m_range);
		  m_state = 40;
			m_executed = false;
			return;
	 }

	 if(strcmp(header, "ThrowTrash") == 0) {			
			m_state = 50;
			m_executed = false;
			return;
		}

		if(strcmp(header, "Finish") == 0) {	
			m_state = 100;
			m_executed = false;
			return;	
		}

		if(strcmp(header, "RandomRouteStart") == 0) {
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

			m_state = 805;
			m_executed = false;
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
	  char *all_msg = (char*)evt.getMsg();		
		printf("all_msg: %s \n", all_msg);
		char *delim = (char *)(" ");
		char *ctx;
		char *header = strtok_r(all_msg, delim, &ctx);

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



void MyController::confirmThrewTrashPos(Vector3d &pos, std::string &name) {
	if(getObj(name.c_str())) {
		SimObj *trash = getObj(name.c_str());
		//printf("created SimObj Trash \n");

		// ゴミの位置取得
		trash->getPosition(pos);
		//printf("捨てた座標：　%lf %lf %lf \n", pos.x(), pos.y(), pos.z());	 
	}	else {
		//printf("cannot find trashbox with such name \n");
	}
	return;
}


/* ゴミの名前から、捨てるべきゴミ箱の名前を検索し、そのゴミ箱の位置を探す
 * @return pos 捨てるべきの位置
 * @param trashName ゴミの名前
 */
bool MyController::findPlace2PutObj(Vector3d &pos, std::string trashName)
{
  /////////////////////////////////////////////
  ///////////ここでゴミを認識します////////////
  /////////////////////////////////////////////

  // 候補のゴミが無い場合
  if(m_trashBoxes.empty()){
    return false;
  } else {
		//printf("m_trashBoxes.size(): %d \n", m_trashBoxes.size());
	}

	std::string trashBoxName = "";
	std::map<std::string, std::string>::iterator it = m_trashTypeMap.begin();
	trashBoxName = m_trashTypeMap.find(trashName)->second;
	std::cout << trashName << " => " << trashBoxName << '\n';
	bool trashBoxExist = false;

	for(int i = 0; i < m_trashBoxes.size(); i++) {
		if(m_trashBoxes[i] == trashBoxName) {
			trashBoxExist = true;			
		}
	}	

	if(trashBoxExist) {
		if(getObj(trashBoxName.c_str())) {
			SimObj *place = getObj(trashBoxName.c_str());
			// ゴミの位置取得
			place->getPosition(pos);
			//printf("ゴミ箱の位置：　%lf %lf %lf \n", pos.x(), pos.y(), pos.z());	 
			return true;
		} else {
			printf("can not get obj with such name \n");
		}	
	} else {
		return false;
	}

  return false;
}


bool MyController::recognizeNearestTrashBox(Vector3d &pos, std::string &name)
{
	// とりあえずランダムに探させる
  /////////////////////////////////////////////
  ///////////ここでゴミを認識します////////////
  /////////////////////////////////////////////

  // 候補のゴミが無い場合
  if(m_trashBoxes.empty()){
    return false;
  } else {
		//printf("m_trashBoxes.size(): %d \n", m_trashBoxes.size());
	}

	bool found = false;

  // 自分の位置の取得
  Vector3d myPos;
  m_my->getPosition(myPos);
	
	// もっと近いゴミを検索する	
  double dis_min = 10000000000000.0;
	double distance;
	double min_idx;

	//int trashNum = rand() % m_trashes.size();
	// 現位置に検出出来るゴミを検索する
	for(int trashNum = 0; trashNum < m_trashBoxes.size(); trashNum++) {
		name = m_trashBoxes[trashNum];
		if(getObj(name.c_str())) {
			SimObj *trash = getObj(name.c_str());
			//printf("created SimObj Trash \n");

			// ゴミの位置取得
			trash->getPosition(pos);
			//printf("ゴミ箱の位置：　%lf %lf %lf \n", pos.x(), pos.y(), pos.z());	 

			distance = (myPos.x() - pos.x()) * (myPos.x() - pos.x()) + 
								 (myPos.z() - pos.z()) * (myPos.z() - pos.z()); 
			if (distance < dis_min) {
				min_idx = trashNum;
				dis_min = distance;
			}				
			found = true;
		}	else {
			//printf("cannot find trashbox with such name \n");
		}
	}

	if (found == true) {
			name = m_trashBoxes[min_idx];
			printf("nearestTrashBox .... : %s \n", name.c_str());
			SimObj *trash = getObj(name.c_str());
			// ゴミの位置取得
			trash->getPosition(pos);
	} else {
		return false;
	}

  return found;
}


bool MyController::recognizeNearestTrash(Vector3d &pos, std::string &name)
{
  /////////////////////////////////////////////
  ///////////ここでゴミを認識します////////////
  /////////////////////////////////////////////

  // 候補のゴミが無い場合
  if(m_trashes.empty()){
    return false;
  } /*else {
		printf("m_trashes.size(): %d \n", m_trashes.size());
	}*/

	bool found = false;

  // 自分の位置の取得
  Vector3d myPos;
  m_my->getPosition(myPos);
	
	// もっと近いゴミを検索する	
  double dis_min = 10000000000000.0;
	double distance;
	double min_idx;

	// 現位置に検出出来るゴミを検索する
	for(int trashNum = 0; trashNum < m_trashes.size(); trashNum++) {
		name = m_trashes[trashNum];
		if(getObj(name.c_str())) {
			SimObj *trash = getObj(name.c_str());
			//printf("created SimObj Trash \n");

			// ゴミの位置取得
			trash->getPosition(pos);
			//printf("ゴミの位置：　%lf %lf %lf \n", pos.x(), pos.y(), pos.z());	 

			distance = (myPos.x() - pos.x()) * (myPos.x() - pos.x()) + 
								 (myPos.z() - pos.z()) * (myPos.z() - pos.z()); 
			if (distance < dis_min) {
				min_idx = trashNum;
				dis_min = distance;
			}				
			found = true;
		}	else {
			//printf("cannot find obj with such name \n");
		}
	}

	if (found == true) {
			name = m_trashes[min_idx];
			printf("nearestObj trash ^^^: %s \n", name.c_str());
			SimObj *trash = getObj(name.c_str());
			// ゴミの位置取得
			trash->getPosition(pos);
	} else {
		return false;
	}

  return found;
}





bool MyController::recognizeRandomTrash(Vector3d &pos, std::string &name)
{
  /////////////////////////////////////////////
  ///////////ここでゴミを認識します////////////
  /////////////////////////////////////////////

  // 候補のゴミが無い場合
  if(m_trashes.empty()){
		broadcastMsgToSrv("Found all known trashes");
    return false;
  } /*else {
		printf("m_trashes.size(): %d \n", m_trashes.size());
	}*/

	bool found = false;

  // 自分の位置の取得
  Vector3d myPos;
  m_my->getPosition(myPos);
	
	// 現位置に検出出来るゴミを検索する

  // ここでは乱数を使ってゴミを決定します
  int trashNum = rand() % m_trashes.size();
	name = m_trashes[trashNum];
	
	// robot stop if it cannot grab nearest object, so i didnt use find nearest obj function
	// when use while() loop, robot freeze when all object found
	// donot use while() loop xxx

	/*while(!getObj(name.c_str())) {
	//for(int trashNum = 0; trashNum < m_trashes.size(); trashNum++) {
		trashNum = rand() % m_trashes.size();
		name = m_trashes[trashNum];
	}*/

	// if cannot find object by random(), iteratate through all the name of trash.
	int objFindTry = 0;	
	if(!getObj(name.c_str())) {
		while(objFindTry < m_trashes.size()) {
			name = m_trashes[objFindTry];
			if(getObj(name.c_str())) {
				found = true;
				break;
			}
			objFindTry++;
		}
	} else if(getObj(name.c_str())) {
		found = true;
	}

	if(!found) {
		//broadcastMsgToSrv("remain known obj not found");
		return false;
	}


	if(found) {
		SimObj *trash = getObj(name.c_str());
		//printf("created SimObj Trash \n");

		// ゴミの位置取得
		trash->getPosition(pos);
		printf("ゴミの位置：　%lf %lf %lf \n", pos.x(), pos.y(), pos.z());	 
	}	

	if (found) {
			//name = m_trashes[min_idx];
			printf("random Obj: %s \n", name.c_str());
			SimObj *trash = getObj(name.c_str());
			// ゴミの位置取得
			trash->getPosition(pos);
	} 

  return found;
}








void MyController::onCollision(CollisionEvent &evt) 
{
  if (m_grasp == false){  
    typedef CollisionEvent::WithC C;  
  
    //触れたエンティティの名前を得ます  
    const std::vector<std::string> & with = evt.getWith();  
  
    // 衝突した自分のパーツを得ます  
    const std::vector<std::string> & mparts = evt.getMyParts();  
  
    //　衝突したエンティティでループします  
    for(int i = 0; i < with.size(); i++){  
  
      //右手に衝突した場合  
      if(mparts[i] == "RARM_LINK7"){  
  
				//自分を取得  
				SimObj *my = getObj(myname());  
				
				//自分の手のパーツを得ます  
				CParts * parts = my->getParts("RARM_LINK7");  
				parts->graspObj(with[i]);  
	
        m_grasp = true;  
      }  
    }  
  }  
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

  
double MyController::rotateTowardObj(Vector3d pos, double velocity, double now)
{
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
	printf("current theta: %lf \n",  theta * 180 / PI);

	// 自分の位置の取得
  Vector3d myPos;
  m_my->getPosition(myPos);

  // 自分の位置からターゲットを結ぶベクトル
  Vector3d tmpPos = pos;
  tmpPos -= myPos;

	// 近すぎるなら，回転なし
	double dis = tmpPos.x() * tmpPos.x() + tmpPos.z() * tmpPos.z();
	if (dis < 1.0) {
		return 0.0;
	}	
  
  // z方向からの角度 
	double rate = tmpPos.x() / tmpPos.z();
	double targetAngle = atan(rate);
	if (tmpPos.z() < 0) {
		targetAngle += PI;
	}
	printf("targetAngle: %lf deg tmpPos.x: %lf tmpPos.z: %lf rate: %lf \n", targetAngle*180.0/PI, tmpPos.x(), tmpPos.z(), rate);

  targetAngle -= theta;
	if (targetAngle > PI) {
		targetAngle = targetAngle - 2 * PI;
	}
	printf("targetAngle: %lf deg %lf \n", targetAngle*180.0/PI, theta * 180 / PI);
	

  if (targetAngle == 0.0) {
    return 0.0;
  } else {
    // 回転すべき円周距離
    double distance = m_distance * PI * fabs(targetAngle) / (2 * PI);
    // 車輪の半径から移動速度を得る
    double vel = m_radius * velocity;
    // 回転時間(u秒)
    double time = distance / vel;
    
    // 車輪回転開始
    if (targetAngle > 0.0) {
      m_my->setWheelVelocity(-velocity, velocity);
    } else {
      m_my->setWheelVelocity(velocity, -velocity);
    }

		return now + time;
  }
}



double MyController::rotateTowardGrabPos(Vector3d pos, double velocity, double now)
{

	printf("start rotate %lf \n", now);
	//自分を取得  
	SimObj *my = getObj(myname());  
	
	//printf("向く座標 %lf %lf %lf \n", pos.x(), pos.y(), pos.z());
  // 自分の位置の取得
  Vector3d myPos;
  m_my->getPosition(myPos);



	//自分の手のパーツを得ます  
	CParts * parts = my->getParts("RARM_LINK7");  

	Vector3d partPos;
	parts->getPosition(partPos);


  // 自分の位置からターゲットを結ぶベクトル
  Vector3d tmpp = pos;
  //tmpp -= myPos;
	tmpp -= partPos;

  // y方向は考えない
  tmpp.y(0);
  
  // 自分の回転を得る
  Rotation myRot;
  m_my->getRotation(myRot);

  // エンティティの初期方向
  Vector3d iniVec(0.0, 0.0, 1.0);
      
  // y軸の回転角度を得る(x,z方向の回転は無いと仮定)
  double qw = myRot.qw();
  double qy = myRot.qy();
      
  double theta = 2*acos(fabs(qw));
	//printf("qw: %lf theta: %lf \n", qw, theta);

  if(qw*qy < 0) {
		//printf("qw * qy < 0 \n");
    theta = -1*theta;		
	}

  // z方向からの角度
 	//printf("結ぶベクトル　座標 %lf %lf %lf \n", tmpp.x(), tmpp.y(), tmpp.z());
  double tmp = tmpp.angle(Vector3d(0.0, 0.0, 1.0));
	//printf("tmp: %lf \n", tmp);
  double targetAngle = acos(tmp);
	//printf("targetAngle: %lf ---> %lf \n", targetAngle, targetAngle*180.0/PI);

  // 方向
	//printf("tmpp.x() %lf \n", tmpp.x());
  if(tmpp.x() > 0) {
		targetAngle = -1*targetAngle;
		//printf("targetAngle: %lf deg \n", targetAngle*180.0/PI);
	}
  targetAngle += theta;
	//printf("targetAngle: %lf <--- %lf \n", targetAngle, theta);

	//printf("qw: %lf qy: %lf theta: %lf tmp: %lf targetAngle: %lf \n", qw, qy, theta, tmp, targetAngle);
	
  if(targetAngle == 0.0){
		//printf("donot need rotate \n");
    return 0.0;
  }
  else 
	{
    // 回転すべき円周距離
    double distance = m_distance*PI*fabs(targetAngle)/(2*PI);

    // 車輪の半径から移動速度を得る
    double vel = m_radius*velocity;
    
    // 回転時間(u秒)
    double time = distance / vel;
    
    // 車輪回転開始
    if(targetAngle > 0.0){
      m_my->setWheelVelocity(velocity, -velocity);
    }
    else
		{
      m_my->setWheelVelocity(-velocity, velocity);
    }

		//printf("distance: %lf vel: %lf time: %lf \n", distance, vel, time);
		printf("rotate time: %lf, time to stop: %lf \n", time, now + time);
    return now + time;
  }
		
}












// object まで移動
double MyController::goToObj(Vector3d pos, double velocity, double range, double now)
{
	printf("goToObj %lf %lf %lf \n", nextPos.x(), nextPos.y(), nextPos.z());	
  // 自分の位置の取得
  Vector3d myPos;
  m_my->getPosition(myPos);
  
  // 自分の位置からターゲットを結ぶベクトル
  pos -= myPos;
  
  // y方向は考えない
  pos.y(0);

  // 距離計算
  double distance = pos.length() - range;
	//printf("distance: %lf \n", distance);
	//printf("range = %lf \n", range);

  // 車輪の半径から移動速度を得る
  double vel = m_radius*velocity;

  // 移動開始
  m_my->setWheelVelocity(velocity, velocity);

  // 到着時間取得
  double time = distance / vel;

  return now + time;
}

bool MyController::calcGrabPos(Vector3d pos, double robotShoulderWidth, Vector3d &grabPos) 
{
	// ロボットの幅の半分 16.5cm
	robotShoulderWidth = 16.5;
	//printf("物体の座標 %lf %lf %lf \n", pos.x(), pos.y(), pos.z());
	// 物体のある高さを保存暗記する
	double grabX = 0, grabY = pos.y(), grabZ = 0;

	// 自分の位置の取得
	Vector3d myPos;
	m_my->getPosition(myPos);
	//printf("ロボットの位置 %lf %lf %lf \n", myPos.x(), myPos.y(), myPos.z());
	// 自分の位置からターゲットを結ぶベクトル
 	pos -= myPos;
	//printf("結ぶ座標 %lf %lf %lf \n", pos.x(), pos.y(), pos.z());
	
	double dis = sqrt(pos.x() * pos.x() + pos.z() * pos.z());
	if(dis > robotShoulderWidth) {
		// 物体を掴むために、ロボットが結ぶベクトルからズレる角度
		double rate = robotShoulderWidth / dis;
		double ang = asin(rate);
		//printf("angle: %lf \n", ang * 180 / PI);
		grabX = cos(ang) * pos.x() + sin(ang) * pos.z();
		grabZ = cos(ang) * pos.z() - sin(ang) * pos.x();
		//printf("grabX %lf, grabZ %lf, myPos.x %lf, myPos.z %lf \n", grabX, grabZ, myPos.x(), myPos.z());

		// ロボットが向くべき座標
		grabX += myPos.x();
		grabZ += myPos.z();

		grabPos.set(grabX, grabY, grabZ);
	
		printf("向くべき座標 %lf %lf %lf \n", grabPos.x(), grabPos.y(), grabPos.z());
		return true;
	} else {
		return false;
	}
	return false;
}





extern "C" Controller * createController() {  
  return new MyController;  
}  


