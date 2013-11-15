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

class Node2D {
public:
	double x;
	double y;
	Node2D(double x, double y) {
		this->x = x;
		this->y = y;
	};
	void setPosition(double x, double y) {
		this->x = x;
		this->y = y;
	};
	void toString() {
		printf("(%lf, %lf) \n", this->x, this->y);
	}
};

class Obstacle {
public:
	double x;
	double y;
	double width;
	double height;
	double x_min;
	double x_max;
	double y_min;
	double y_max;
	
	Obstacle() {

	};
	Obstacle(double x, double y, double width, double height) {
		this->x = x;
		this->y = y;
		this->width = width;
		this->height = height;
		this->x_min = x - width / 2;
		this->x_max = x + width / 2;
		this->y_min = y - height / 2;
		this->y_max = y + height / 2;	
	};

	void setPosition(double x, double y, double width, double height) {
		this->x = x;
		this->y = y;
		this->width = width;
		this->height = height;
		this->x_min = x - width / 2;
		this->x_max = x + width / 2;
		this->y_min = y - height / 2;
		this->y_max = y + height / 2;	
	};

};


class MyController : public Controller {  
public:  
  void onInit(InitEvent &evt);  
  double onAction(ActionEvent&);  
  void onRecvMsg(RecvMsgEvent &evt); 
  void onCollision(CollisionEvent &evt); 

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

	int getPointPositionIndex(Node2D pos, Obstacle obs);
	int getGrabPositionIndex(Node2D objPos, Obstacle obs);
	Node2D getGrabPosition(Node2D objPos, Obstacle obs);
	std::vector<Node2D> calcRoute(Node2D startPos, Node2D goalPos, Obstacle obs);
	std::vector<Node2D> calcFullRoute(Node2D startPos, Node2D goalPos, std::vector<Obstacle> roomObs);

private:
  RobotObj *m_my;

  // ゴミの場所
  Vector3d m_tpos;  
	// 捨てたゴミの座標
	Vector3d m_threwPos;
  // ゴミ箱の場所
  Vector3d m_trashBoxPos;  
	// 目的地の座標
	Vector3d nextPos;


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

	// 障害物の座標
	//Obstacle m_obstacle(0, 0, 0, 0);
	std::map<std::string, Obstacle> m_obstacleMap;
	std::vector<Obstacle> m_roomObs;

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

	// ルート
	std::vector<Node2D> m_route;
	// node Id
	int m_nodeId;	


	Obstacle m_table_0;//(0., 0., 105., 60.); // scale 1.5
	Obstacle m_table_1;//(100., -100., 130., 60.);
	Obstacle m_table_2;//(-50., -90., 70., 40.);
	Obstacle m_wagon_0;//(100., 100., 60., 65.);
	Obstacle m_trashbox_0;//(-150., 0., 40., 20.);
	Obstacle m_trashbox_1;//(-150., -50., 40., 20.);
	Obstacle m_trashbox_2;//(-150., -100., 40., 20.);
	
	bool m_isOstacleCaculated;

};  

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



/*
sePetbottle_2l_half_c02 	petbottle_0 	-40.000 	64.650 	40.000 	33.3 	ワゴン 		○
sePetbottle_2l_empty_c02 	petbottle_1 	-20.000 	64.650 	40.000 	33.3 	リサイクル 		○
sePetbottle_2l_empty_c01 	petbottle_2 	0.000 	64.650 	40.000 	33.3 	リサイクル 		○
sePetbottle_500ml_full_c02 	petbottle_3 	20.000 	59.150 	40.000 	22.3 	ワゴン 		○
sePetbottle_500ml_empty_c01 	petbottle_4 	40.000 	59.150 	40.000 	22.3 	リサイクル 	○ 	
seBanana 	banana 	-40.000 	48.890 	60.000 	1.78 	ワゴン 	○ 	
seChigarette 	chigarette 	-20.000 	51.250 	60.000 	6.5 	ワゴン 		
seChocolate 	chocolate 	0.000 	48.350 	60.000 	0.7 	ワゴン 		
seMayonaise_full 	mayonaise_0 	20.000 	55.050 	60.000 	14.1 	ワゴン 		
seMayonaise_empty 	mayonaise_1 	40.000 	55.050 	60.000 	14.1 	リサイクル 		
seMugcup_c01 	mugcup 	-40.000 	51.990 	80.000 	7.98 	ワゴン 	○ 	
seCannedjuice_200ml_c01 	can_0 	-20.000 	52.535 	80.000 	9.07 	カン 		
seCannedjuice_200ml_c02 	can_1 	0.000 	52.535 	80.000 	9.07 	カン 	○ 	
seCannedjuice_350ml_c01 	can_2 	20.000 	54.250 	80.000 	12.5 	カン 		
seCannedjuice_350ml_c02 	can_3 	40.000 	54.250 	80.000 	12.5 	カン 		
*/

  // ここではゴミの名前が分かっているとします
	//m_trashNum = 0;
  //m_trashes.push_back("petbottle_0"); 
  //m_trashes.push_back("petbottle_1"); 
  //m_trashes.push_back("petbottle_2"); 
  //m_trashes.push_back("petbottle_3"); 
  //m_trashes.push_back("petbottle_4"); 
  m_trashes.push_back("banana"); 
  //m_trashes.push_back("chigarette"); 
  m_trashes.push_back("chocolate"); 
  //m_trashes.push_back("mayonaise_0"); 
  m_trashes.push_back("mayonaise_1"); 
  //m_trashes.push_back("mugcup"); 
  m_trashes.push_back("can_0"); 
  m_trashes.push_back("can_1"); 
  //m_trashes.push_back("can_2"); 
  //m_trashes.push_back("can_3"); 


	// ゴミ箱の名前を何個か入れる 
	m_trashBoxes.push_back("trashbox_0");
	m_trashBoxes.push_back("trashbox_1");
	m_trashBoxes.push_back("trashbox_2"); 
	m_trashBoxes.push_back("wagon_0"); 

	// ゴミのタイプ及び入れるべきゴミ箱の設定
	m_trashTypeMap.insert(std::pair<std::string, std::string>("petbottle_0", "wagon_0"));
	m_trashTypeMap.insert(std::pair<std::string, std::string>("petbottle_1", "trashbox_0"));
	m_trashTypeMap.insert(std::pair<std::string, std::string>("petbottle_2", "trashbox_0"));
	m_trashTypeMap.insert(std::pair<std::string, std::string>("petbottle_3", "wagon_0"));
	m_trashTypeMap.insert(std::pair<std::string, std::string>("petbottle_4", "trashbox_0"));
	m_trashTypeMap.insert(std::pair<std::string, std::string>("banana", "wagon_0"));
	m_trashTypeMap.insert(std::pair<std::string, std::string>("chigarette", "wagon_0"));
	m_trashTypeMap.insert(std::pair<std::string, std::string>("chocolate", "wagon_0"));
	m_trashTypeMap.insert(std::pair<std::string, std::string>("mayonaise_0", "wagon_0"));
	m_trashTypeMap.insert(std::pair<std::string, std::string>("mayonaise_1", "trashbox_0"));
	m_trashTypeMap.insert(std::pair<std::string, std::string>("mugcup", "wagon_0"));
	m_trashTypeMap.insert(std::pair<std::string, std::string>("can_0", "trashbox_2"));
	m_trashTypeMap.insert(std::pair<std::string, std::string>("can_1", "trashbox_2"));
	m_trashTypeMap.insert(std::pair<std::string, std::string>("can_2", "trashbox_2"));
	m_trashTypeMap.insert(std::pair<std::string, std::string>("can_3", "trashbox_2"));
	
	// 障害物の初期化
	//m_obstacle.x = 0;
	//m_obstacle.y = 60;
	//m_obstacle.width = 120;
	//m_obstacle.height = 60;


	//route
	m_route.clear();

  // 車輪の回転速度
  m_vel = 1.0;

  // 関節の回転速度
  m_jvel = 0.6;

  // grasp初期化
  m_grasp = false;
  m_srv = NULL;
  m_sended = false;
	m_executed = false;

	m_isOstacleCaculated = false;

	m_nodeId = 0;
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
				m_srv->sendMsgToSrv("Start");
				printf("Started! \n");
				m_executed = true;
			}
			break;
		}



		// "AskRobotPos"
		case 10: {
			if(m_executed == false) {
				// ロボットの現在位置を取得し、ビューアーに送り返す
				Vector3d myPos;
				m_my->getPosition(myPos);
				double x = myPos.x();
				double z = myPos.z();
				double theta = 0;			// y方向の回転は無しと考える		
				char replyMsg[256];			
				sprintf(replyMsg, "AskObjPos %6.1lf %6.1lf %6.1lf", x, z, theta);
				printf("%s \n", replyMsg);
				m_srv->sendMsgToSrv(replyMsg);
				m_executed = true;				
			}
			break;
		}




		// 物体の方向が帰ってきた
		case 20: {
			// 送られた座標に回転する
			m_time = rotateTowardObj(nextPos, m_vel, evt.time());
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
			m_time = rotateTowardObj(nextPos, m_vel, evt.time());
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
					m_time = rotateTowardObj(grabPos, m_vel, evt.time());
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
			m_time = rotateTowardObj(nextPos, m_vel, evt.time());
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
					m_time = rotateTowardObj(throwPos, m_vel, evt.time());
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


		case 500: {
			if(m_executed == false) {
				printf("state 500 \n");
				// 自分の位置の取得
				Vector3d myPos;
				m_my->getPosition(myPos);
				double x = myPos.x();								// 単位はcm
				double z = myPos.z();
				double theta = 0;										// y方向の回転は無しと考える	

				// ゴミの場所分かるけど、行き方分からないため、教えてくれるかな
				char replyMsg[256];
				sprintf(replyMsg, "AskRoute %6.1lf %6.1lf %6.1lf %6.1lf %6.1lf %6.1lf", 
																			x, z, theta, m_tpos.x(), m_tpos.y(), m_tpos.z());				
				m_srv->sendMsgToSrv(replyMsg);

				printf("state500 replyMess %s \n", replyMsg);
				m_executed = true;				
			}
			break;
		}






		// 物体の方向が帰ってきた
		case 620: {
			if(m_nodeId < m_route.size()) {
				nextPos.set(m_route[m_nodeId].x, m_tpos.y(), m_route[m_nodeId].y);	
				// 送られた座標に回転する
				m_time = rotateTowardObj(nextPos, m_vel, evt.time());
				m_state = 621;

			} else if(m_nodeId == m_route.size()){
				printf("grab ---> state 730\n");
				m_state = 730;
				//m_executed = false;
			}
			
			//nextPos.set(x, y, z);
			// 送られた座標に回転する
			//m_time = rotateTowardObj(nextPos, m_vel, evt.time());
			//printf("debug %lf %lf \n", evt.time(), m_time);
			//m_state = 621;
			m_executed = false;
			break;
		}
		
		case 621: {
			// ロボットが回転中
			//printf("debug %lf %lf \n", evt.time(), m_time);
			if(evt.time() > m_time && m_executed == false) {
				// 物体のある場所に到着したので、車輪と止め、関節を回転し始め、物体を拾う
				m_my->setWheelVelocity(0.0, 0.0);
				// 物体のある方向に回転した
				//printf("目的地の近くに移動します %lf %lf %lf \n", nextPos.x(), nextPos.y(), nextPos.z());	
				// 送られた座標に移動する
				if(m_nodeId < m_route.size()) {
					//m_time = goToObj(nextPos, m_vel*4, m_range, evt.time());
					double range = 0;
					m_time = goToObj(nextPos, m_vel*4, range, evt.time());
				} 		

				//m_time = goToObj(nextPos, m_vel*4, 40, evt.time());
				m_state = 622;
				m_executed = false;
			}
			break;
		}

		case 622: {
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

				if(m_nodeId < m_route.size()) {
					printf("routing \n");
					m_nodeId++;
					m_state = 620;
				} 
				//break; 

				/*
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
				}*/

			}
			break;
		}



		case 730: {
			// 送られた座標に回転する
			m_time = rotateTowardObj(m_tpos, m_vel, evt.time());
			printf("case 730 time: %lf \n", m_time);
			m_state = 731;
			m_executed = false;
			break;
		}

		// 物体を掴むために、ロボットの向く角度をズラス
		case 731: {
			if(evt.time() > m_time && m_executed == false) {
				Vector3d grabPos;
				//printf("斜めにちょっとずれた以前の時間 time: %lf \n", evt.time());
				printf("case 731 %lf %lf %lf \n", m_tpos.x(), m_tpos.y(), m_tpos.z());
				if(calcGrabPos(m_tpos, 16.5, grabPos)) {
					m_time = rotateTowardObj(grabPos, m_vel, evt.time());
					printf("斜め grabPos :%lf %lf %lf \n", grabPos.x(), grabPos.y(), grabPos.z());
					printf("time: %lf \n", m_time);
				}
				//rotateTowardGrabPos(m_tpos, m_vel, evt.time());
				m_state = 732;
				m_executed = false;				
			}
			break;
		}

		// 物体の方向が帰ってきた
		case 732: {
			if(evt.time() > m_time && m_executed == false) {
				// 物体のある場所に到着したので、車輪と止め、関節を回転し始め、物体を拾う
				m_my->setWheelVelocity(0.0, 0.0);
				//printf("回転を止めた evt.time %lf \n", evt.time());
				// 関節の回転を始める
				// izen to gyakuhoukou ni kaiten saseru 
				m_my->setJointVelocity("RARM_JOINT4", m_jvel, 0.0);
				// 50°回転
				m_time = DEG2RAD(ROTATE_ANG) / m_jvel + evt.time();
				m_state = 734;
				m_executed = false;
			}
			break;
		}


		// 物体の方向が帰ってきた
		case 734: {
			if(evt.time() > m_time && m_executed == false) {
				// 物体のある場所に到着したので、車輪と止め、関節を回転し始め、物体を拾う
				m_my->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
				//printf("回転を止めた evt.time %lf \n", evt.time());
				// 関節の回転を始める
				// izen to gyakuhoukou ni kaiten saseru 
				m_my->setJointVelocity("RARM_JOINT4", -m_jvel, 0.0);
				// 50°回転
				m_time = DEG2RAD(ROTATE_ANG) / m_jvel + evt.time();
				m_state = 736;
				m_executed = false;
			}
			break;
		}

		case 736: {
			// 関節回転中
			if(evt.time() > m_time && m_executed == false) {
				// 関節の回転を止める
				m_my->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
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
					printf("erased ... state 733 \n");	

					// ゴミ箱への行き方と問い合わせする
					char replyMsg[256];

					// もっとも近いゴミ箱を探す
					//bool found = recognizeNearestTrashBox(m_trashBoxPos, m_trashBoxName);

					// ゴミを置くべき座標を探す
					bool found = findPlace2PutObj(m_trashBoxPos, m_tname); 
					if(found) {
						// ゴミ箱が検出出来た
						std::cout << "trashboxName ........^^^^^^ " << m_trashBoxName << std::endl;
						//sprintf(replyMsg, "AskTrashBoxRoute %6.1lf %6.1lf %6.1lf %6.1lf %6.1lf %6.1lf", 
							//										x, z, theta, m_trashBoxPos.x(), m_trashBoxPos.y(), m_trashBoxPos.z());



							// route 2 trashBox
							// 自分の位置の取得
							//Vector3d myPos;
							//m_my->getPosition(myPos);
							//double x = myPos.x();								// 単位はcm
							//double z = myPos.z();
							//double theta = 0;			


							//Vector3d placePos;
							//SimObj *place = getObj(m_trashBoxName.c_str());
							////printf("created SimObj Trash \n");
							//// ゴミの位置取得
							//place->getPosition(placePos);

							// gomibako coordinate ....
							//Obstacle obs(m_trashBoxPos.x(), -50, 40, 110);
							Obstacle obs(m_trashBoxPos.x(), m_trashBoxPos.z(), 20, 40);
							printf("trashBox %lf %lf \n", m_trashBoxPos.x(), m_trashBoxPos.z());
							printf("set place for gomibako \n");

							if(m_trashBoxName == "wagon_0") {
								printf("bring trash to wagon_0 \n");							
								obs.setPosition(m_trashBoxPos.x()-10, m_trashBoxPos.z()-8, 60, 40);
							}
							
							Node2D robotPos(x, z);
							Node2D objPos(m_trashBoxPos.x(), m_trashBoxPos.z());
							Node2D grabPos = getGrabPosition(objPos, obs);
							m_route.clear();
							m_route = calcRoute(robotPos, grabPos, obs);
							//m_roomObs.clear();
							//m_roomObs.push_back(obs);calc
							//m_roomObs.push_back(m_obstacleMap["table_0"]);
							//m_route = calcFullRoute(robotPos, grabPos, m_roomObs);
							printf("calc route ......733............. \n");
							for(int i=0; i < m_route.size(); i++) {
									printf("routeIdx %d %lf %lf \n", i, m_route[i].x, m_route[i].y);
							}

							m_state = 740;
							m_executed = false;
							m_nodeId = 1;


					} else {
						//sprintf(replyMsg, "AskTrashBoxPos %6.1lf %6.1lf %6.1lf", 
							//										x, z, theta);
						//m_state = 500;
						printf("debug case 733 send start \n");						
						m_srv->sendMsgToSrv("Start");			
						
						m_executed = true;

						printf("bbb \n\n\n\n\n\n aaa");
					}

					//m_srv->sendMsgToSrv(replyMsg);	
					//_executed = true;	
							
				} else {					// 物体を掴めなかった、次に探す場所を問い合わせる
					// ゴミを掴めなかったもしくはゴミが無かった、次にゴミのある場所を問い合わせする
					// 逆方向に関節の回転を始める
					// izen to gyakuhoukou ni kaiten saseru
					m_my->setJointVelocity("RARM_JOINT4", -m_jvel, 0.0);
					// 50°回転
					m_time = DEG2RAD(ROTATE_ANG) / m_jvel + evt.time();
					m_state = 738;				
					m_lastFailedTrash = m_tname;
					m_executed = false;
				}		
				
			}
			break;
		}
		
		case 738: {
			if(evt.time() > m_time && m_executed == false) {
				// 関節の回転を止める
				m_my->setJointVelocity("RARM_JOINT4", 0.0, 0.0);

				// 自分の位置の取得
				Vector3d myPos;
				m_my->getPosition(myPos);
				double x = myPos.x();
				double z = myPos.z();
				double theta = 0;			
				char replyMsg[256];
				//sprintf(replyMsg, "AskObjPos %6.1lf %6.1lf %6.1lf", x, z, theta);
				//printf("case 734 debug %s \n", replyMsg);

				//m_srv->sendMsgToSrv(replyMsg);			
				printf("case 734 send start \n");	
				m_srv->sendMsgToSrv("Start");			
	
				m_executed = true;
			}
			break;
		}



		case 740: {
			// 送られた座標に回転する


			if(m_nodeId < m_route.size()) {
				nextPos.set(m_route[m_nodeId].x, m_tpos.y(), m_route[m_nodeId].y);	
				// 送られた座標に回転する
				printf("go state 741 \n");
				m_time = rotateTowardObj(nextPos, m_vel, evt.time());
				m_state = 741;

			} else if(m_nodeId == m_route.size()){
				//throwtrash
				printf("throwTrash \n");

				//m_trashBoxPos.x(), m_trashBoxPos.z()
				m_time = rotateTowardObj(m_trashBoxPos, m_vel, evt.time());
				//m_time = rotateTowardGrabPos(m_trashBoxPos, m_vel, evt.time());

				m_state = 750;
				m_executed = false;
			}


			//m_time = rotateTowardObj(nextPos, m_vel, evt.time());
			//m_state = 741;
			m_executed = false;
			break;
	  }

		case 741: {
			// 送られた座標に回転中
			if(evt.time() > m_time && m_executed == false) {
				// 送られた座標に移動する
				//printf("目的地の近くに移動します %lf %lf %lf \n", nextPos.x(), nextPos.y(), nextPos.z());	
				double range = 0;
				m_time = goToObj(nextPos, m_vel*4, range, evt.time());
				m_state = 742;
				m_executed = false;
			}
			break;
	  }

		case 742: {
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
				/*bool found = recognizeNearestTrashBox(m_trashBoxPos, m_trashBoxName);
				if(found) {
					// ゴミ箱が検出出来た
					std::cout << "trashboxName " << m_trashBoxName << std::endl;
					sprintf(replyMsg, "AskTrashBoxRoute %6.1lf %6.1lf %6.1lf %6.1lf %6.1lf %6.1lf", 
																x, z, theta, m_trashBoxPos.x(), m_trashBoxPos.y(), m_trashBoxPos.z());
				} else {
					sprintf(replyMsg, "AskTrashBoxPos %6.1lf %6.1lf %6.1lf", 
																x, z, theta);
				}*/
				
				m_nodeId++;
				m_state = 740;	


				//m_srv->sendMsgToSrv(replyMsg);
				//m_executed = true;
				m_executed = false;
			}
			break;
		}



	case 750: {
			if(evt.time() > m_time && m_executed == false) {

				
				m_my->setJointVelocity("RARM_JOINT4", 0.0, 0.0);

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
				printf("trashboxPos %lf %lf \n",  m_trashBoxPos.x(), m_trashBoxPos.z());
				if(calcGrabPos(m_trashBoxPos, 20, throwPos)) 
				{
					m_time = rotateTowardObj(throwPos, m_vel, evt.time());
					printf("斜めに捨てる throwPos :%lf %lf %lf \n", throwPos.x(), throwPos.y(), throwPos.z());
					//printf("time: %lf \n", m_time);
				}
				m_state = 751;
				m_executed = false;				
			}
		
			break;
		}

		case 751: {
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
			std::cout << "suteta gomi " << m_tname << std::endl;
		  sleep(1);
			// grasp終了
		  m_grasp = false;

			//confirmThrewTrashPos(m_threwPos, m_tname);
			//printf("捨てた座標：　%lf %lf %lf \n", m_threwPos.x(), m_threwPos.y(), m_threwPos.z());	
			
			// gyakuhoukou ni kaiten saseru
		  m_my->setJointVelocity("RARM_JOINT4", -m_jvel, 0.0);
		  m_time = DEG2RAD(ROTATE_ANG) / m_jvel + evt.time() + 1.0;   
			m_state = 752;
			m_executed = false;
			break;
		}

		case 752: {
			// 関節が回転中
			if(evt.time() > m_time && m_executed == false) {
				// 関節が元に戻った、関節の回転を止める
				m_my->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
				// 自分の位置の取得
				Vector3d myPos;
				m_my->getPosition(myPos);
				double x = myPos.x();
				double z = myPos.z();
				double theta = 0;										// y方向の回転は無しと考える	
				
				// ゴミを捨てたので、次にゴミのある場所を問い合わせする
				char replyMsg[256];
							
				
				if(recognizeRandomTrash(m_tpos, m_tname)) {
				//if(recognizeNearestTrash(m_tpos, m_tname)) {
					m_executed = true;
					// 物体が発見された
					printf("case 752 debug \n");
					m_srv->sendMsgToSrv("Start");
					//m_executed = true;
										Vector3d myPos;
					m_my->getPosition(myPos);
					double x = myPos.x();								// 単位はcm
					double z = myPos.z();
					double theta = 0;			

					// table coordinate ....
					Obstacle obs = m_obstacleMap["table_0"];
					Node2D robotPos(x, z);
					Node2D objPos(m_tpos.x(), m_tpos.z());
					Node2D grabPos = getGrabPosition(objPos, obs);
					printf("@@@ robotPos %lf %lf objPos %lf %lf GrabPos %lf %lf @@@ \n",
									robotPos.x, robotPos.y, objPos.x, objPos.y, grabPos.x, grabPos.y);
					m_route.clear();
					m_route = calcRoute(robotPos, grabPos, obs);
					for(int i=0; i < m_route.size(); i++) {
							printf("routeIndex %d %lf %lf \n", i, m_route[i].x, m_route[i].y);
					}
					printf("route is calculated .... ////// ||||| \n");
			
					//state 500?600 でプログラムが変わる
					//m_state = 500;				
					//m_state = 620;
					//m_executed = false;		
					//printf("m_executed = false, state = 500 \n");		
					m_nodeId = 1;		
					m_state = 620;
	
				} else {
					//sprintf(replyMsg, "AskObjPos %6.1lf %6.1lf %6.1lf", x, z, theta);
					//m_srv->sendMsgToSrv(replyMsg);
					m_state = 10;
					m_executed = true;
					broadcastMsgToSrv("cannot find object anymore");
					printf("cannot find object anymore \n");
				}
				
			}
			break;
		}
		
		case 800: {
			if(evt.time() > m_time && m_executed == false) {
				m_my->setWheelVelocity(0.0, 0.0);
				// 関節の回転を止める
				//m_my->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
				// 自分の位置の取得
				Vector3d myPos;
				m_my->getPosition(myPos);
				double x = myPos.x();
				double z = myPos.z();
				double theta = calcHeadingAngle();		

				// カメラがついているリンク名取得
				//std::string link = "WAIST_LINK0"; //m_my->getCameraLinkName();
				std::string link = "WAIST_LINK0"; //m_my->getCameraLinkName(3);
				//const dReal *lpos = m_my->getParts(link.c_str())->getPosition();
				Vector3d lpos;
				m_my->getParts(link.c_str())->getPosition(lpos);
	      
				// カメラの位置取得(リンク座標系)
				Vector3d cpos;
				m_my->getCamPos(cpos, 3);
	      
				// カメラの位置(絶対座標系,ロボットの回転はないものとする)
				//Vector3d campos(lpos[0] + cpos.x(), lpos[1] + cpos.y(), lpos[2] + cpos.z());
			  Vector3d campos(lpos.x() + cpos.x(), lpos.y() + cpos.y(), lpos.z() + cpos.z());
				
				// カメラの方向取得(ロボットの回転,関節の回転はないものとする)
				Vector3d cdir;
				m_my->getCamDir(cdir, 3);

				char replyMsg[1024];
				sprintf(replyMsg, "AskRandomRoute %6.1lf %6.1lf %6.1lf %6.1lf %6.1lf %6.1lf %6.1lf %6.1lf %6.1lf", 
																					x, z, theta, campos.x(), campos.y(), campos.z(), cdir.x(), cdir.y(), cdir.z());


				//sprintf(replyMsg, "AskRandomRoute %6.1lf %6.1lf %6.1lf", x, z, theta);
				printf("%s \n", replyMsg);
				m_srv->sendMsgToSrv(replyMsg);
				m_executed = true;
				//m_state = 805;
				//printf("m_state 800 ...");
			}
			break;
		}

		case 805: {
			if(evt.time() > m_time && m_executed == false) {
				// 送られた座標に移動する
				//printf("目的地の近くに移動します %lf %lf %lf \n", nextPos.x(), nextPos.y(), nextPos.z());	
				//m_my->setWheelVelocity(0.0, 0.0);
				//printf("robot stopped 111\n");
				double range = 0;
				m_time = rotateTowardObj(nextPos, m_vel, evt.time());
				//m_time = goToObj(nextPos, m_vel*4, range, evt.time());
				m_state = 807;
				m_executed = false;
			}
			break;
	  }

		case 807: {
			if(evt.time() > m_time && m_executed == false) {
				// 物体のある場所に到着したので、車輪と止め、関節を回転し始め、物体を拾う
				//m_my->setWheelVelocity(0.0, 0.0);
				// 物体のある方向に回転した
				//printf("目的地の近くに移動します %lf %lf %lf \n", nextPos.x(), nextPos.y(), nextPos.z());	
				// 送られた座標に移動する
				//printf("onAction goToObj \n");
				m_time = goToObj(nextPos, m_vel*4, m_range, evt.time());
				//m_time = goToObj(nextPos, m_vel*4, 40, evt.time());
				m_state = 810;
				m_executed = false;
			}
			break;
		}

		case 810: {
			// 送られた座標に移動中
			if(evt.time() > m_time && m_executed == false) {
				m_my->setWheelVelocity(0.0, 0.0);
				//printf("robot stopped 222\n");
				// 送られた座標に到着した、 自分の位置の取得
				Vector3d myPos;
				m_my->getPosition(myPos);
				double x = myPos.x();
				double z = myPos.z();
				double theta = calcHeadingAngle();			// y方向の回転は無しと考える	


				// カメラがついているリンク名取得
				std::string link = "WAIST_LINK0"; //m_my->getCameraLinkName(3);
				//const dReal *lpos = m_my->getParts(link.c_str())->getPosition();
				Vector3d lpos;
				m_my->getParts(link.c_str())->getPosition(lpos);
	      
				// カメラの位置取得(リンク座標系)
				Vector3d cpos;
				m_my->getCamPos(cpos, 3);
	      
				// カメラの位置(絶対座標系,ロボットの回転はないものとする)
				//Vector3d campos(lpos[0] + cpos.x(), lpos[1] + cpos.y(), lpos[2] + cpos.z());
				Vector3d campos(lpos.x() + cpos.x(), lpos.y() + cpos.y(), lpos.z() + cpos.z());
				char msg[1024];
	      
				// カメラの方向取得(ロボットの回転,関節の回転はないものとする)
				Vector3d cdir;
				m_my->getCamDir(cdir, 3);



				char replyMsg[1024];
				sprintf(replyMsg, "AskRandomRoute %6.1lf %6.1lf %6.1lf %6.1lf %6.1lf %6.1lf %6.1lf %6.1lf %6.1lf", 
																					x, z, theta, campos.x(), campos.y(), campos.z(), cdir.x(), cdir.y(), cdir.z());
				printf("%s \n", replyMsg);
				m_srv->sendMsgToSrv(replyMsg);


				m_executed = true;
				//printf("m_state 810");
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
  
  // 送信者がゴミ認識サービスの場合
  if(sender == "RecogTrash"){


		if(FIND_OBJ_BY_ID_MODE == true)
		if(m_isOstacleCaculated == false) {
			std::vector<std::string> m_obstacleName;
			m_obstacleName.clear();
			m_obstacleName.push_back("table_0");
			m_obstacleName.push_back("table_1");
			m_obstacleName.push_back("table_2");
			m_obstacleName.push_back("wagon_0");
			m_obstacleName.push_back("trashbox_0");
			m_obstacleName.push_back("trashbox_0");
			m_obstacleName.push_back("trashbox_1");
			m_obstacleName.push_back("trashbox_2");

			m_table_0.setPosition(0., 0., 105., 60.); // scale 1.5
			m_table_1.setPosition(100., -100., 130., 60.);
			m_table_2.setPosition(-50., -90., 70., 40.);
			m_wagon_0.setPosition(100., 100., 60., 65.);
			m_trashbox_0.setPosition(-150., 0., 40., 20.);
			m_trashbox_1.setPosition(-150., -50., 40., 20.);
			m_trashbox_2.setPosition(-150., -100., 40., 20.);

			m_roomObs.clear();
			m_roomObs.push_back(m_table_0);
			m_roomObs.push_back(m_table_1);
			m_roomObs.push_back(m_table_2);
			m_roomObs.push_back(m_wagon_0);
			m_roomObs.push_back(m_trashbox_0);
			m_roomObs.push_back(m_trashbox_1);
			m_roomObs.push_back(m_trashbox_2);

			Vector3d obsPos;
		
			// 現位置に検出出来るゴミを検索する
			for(int obsNum = 0; obsNum < m_obstacleName.size(); obsNum++) {
				std::cout << "obstacle name:" << m_obstacleName[obsNum] << std::endl;

				std::string obsName = m_obstacleName[obsNum];
				if(getObj(obsName.c_str())) {
					SimObj *obstacleObj = getObj(obsName.c_str());
					printf("created SimObj 4 obstacle \n");

					// ゴミの位置取得
					obstacleObj->getPosition(obsPos);
					double width = m_roomObs[obsNum].width;
					double height = m_roomObs[obsNum].height;
					m_roomObs[obsNum].setPosition(obsPos.x(), obsPos.z(), width, height);

					/*if(m_trashBoxName == "wagon_0") {
						printf("bring trash to wagon_0 \n");							
						m_roomObs[obsNum].setPosition(obsPos.x()-5, obsPos.z()-8, 60.0, 40.0);
					}*/
				}
				m_obstacleMap.insert(std::pair<std::string, Obstacle>(obsName, m_roomObs[obsNum]));
				printf("caculating obstacle \n");
			}


			/*m_obstacleMap.insert(std::pair<std::string, Obstacle>("table_0", m_table_0));
			m_obstacleMap.insert(std::pair<std::string, Obstacle>("table_1", m_table_1));
			m_obstacleMap.insert(std::pair<std::string, Obstacle>("table_2", m_table_2));
			m_obstacleMap.insert(std::pair<std::string, Obstacle>("wagon_0", m_wagon_0));
			m_obstacleMap.insert(std::pair<std::string, Obstacle>("trashbox_0", m_trashbox_0));
			m_obstacleMap.insert(std::pair<std::string, Obstacle>("trashbox_1", m_trashbox_1));
			m_obstacleMap.insert(std::pair<std::string, Obstacle>("trashbox_2", m_trashbox_2));*/
			
			m_isOstacleCaculated = true;
		}



	  char *all_msg = (char*)evt.getMsg();		
		//printf("all_msg: %s \n", all_msg);
		char *delim = (char *)(" ");
		char *ctx;
		char *header = strtok_r(all_msg, delim, &ctx);

		double x = 0, y = 0, z = 0; // range = 0;
		//sscanf(all_msg, "%s %lf %lf %lf", header, &x, &y, &z);

		// "AskRobotPos" ロボットの現在位置の問い合わせが来た
		if(strcmp(header, "AskRobotPos") == 0) {
			// ロボットのステートを更新
			m_state = 10;
			m_executed = false;
			return;
		}
		
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

		if(strcmp(header, "FindObjPlease") == 0) {
			printf("FindObjPlease \n");
			//bool found = recognizeNearestTrash(m_tpos, m_tname);
			bool found = recognizeRandomTrash(m_tpos, m_tname);

			// ロボットのステートを更新
			
			if (found == true) {
				broadcastMsgToSrv("I found trash");
				// 自分の位置の取得
				Vector3d myPos;
				m_my->getPosition(myPos);
				double x = myPos.x();								// 単位はcm
				double z = myPos.z();
				double theta = 0;			

				// table coordinate ....
				Obstacle obs = m_obstacleMap["table_0"];
				Node2D robotPos(x, z);
				Node2D objPos(m_tpos.x(), m_tpos.z());
				Node2D grabPos = getGrabPosition(objPos, obs);
				printf("@@@ robotPos %lf %lf objPos %lf %lf GrabPos %lf %lf @@@ \n",
								robotPos.x, robotPos.y, objPos.x, objPos.y, grabPos.x, grabPos.y);
				m_route.clear();
				//m_route = calcFullRoute(robotPos, grabPos, m_roomObs);
				m_route = calcRoute(robotPos, grabPos, obs);
				for(int i=0; i < m_route.size(); i++) {
						printf("routeIndex %d %lf %lf \n", i, m_route[i].x, m_route[i].y);
				}
				printf("route is calculated .... ////// ||||| \n");
			
				//state 500?600 でプログラムが変わる
				//m_state = 500;				
				m_state = 620;
				m_executed = false;		
				//printf("m_executed = false, state = 500 \n");		
				m_nodeId = 1;		

				return;
			} else {
				broadcastMsgToSrv("I cannot find trash");
				m_state = 10;
				m_executed = false;
				return;
			}
			
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
			y = atof(strtok_r(NULL, delim, &ctx));
			z = atof(strtok_r(NULL, delim, &ctx));
			m_range = atof(strtok_r(NULL, delim, &ctx));
			//printf("onRecv RandomRoute %lf %lf %lf %lf \n", x, y, z, m_range);
			nextPos.set(x, y, z);
			m_state = 805;
			m_executed = false;
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
	bool debug = true;
  // 自分の回転を得る
  Rotation myRot;
  m_my->getRotation(myRot);

  // エンティティの初期方向
  Vector3d iniVec(0.0, 0.0, 1.0);
      
  // y軸の回転角度を得る(x,z方向の回転は無いと仮定)
  double qw = myRot.qw();
  double qy = myRot.qy();
      
  double theta = 2*acos(fabs(qw));
	if(debug) printf("heading angle: %lf\n", theta * 180.0 / PI);

  if(qw*qy < 0) {   
	 theta = -1*theta;
	 if(debug) printf("qw: %lf qy: %lf \n", qw, qy);
	}

	if(debug) printf("heading angle: %lf\n", theta * 180.0 / PI);

	return theta * 180.0 / PI;
}

  
double MyController::rotateTowardObj(Vector3d pos, double velocity, double now)
{
	//printf("start rotate %lf \n", now);
	//自分を取得  
	SimObj *my = getObj(myname());  
	
	//printf("向く座標 %lf %lf %lf \n", pos.x(), pos.y(), pos.z());
  // 自分の位置の取得
  Vector3d myPos;
  m_my->getPosition(myPos);

	//自分の手のパーツを得ます  
	CParts * parts = my->getParts("RARM_LINK2");  

	Vector3d partPos;
	parts->getPosition(partPos);


  // 自分の位置からターゲットを結ぶベクトル
  Vector3d tmpPos = pos;
	//Vector3d tmpp = myPos;
  tmpPos -= myPos;
	//tmpp -= partPos;
  
	// y方向は考えない
  tmpPos.y(0);
	// 近すぎるなら，回転なし
	double dis = tmpPos.x() * tmpPos.x() + tmpPos.y() * tmpPos.y() + tmpPos.z() * tmpPos.z();
	if(dis < 1.0) {
		return 0.0;
	}	
  
  // 自分の回転を得る
  Rotation myRot;
  m_my->getRotation(myRot);

  // エンティティの初期方向
  Vector3d iniVec(0.0, 0.0, 1.0);
      
  // y軸の回転角度を得る(x,z方向の回転は無いと仮定)
  double qw = myRot.qw();
  double qy = myRot.qy();
      
  double theta = 2 * acos(fabs(qw));
	printf("theta: %lf \n",  theta * 180 / PI);
  if(qw * qy < 0) {
    theta = -1 * theta;
		printf("qw: %lf qy: %lf theta: %lf \n", qw, qy, theta * 180 / PI);
	}

  // z方向からの角度
  double tmp = tmpPos.angle(Vector3d(0.0, 0.0, 1.0));  
	double targetAngle = acos(tmp);
	printf("targetAngle: %lf deg %lf \n", targetAngle*180.0/PI, theta * 180 / PI);

  // 方向
	//printf("tmpp.x() %lf \n", tmpp.x());
  if(tmpPos.x() > 0) {
		targetAngle = -1 * targetAngle;
		//printf("targetAngle: %lf deg \n", targetAngle*180.0/PI);
	}

  targetAngle += theta;
	printf("targetAngle: %lf deg %lf \n", targetAngle*180.0/PI, theta * 180 / PI);

	//printf("qw: %lf qy: %lf theta: %lf tmp: %lf targetAngle: %lf \n", qw, qy, theta, tmp, targetAngle);
	
  if(targetAngle == 0.0){
		//printf("donot need rotate \n");
    return 0.0;
  }
  else 
	{
    // 回転すべき円周距離
    double distance = m_distance * PI * fabs(targetAngle) / (2 * PI);

    // 車輪の半径から移動速度を得る
    double vel = m_radius * velocity;
    
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
		//printf("rotate time: %lf, time to stop: %lf \n", time, now + time);
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




//          ªy
//   0  |    7   |   6   
//--------------------------
//      |xxxxxxxx| 
//x<- 1 |xxxxxxxx|   5
//      |xxxxxxxx|
//--------------------------    
//   2  |    3   |   4      
//
int MyController::getPointPositionIndex(Node2D pos, Obstacle obs) {
	int posIdx = 3;		//禁止領域の中に入った状態

	if(pos.x > obs.x_max) {
		if(pos.y > obs.y_max) {
			posIdx = 0;
		} else if(pos.y <= obs.y_max && pos.y > obs.y_min) {
			posIdx = 1;	
		} else if(pos.y <= obs.y_min) {
			posIdx = 2;
		}
	} else if(pos.x <= obs.x_max && pos.x > obs.x_min) {
		if(pos.y >= obs.y_max) {
			posIdx = 7;
		} else if(pos.y <= obs.y_min) {
			posIdx = 3;
		}
	} else if(pos.x <= obs.x_min) {
		if(pos.y > obs.y_max) {
			posIdx = 6;
		} else if(pos.y <= obs.y_max && pos.y > obs.y_min) {
			posIdx = 5;
		} else if(pos.y <= obs.y_min) {
			posIdx = 4;
		}
	}

	//printf("posIdx: %d \n", posIdx);
	return posIdx;
}




//          ªy
//   0  |    7   |   6   
//--------------------------
//      |        | 
//x<- 1 |        |   5
//      |  o     |
//--------------------------    
//   2  |    3   |   4      
//        robot


int MyController::getGrabPositionIndex(Node2D objPos, Obstacle obs) {
	int grabPosIdx = 3;		//意図的に何の状態も入っていないとき、3を返す

	printf("object %lf %lf obstacle %lf %lf %lf %lf \n", objPos.x, objPos.y, obs.x_min, obs.y_min, obs.x_max, obs.y_max);

	double dis7 = obs.y_max - objPos.y;
	double dis1 = obs.x_max - objPos.x;
	double dis3 = objPos.y - obs.y_min;
	double dis5 = objPos.x - obs.x_min;

	double min = obs.width + obs.height;

	if(dis7 < min) {
		min = dis7;
		grabPosIdx = 7;
		printf("dis7 min %lf \n", dis7);
	}
	if(dis1 < min) {
		min = dis1;
		grabPosIdx = 1;
		printf("dis1 min %lf \n", dis1);
	}
	if(dis3 < min) {
		min = dis3;
		grabPosIdx = 3;
		printf("dis3 min %lf \n", dis3);
	}
	if(dis5 < min) {
		min = dis5;
		grabPosIdx = 5;
		printf("dis5 min %lf \n", dis5);
	}

	return grabPosIdx;
}



Node2D MyController::getGrabPosition(Node2D objPos, Obstacle obs) {
	Node2D grabPos(objPos.x, objPos.y - ARM_RADIUS);	

	int grabPosIdx = getGrabPositionIndex(objPos, obs);

	if(grabPosIdx == 7) {
		grabPos.setPosition(objPos.x, objPos.y + ARM_RADIUS);
	}

	if(grabPosIdx == 1) {
		grabPos.setPosition(objPos.x + ARM_RADIUS, objPos.y);
	}

	if(grabPosIdx == 3) {
		grabPos.setPosition(objPos.x, objPos.y - ARM_RADIUS);
	}

	if(grabPosIdx == 5) {
		grabPos.setPosition(objPos.x - ARM_RADIUS, objPos.y);
	}

	return grabPos;
}


std::vector<Node2D> MyController::calcRoute(Node2D startPos, Node2D goalPos, Obstacle obs) {

		printf("calcRoute start \n");
	
    std::vector<Node2D> route;
    route.clear();
    route.push_back(startPos);
    route.push_back(goalPos);

    int startIdx = getPointPositionIndex(startPos, obs);
    int goalIdx = getPointPositionIndex(goalPos, obs);
    int step = 0;
    printf("start %lf %lf -> goal %lf %lf \n", startPos.x, startPos.y, goalPos.x, goalPos.y);
    printf("posId %d -> %d \n", startIdx, goalIdx);

    if(startIdx == goalIdx) {
        // go straight between 2 points
        return route;
    } else if(startIdx < goalIdx) {
        step = 1;
    } else if (startIdx > goalIdx) {
        step = -1;
    }

    Node2D supplyNode(goalPos.x, goalPos.y);

    while(startIdx != goalIdx) {
        //printf("startIdx: %d goadIdx: %d \n", startIdx, goalIdx);
        startIdx += step;
        int i = startIdx;
        if(i == 0) {
            supplyNode.setPosition(obs.x + obs.width / 2 + TRUCK_RADIUS, obs.y + obs.height / 2 + TRUCK_RADIUS);
            if(goalPos.x >= supplyNode.x && goalPos.y >= supplyNode.y) {
                //startIdx += step;
                if(startIdx == goalIdx)
                {
                    break;
                }
            } else {
                route.insert(route.begin() + route.size() -1, supplyNode);
                printf("added node0 \n");
            }
        } else if(i == 2) {
            supplyNode.setPosition(obs.x + obs.width / 2 + TRUCK_RADIUS, obs.y - obs.height / 2 - TRUCK_RADIUS);
            if(goalPos.x >= supplyNode.x && goalPos.y <= supplyNode.y) {
                //startIdx += step;
                if(startIdx == goalIdx)
                {
                    break;
                }
            } else {
                route.insert(route.begin() + route.size() -1, supplyNode);
                printf("added node2 \n");
            }
        } else if(i == 4) {
            supplyNode.setPosition(obs.x - obs.width / 2 - TRUCK_RADIUS, obs.y - obs.height / 2 - TRUCK_RADIUS);
            if(goalPos.x <= supplyNode.x && goalPos.y <= supplyNode.y) {
                //startIdx += step;
                if(startIdx == goalIdx)
                {
                    break;
                }
            } else {
                route.insert(route.begin() + route.size() -1, supplyNode);
                printf("added node4 \n");
            }
        } else if(i == 6) {
            printf("goal %lf %lf -> supplyNode %lf %lf \n", startPos.x, startPos.y, supplyNode.x, supplyNode.y);
            supplyNode.setPosition(obs.x - obs.width / 2 - TRUCK_RADIUS, obs.y + obs.height / 2 + TRUCK_RADIUS);
            if(goalPos.x <= supplyNode.x && goalPos.y >= supplyNode.y) {
                //startIdx += step;
                if(startIdx == goalIdx)
                {
                    break;
                }
            } else {
                route.insert(route.begin() + route.size() -1, supplyNode);
                printf("added node6 \n");
            }
        }
    }

    /*for(int i=0; i<route.size(); i++) {
        printf("\ncalculate route %lf %lf \n", route[i].x, route[i].y);
    }*/

    return route;
}






std::vector<Node2D> MyController::calcFullRoute(Node2D startPos, Node2D goalPos, std::vector<Obstacle> roomObs) {
	std::vector<Node2D> route;
	route.clear();
	if(roomObs.size() > 0) {
		route = calcRoute(startPos, goalPos, roomObs[0]);
	}

	for(int i = 0; i < route.size(); i++) {
		printf("route %lf %lf", route[i].x, route[i].y);
	}

	int startIdx;
	int goalIdx;


	//for(int i = 1; i < roomObs.size(); i++) {
	for(int i = 1; i < 2; i++) {
		for(int j = 0; j < route.size() - 1; j++) {			
			startIdx = getPointPositionIndex(route[j], roomObs[i]);
			goalIdx = getPointPositionIndex(route[j+1], roomObs[i]);
			std::vector<Node2D> subRoute = calcRoute(route[j], route[j+1], roomObs[i]);
				
			for(int k=0; k<subRoute.size(); k++) {
				printf("sub route %lf %lf", subRoute[k].x, subRoute[k].y);
			}

			if(subRoute.size() > 2) {
				for(int k=1; k<subRoute.size()-1; k++) {
					printf("added route %lf %lf for (%lf %lf) -> (%lf %lf) because of obstacle %d center(%lf %lf) size(%lf %lf) (x_min %lf y_min %lf x_max %lf y_max %lf)\n", 
						subRoute[k].x, subRoute[k].y, route[j].x, route[j].y, route[j+1].x, route[j+1].y, 
						i, roomObs[i].x, roomObs[i].y, roomObs[i].width, roomObs[i].height, roomObs[i].x_min, roomObs[i].y_min, roomObs[i].x_max, roomObs[i].y_max);
					route.insert(route.begin() + j + 1, subRoute[k]);
					j++;
				}
			}
		
			if(subRoute.size() > 2) {
				i = roomObs.size();
				printf("break here \n");
			}
			//for(int i=0; i<route.size(); i++) {
				//printf("refresh route %lf %lf \n", route[i].x, route[i].y);
			//}
		}
	}

	for(int i=0; i<route.size(); i++) {
		printf("\ncalculate route %lf %lf \n", route[i].x, route[i].y);
	}

	if(route.size() > 10) {
		route = calcRoute(startPos, goalPos, roomObs[0]);
	}

	for(int i=0; i<route.size(); i++) {
		printf("final... route %lf %lf \n", route[i].x, route[i].y);
	}

	return route;
}










extern "C" Controller * createController() {  
  return new MyController;  
}  


