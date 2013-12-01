#include "ControllerEvent.h"  
#include "Controller.h"
#include "Logger.h"  
#include <algorithm>
#include "PositionManager.h"
#include "Entity.h"
#include "Parameter.h"

#define PI 3.1415926535

//角度からラジアンに変換します
#define DEG2RAD(DEG) ( (PI) * (DEG) / 180.0 )

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
	bool recognizeTrash(Vector3d &pos, std::string &name);
	void GetRobotPositionInfo();
	bool GetEntityInfo(Vector3d &pos, std::vector<std::string> v_entities);
	void UpdatePosition(Entity entity);

	/* @brief  位置を指定しその方向に回転を開始し、回転終了時間を返します
	* @param  pos 回転したい方向の位置
	* @param  vel 回転速度
	* @param  now 現在時間
	* @return 回転終了時間
	*/
	double rotateTowardObj(Vector3d pos, double vel, double now);


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

	// ゴミの場所
	Vector3d m_tpos;  

	// ゴミの名前
	std::string m_tname;  

	/* ロボットの状態
	* 0 初期状態
	* 1 ゴミがある方向に回転している状態
	* 2 関節を曲げてゴミを取りに行っている状態
	* 3 ゴミを持ってゴミ箱の方向に回転している状態
	* 4 ゴミを持ってゴミ箱に向かっている状態
	* 5 ゴミを捨てて関節角度を元に戻している状態
	* 6 元に場所に戻る方向に回転している状態
	* 7 元の場所に向かっている状態
	* 8 元の向きに回転している状態
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

	// ゴミ候補オブジェクト
	std::vector<std::string> m_trashes;

	// 障害物候補の格納
	std::vector<std::string> m_obstacles;

	// ロボットの候補
	std::vector<std::string> m_robots;

	// エンティティの名前
	std::string m_entiyName;

	// 移動終了時間
	double m_time;

	// 初期位置
	Vector3d m_inipos;

	// ゴミ認識サービス
	BaseService *m_srv;

	// 動作を実行したかどうかのフラグ、動作を複数回実行を防ぐ
	bool m_executed;

	// 物体、障害物、ロボットの情報の送った数
	int m_numberOfSendedEntityFromCandidateList;
};  

void MyController::onInit(InitEvent &evt) 
{  
	m_my = getRobotObj(myname());

	// 初期位置取得
	m_my->getPosition(m_inipos);

	// 車輪の半径と車輪間隔距離
	m_radius = 10.0;
	m_distance = 10.0;

	m_time = 0.0;

	// 車輪の半径と車輪間距離設定
	m_my->setWheel(m_radius, m_distance);
	m_state = 0;

	// ここではゴミの名前が分かっているとします
	m_trashes.push_back("can_0");
	m_trashes.push_back("can_1"); 
	m_trashes.push_back("petbottle_0");
    
	m_obstacles.push_back("table_0");
	m_obstacles.push_back("trashbox_0");
	m_obstacles.push_back("trashbox_1");
	m_obstacles.push_back("trashbox_2");

	m_robots.push_back("robot_000");

	m_numberOfSendedEntityFromCandidateList = 0;

	srand((unsigned)time( NULL ));

	// 車輪の回転速度
	m_vel = 0.3;

	// 関節の回転速度
	m_jvel = 0.6;
}  
  
double MyController::onAction(ActionEvent &evt)
{
  switch(m_state){
		// 初期状態
		case 0: {
			if(m_srv == NULL){
				// ゴミ認識サービスが利用可能か調べる
				if(checkService(LAYOUT_MANAGE_SERVICE_NAME)){
					// ゴミ認識サービスに接続
					m_srv = connectToService(LAYOUT_MANAGE_SERVICE_NAME);
				} else {
					printf("cannot connect to service \n");
				}
			} else if(m_srv != NULL && m_executed == false) {  
				m_time = evt.time();
				m_srv->sendMsgToSrv(START_ASK_POS_MSG);
				m_state = 5;
			}
			break;
		}
  }
  return 0.1;      
}  
  
void MyController::onRecvMsg(RecvMsgEvent &evt)
{
	// 送信者取得
	std::string sender = evt.getSender();
	std::cout << "sender: " << sender << std::endl;

	char *all_msg = (char*)evt.getMsg();		
	printf("all_msg: %s \n", all_msg);
	char *delim = (char *)(" ");
	char *ctx;
	char *header = strtok_r(all_msg, delim, &ctx);
	printf("header: %s \n", header);	

	Vector3d pos;
	char *replyMsg = new char[1024];

	if (strcmp(header, ASK_OBJECT_POS_MSG) == 0) {
		printf("Received %s \n", ASK_OBJECT_POS_MSG);
		if (GetEntityInfo(pos, m_trashes)) {
			sprintf(replyMsg, "%s %d %s %6.1lf %6.1lf %6.1lf", ANS_OBJECT_POS_MSG, m_numberOfSendedEntityFromCandidateList - 1, m_entiyName.c_str(), pos.x(), pos.y(), pos.z());
			printf("%s \n", replyMsg);
			m_srv->sendMsgToSrv(replyMsg);
		} else {
			m_srv->sendMsgToSrv(FIN_ASK_OBJECT_POS_MSG);
			printf("%s \n", FIN_ASK_OBJECT_POS_MSG);
		}
		return;
	}

	if (strcmp(header, ASK_OBSTACLE_POS_MSG) == 0) {
		printf("Received %s \n", ASK_OBSTACLE_POS_MSG);
		if (GetEntityInfo(pos, m_obstacles)) {
			sprintf(replyMsg, "%s %d %s %6.1lf %6.1lf %6.1lf", ANS_OBSTACLE_POS_MSG, m_numberOfSendedEntityFromCandidateList - 1, m_entiyName.c_str(), pos.x(), pos.y(), pos.z());
			printf("%s \n", replyMsg);
			m_srv->sendMsgToSrv(replyMsg);
		} else {
			m_srv->sendMsgToSrv(FIN_ASK_OBSTACLE_POS_MSG);
			printf("%s \n", FIN_ASK_OBSTACLE_POS_MSG);
		}
		return;
	}

	if (strcmp(header, ASK_ROBOT_POS_MSG) == 0) {
 		printf("Received %s \n", ASK_ROBOT_POS_MSG);
		if (GetEntityInfo(pos, m_robots)) {
			sprintf(replyMsg, "%s %d %s %6.1lf %6.1lf %6.1lf", ANS_ROBOT_POS_MSG, m_numberOfSendedEntityFromCandidateList - 1, m_entiyName.c_str(), 0.0, 0.0, 0.0);
			printf("%s \n", replyMsg);
			m_srv->sendMsgToSrv(replyMsg);
		}
		return;
	}

	


	if (strcmp(header, START_SET_POS_MSG) == 0) {			
		m_srv->sendMsgToSrv(REQ_ENTITY_POS_MSG);	
		return;
	}

	printf("aaa\n");
	printf("%s _____\n", header);
	printf("%s \n", SET_ENTITY_POS_MSG);
	if (strcmp(header, SET_ENTITY_POS_MSG) == 0) {
		// メッセージを解析して、エンティティの位置をセットする
		printf("111 \n");
		
		printf("222 \n");
		printf("メッセージの未解析部分 :%s \n", ctx);
		
		Entity entity ;
		printf("333 \n");
		printf("ctx: %s \n", ctx);
					
		entity.PrintToConsole();
		//entity.GetEntityInfo(ctx);

		printf("333\n");
		// 移動させる
		printf("444 \n");	
		//UpdatePosition(entity);
		printf("555 \n");
		m_srv->sendMsgToSrv(REQ_ENTITY_POS_MSG);
		printf("666\n");		
		return;
	}

	if (strcmp(header, FIN_SET_POS_MSG) == 0) {			
		m_srv->sendMsgToSrv(FIN_SET_POS_MSG);
		return;
	}


	//if (strcmp(header, START_SET_POSITION_MSG) == 0		||
	//		strcmp(header, FIN_SET_OBJECT_POS_MSG) == 0	||
	//		strcmp(header, FIN_SET_OBSTACLE_POS_MSG) == 0) {			
	//		m_srv->sendMsgToSrv(header);	
	//	return;
	//}

	//if (strcmp(header, SET_OBJECT_POS_MSG) == 0) {
	//	//REQUEST_OBJECT_POS_MSG
	//	return;
	//}

	//if (strcmp(header, SET_OBSTACLE_POS_MSG) == 0) {
	//	//REQUEST_OBSTACLE_POS_MSG
	//	return;
	//}

	//if (strcmp(header, SET_ROBOT_POS_MSG) == 0) {
	//	//REQUEST_ROBOT_POS_MSG
	//	return;
	//}

	//if (strcmp(header, FIN_SET_ROBOT_POS_MSG) == 0) {
	//	m_srv->sendMsgToSrv(START_SET_POSITION_MSG);
	//	return;
	//}

	return;

}  


void MyController::UpdatePosition(Entity entity) {
	printf("ccc \n");	
	SimObj *simObj = getObj(entity.name.c_str());
	printf("ddd\n");
	Vector3d pos(entity.x, entity.y, entity.z);
	printf("eee\n");
	simObj->setPosition(pos);
	printf("ggg\n");	
	return;
}

void MyController::onCollision(CollisionEvent &evt) 
{
}
  

bool MyController::recognizeTrash(Vector3d &pos, std::string &name)
{
	/////////////////////////////////////////////
	///////////ここでゴミを認識します////////////
	/////////////////////////////////////////////

	// 候補のゴミが無い場合
	if(m_trashes.empty()){
	return false;
	}

	// ここでは乱数を使ってゴミを決定します
	int trashNum = rand() % m_trashes.size();

	// ゴミの名前と位置を取得します
	name = m_trashes[trashNum];
	SimObj *trash = getObj(name.c_str());

	// ゴミの位置取得
	trash->getPosition(pos);
	return true;
}



bool MyController::GetEntityInfo(Vector3d &pos, std::vector<std::string> v_entities)
{
	// 候補のゴミが無い場合
	if (m_numberOfSendedEntityFromCandidateList == v_entities.size()){
		m_numberOfSendedEntityFromCandidateList = 0;
	return false;
	}

	printf("候補のサイズ :%d \n", v_entities.size());

	// エンティティの名前を取得します
	m_entiyName = v_entities[m_numberOfSendedEntityFromCandidateList];

	// エンティティの生成
	SimObj *obj = getObj(m_entiyName.c_str());

	// エンティティの位置取得
	obj->getPosition(pos);

	printf("エンティティの座標 :%lf %lf %lf \n", pos.x(), pos.y(), pos.z());

	// 候補リストから送ったエンティティの数	
	m_numberOfSendedEntityFromCandidateList++;

	return true;
}


void MyController::UpdatePosition(Entity entity) {
	printf("ccc \n");	
	SimObj *simObj = getObj(entity.name);
	printf("ddd\n");
	Vector3d pos(entity.x, entity.y, entity.z);
	printf("eee\n");
	simObj->setPosition(pos);
	printf("ggg\n");	
	return;
}



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

	// 自分の回転を得る
	Rotation myRot;
	m_my->getRotation(myRot);

	// エンティティの初期方向
	Vector3d iniVec(0.0, 0.0, 1.0);
	  
	// y軸の回転角度を得る(x,z方向の回転は無いと仮定)
	double qw = myRot.qw();
	double qy = myRot.qy();
	  
	double theta = 2*acos(fabs(qw));

	if(qw*qy < 0)
		theta = -1*theta;

	// z方向からの角度
	double tmp = tmpp.angle(Vector3d(0.0, 0.0, 1.0));
	double targetAngle = acos(tmp);

	// 方向
	if(tmpp.x() > 0) targetAngle = -1*targetAngle;
	targetAngle += theta;

	if(targetAngle == 0.0){
		return 0.0;
	}
	else {
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
		else{
		  m_my->setWheelVelocity(-velocity, velocity);
		}

		return now + time;
	}
}

// object まで移動
double MyController::goToObj(Vector3d pos, double velocity, double range, double now)
{
	// 自分の位置の取得
	Vector3d myPos;
	m_my->getPosition(myPos);

	// 自分の位置からターゲットを結ぶベクトル
	pos -= myPos;

	// y方向は考えない
	pos.y(0);

	// 距離計算
	double distance = pos.length() - range;

	// 車輪の半径から移動速度を得る
	double vel = m_radius*velocity;

	// 移動開始
	m_my->setWheelVelocity(velocity, velocity);

	// 到着時間取得
	double time = distance / vel;

	return now + time;
}

extern "C" Controller * createController() {  
	return new MyController;  
}  

