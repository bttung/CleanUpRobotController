#include "ControllerEvent.h"  
#include "Controller.h"
#include "Logger.h"  
#include <string>
#include <algorithm>
#include "Parameter.h"

using namespace std;

class Messenge 
{
public:
	string GetString(string header, string content);
};

string Messenge::GetString(string header, string content) {
	printf("header: %s content: %s \n", header.c_str(), content.c_str());	
	string msg = header + " " + content;
	printf("msg: %s \n", msg.c_str());
	return msg;
}

class Entity
{
public:
	string type;
	int id;
	string name;
	double x;
	double y;
	double z;

public:
	Entity() {};
	Entity(string type, int id, string name);
	~Entity() {};
public:
	string ToString();
	void PrintToConsole();
	void GetEntityInfo(char* msg);
	void SetPosition(double _x, double _y, double _z);
};

Entity::Entity(string _type, int _id, string _name) {
	type = _type;
	id	 = _id;
	name = _name;
}

string Entity::ToString() {
	char content[256];
	sprintf(content, "%s %d %s %lf %lf %lf \n", type.c_str(), id, name.c_str(), x, y, z);
	printf("entity ToString: %s \n", content);	
	return string(content);
}

void Entity::PrintToConsole() {
	printf("entity: %s %d %s %lf %lf %lf \n", type.c_str(), id, name.c_str(), x, y, z);
	return;
}

void Entity::GetEntityInfo(char* str) {
	char entityName[256];
	char entityType[256];
	sscanf(str, "%s %d %s %lf %lf %lf", entityType, &id, entityName, &x, &y, &z);
	name = string(entityName);
	type = string(entityType);
	return;
}

void Entity::SetPosition(double _x, double _y, double _z) {
	x = _x; y = _y; z = _z;
	return;
}




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
	void GetEntityPositionInfo(vector<Entity> &v_entities);
	bool GetEntityInfo(Vector3d &pos, std::vector<std::string> v_entities);
	void UpdatePosition(Entity entity);
	void InitEntityInfo();

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
	std::vector<std::string> m_objects;

	// 障害物候補の格納
	std::vector<std::string> m_obstacles;

	// ロボットの候補
	std::vector<std::string> m_robots;

	vector<Entity> m_entities;

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
	int m_sendedEntityNum;
	Messenge m_messenge;

};  


void MyController::InitEntityInfo() {
	m_objects.clear();
	m_obstacles.clear();
	m_robots.clear();
	m_entities.clear();

	m_objects.push_back("can_0");
	m_objects.push_back("can_1");
	m_objects.push_back("petbottle_0");   
	m_obstacles.push_back("table_0");
	m_obstacles.push_back("trashbox_0");
	m_obstacles.push_back("trashbox_1");
	m_obstacles.push_back("trashbox_2");
	m_robots.push_back("robot_000");

	for (int i = 0; i < m_objects.size(); i++) {
		Entity entity(OBJECT, i, m_objects[i]);
		m_entities.push_back(entity);
	}

	for (int i = 0; i < m_obstacles.size(); i++) {
		Entity entity(OBSTACLE, i, m_obstacles[i]);
		m_entities.push_back(entity);
	}

	for (int i = 0; i < m_robots.size(); i++) {
		Entity entity(ROBOT, i, m_robots[i]);
		m_entities.push_back(entity);
	}

	return;
}

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

	m_sendedEntityNum = 0;

	srand((unsigned)time( NULL ));

	// 車輪の回転速度
	m_vel = 0.3;

	// 関節の回転速度
	m_jvel = 0.6;

	// エンティティの初期化
	InitEntityInfo();
	
	// エンティティの位置情報をゲットする
	GetEntityPositionInfo(m_entities);
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
  

bool MyController::GetEntityInfo(Vector3d &pos, std::vector<std::string> v_entities)
{
	// 候補のゴミが無い場合
	if (m_sendedEntityNum == v_entities.size()){
		m_sendedEntityNum = 0;
	return false;
	}

	printf("候補のサイズ :%d \n", v_entities.size());

	// エンティティの名前を取得します
	m_entiyName = v_entities[m_sendedEntityNum];

	// エンティティの生成
	SimObj *obj = getObj(m_entiyName.c_str());

	// エンティティの位置取得
	obj->getPosition(pos);

	printf("エンティティの座標 :%lf %lf %lf \n", pos.x(), pos.y(), pos.z());

	// 候補リストから送ったエンティティの数	
	m_sendedEntityNum++;

	return true;
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

	if (strcmp(header, ASK_ENTITY_POS_MSG) == 0) {
		printf("Received %s \n", ASK_ENTITY_POS_MSG);
		
		if (m_sendedEntityNum < m_entities.size()) {
			m_srv->sendMsgToSrv(m_messenge.GetString(ANS_ENTITY_POS_MSG, m_entities[m_sendedEntityNum].ToString()));
			m_sendedEntityNum++;
		} else {
			m_srv->sendMsgToSrv(FIN_ASK_POS_MSG);
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
		entity.GetEntityInfo(ctx);

		printf("333\n");
		// 移動させる
		printf("444 \n");	
		UpdatePosition(entity);
		printf("555 \n");
		m_srv->sendMsgToSrv(REQ_ENTITY_POS_MSG);
		printf("666\n");		
		return;
	}

	if (strcmp(header, FIN_SET_POS_MSG) == 0) {			
		m_srv->sendMsgToSrv(FIN_SET_POS_MSG);
		return;
	}

	return;

}  


void MyController::UpdatePosition(Entity entity) {
	SimObj *simObj = getObj(entity.name.c_str());
	Vector3d pos(entity.x, entity.y, entity.z);

	printf("name: %s, pos: %lf %lf %lf \n", entity.name.c_str(), pos.x(), pos.y(), pos.z());
	simObj->setPosition(pos);
	return;
}

void MyController::onCollision(CollisionEvent &evt) {}
  

bool MyController::recognizeTrash(Vector3d &pos, std::string &name)
{
	/////////////////////////////////////////////
	///////////ここでゴミを認識します////////////
	/////////////////////////////////////////////

	// 候補のゴミが無い場合
	if(m_objects.empty()){
	return false;
	}

	// ここでは乱数を使ってゴミを決定します
	int trashNum = rand() % m_objects.size();

	// ゴミの名前と位置を取得します
	name = m_objects[trashNum];
	SimObj *trash = getObj(name.c_str());

	// ゴミの位置取得
	trash->getPosition(pos);
	return true;
}


void MyController::GetEntityPositionInfo(vector<Entity> &v_entities) {

	for (int i = 0; i < v_entities.size(); i++) {
		// エンティティの生成
		SimObj *obj = getObj(v_entities[i].name.c_str());
		// エンティティの位置取得
		Vector3d pos;
		obj->getPosition(pos);
		v_entities[i].SetPosition(pos.x(), pos.y(), pos.z());
	}
	
	for (int i = 0; i < m_entities.size(); i++) {
		printf("entities: %s %d %s %lf %lf %lf \n", 
			m_entities[i].type.c_str(), m_entities[i].id, m_entities[i].name.c_str(), m_entities[i].x, m_entities[i].y, m_entities[i].z);
	}

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

