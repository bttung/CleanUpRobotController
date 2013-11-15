#include "ControllerEvent.h"  
#include "Controller.h"  
#include "Logger.h"  
#include <algorithm>

#define PI 3.1415926535

//角度からラジアンに変換します
#define DEG2RAD(DEG) ( (PI) * (DEG) / 180.0 )   

class MyController : public Controller {  
public:  
  void onInit(InitEvent &evt);  
  double onAction(ActionEvent&);  
  void onRecvMsg(RecvMsgEvent &evt); 
  void onCollision(CollisionEvent &evt); 

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
  BaseService *m_recogSrv;

  // サービスへのリクエスト複数回送信を防ぐ
  bool m_sended;
};  

void MyController::onInit(InitEvent &evt) 
{  
  m_my = getRobotObj(myname());

  // 初期位置取得
  m_my->getPosition(m_inipos);

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
  m_vel = 0.3;

  // 関節の回転速度
  m_jvel = 0.6;

  // grasp初期化
  m_grasp = false;
  m_recogSrv = NULL;
  m_sended = false;
}  
  
double MyController::onAction(ActionEvent &evt)
{  
  switch(m_state){

    // 初期状態
  case 0: {
    if(m_recogSrv == NULL){
      // ゴミ認識サービスが利用可能か調べる
      if(checkService("RecogTrash")){
	// ゴミ認識サービスに接続
	m_recogSrv = connectToService("RecogTrash");
      }
    }
    
    else if(m_recogSrv != NULL && m_sended == false){

      // カメラがついているリンク名取得
      std::string link = m_my->getCameraLinkName();
      const dReal *lpos = m_my->getParts(link.c_str())->getPosition();
      
      // カメラの位置取得(リンク座標系)
      Vector3d cpos;
      m_my->getCamPos(cpos);
      
      // カメラの位置(絶対座標系,ロボットの回転はないものとする)
      Vector3d campos(lpos[0] + cpos.x(), lpos[1] + cpos.y(), lpos[2] + cpos.z());
      char msg[1024];
      
      // カメラの方向取得(ロボットの回転,関節の回転はないものとする)
      Vector3d cdir;
      m_my->getCamDir(cdir);

      // このサンプルではカメラID1の情報のみサービスに送信します
      sprintf(msg, "RecognizeTrash %f %f %f %f %f %f", 
	      campos.x(), 
	      campos.y(), 
	      campos.z(),
	      cdir.x(),
	      cdir.y(),
	      cdir.z());

      // ゴミ認識リクエスト送信
      m_recogSrv->sendMsgToSrv(msg);
      m_sended = true;
    }
    break;
  }
  case 1: {

    // ゴミの方向に回転をはじめる
    m_time = rotateTowardObj(m_tpos, m_vel, evt.time());
    m_state = 2;
    
    break;
  }
    // ゴミの方向に回転中
  case 2: {

    // 回転終了
    if(evt.time() >= m_time){

      // 回転を止める
      m_my->setWheelVelocity(0.0, 0.0);

      // 関節の回転を始める
      m_my->setJointVelocity("RARM_JOINT1", -m_jvel, 0.0);

      // 50°回転
      m_time = DEG2RAD(50) / m_jvel + evt.time();

      // ゴミを取りに関節を曲げる状態に移行します
      m_state = 3;
    }
    break;
  }
    // 関節を回転中
  case 3: {

    // 関節回転終了    
    if(evt.time() >= m_time){

      m_my->setJointVelocity("RARM_JOINT1", 0.0, 0.0);

      // graspした状態
      if(m_grasp) {
				// ゴミ箱の位置を取得します
				SimObj *trashbox = getObj("trashbox_1");
				Vector3d pos;
				trashbox->getPosition(pos);
	
				// ゴミ箱の方向に移動を開始します
				m_time = rotateTowardObj(pos, m_vel, evt.time());      
				m_state = 4;
      }
      else{
				// graspできない
				broadcastMsgToSrv("I cannot grasp trash");
      }
    }
    
    break;
  }
    // ゴミ箱の方向に回転中
  case 4: {

    // ゴミ箱到着
    if(evt.time() >= m_time){

      // ここではゴミ箱の名前 位置は知っているものとします
      SimObj *trashbox = getObj("trashbox_1");
      Vector3d pos;
      trashbox->getPosition(pos);

      // ゴミ箱の近くに移動します
      m_time = goToObj(pos, m_vel*4, 50.0, evt.time());
      m_state = 5;
    }
    break;
  }

    // ゴミを持ってゴミ箱に向かっている状態
  case 5: {

    // ゴミ箱に到着
    if(evt.time() >= m_time){
      m_my->setWheelVelocity(0.0, 0.0);

      // grasp中のパーツを取得します
      CParts *parts = m_my->getParts("RARM_LINK7");
      
      // releaseします
      parts->releaseObj();

      // ゴミが捨てられるまで少し待つ
      sleep(1);

      // grasp終了
      m_grasp = false;

      // 関節の回転を始める
      m_my->setJointVelocity("RARM_JOINT1", m_jvel, 0.0);
      m_time = DEG2RAD(50) / m_jvel + evt.time() + 1.0;      

      m_state = 6;
    }
    break;
  }
    // ゴミを捨てて関節を戻している状態
  case 6: {

    // 関節が元に戻った
    if(evt.time() >= m_time){

      // 関節の回転を止める
      m_my->setJointVelocity("RARM_JOINT1", 0.0, 0.0);

      // 最初にいた方向に体を回転させます
      m_time = rotateTowardObj(m_inipos, m_vel, evt.time());
      m_state = 7;
    }
    break;
  }
    // 元に場所に戻る方向に回転している状態
  case 7: {

    if(evt.time() >= m_time){
      m_my->setWheelVelocity(0.0, 0.0);
      
      // 最初にいた場所に移動します
      m_time = goToObj(m_inipos, m_vel*4, 5.0, evt.time());
      m_state = 8;
    }
    break;
  }

    //  元の場所に向かっている状態
  case 8: {

    // 元の場所に到着
    if(evt.time() >= m_time){
      m_my->setWheelVelocity(0.0, 0.0);
      
      // 最初の方向に回転(z軸と仮定)
      m_time = rotateTowardObj(Vector3d(0.0, 0.0, 10000.0), m_vel, evt.time());
      m_state = 9;
    }
    break;
  }
    // 元の向きに回転している状態
  case 9: {
    
    if(evt.time() >= m_time){
      // 回転を止める
      m_my->setWheelVelocity(0.0, 0.0);

      // 最初の状態に戻る
      m_state = 0;
      m_sended = false;
    }
  }
  }
  return 0.1;      
}  
  
void MyController::onRecvMsg(RecvMsgEvent &evt)
{  
  // 送信者取得
  std::string sender = evt.getSender();

  // 送信者がゴミ認識サービスの場合
  if(sender == "RecogTrash"){
    char *all_msg = (char*)evt.getMsg();
    
    // 位置を取得
    double x = atof(strtok(all_msg," "));
    double y = atof(strtok(NULL," "));
    double z = atof(strtok(NULL," "));
    
    LOG_MSG(("recognized trash position (%f, %f, %f)", x, y, z));
    m_tpos.set(x, y, z);

    // ゴミを取りにいく状態に移行
    m_state = 1;
  }
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
				if(parts->graspObj(with[i])){
					m_grasp = true;  
				}
      }  
    }  
  }  
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

