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


  /* @brief  ゴミを認識しゴミの位置と名前を返す
   * @return  pos ゴミの位置
   * @return  ゴミの名前
   * @return  ゴミの認識に成功した場合はtrue
   */
  bool recognizeTrash(Vector3d &pos, std::string &name); 

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

  // 移動終了時間
  double m_time;

  // 初期位置
  Vector3d m_inipos;
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

    // ゴミがある場所と名前を取得します
    if(!this->recognizeTrash(m_tpos,m_tname)){

      // ゴミが見つからないので終了
      broadcastMsgToSrv("I cannot find trash");
      m_state = 8;
    }
    // ゴミが見つかった
    else{

      // ゴミの方向に回転をはじめる
      m_time = rotateTowardObj(m_tpos, m_vel, evt.time());
      m_state = 1;
    }
    break;
  }
    // ゴミの方向に回転中
  case 1: {

    // 回転終了
    if(evt.time() >= m_time){

      // 回転を止める
      m_my->setWheelVelocity(0.0, 0.0);

      // 関節の回転を始める
      // orig
      m_my->setJointVelocity("RARM_JOINT1", -m_jvel, 0.0);

      // 50°回転
      m_time = DEG2RAD(50) / m_jvel + evt.time();

      // ゴミを取りに関節を曲げる状態に移行します
      m_state = 2;
    }
    break;
  }
    // 関節を回転中
  case 2: {

    // 関節回転終了    
    if(evt.time() >= m_time){

      m_my->setJointVelocity("RARM_JOINT1", 0.0, 0.0);

      // graspしたいパーツを取得します
      CParts *parts = m_my->getParts("RARM_LINK7");
      
      // graspします
      parts->graspObj(m_tname);
      
      // ゴミ箱の位置を取得します
      SimObj *trashbox = getObj("trashbox_1");
      Vector3d pos;
      trashbox->getPosition(pos);
      
      // ゴミ箱の方向に移動を開始します
      m_time = rotateTowardObj(pos, m_vel, evt.time());      
      m_state = 3;
    }
    
    break;
  }
    // ゴミ箱の方向に回転中
  case 3: {

    // ゴミ箱到着
    if(evt.time() >= m_time){

      // ここではゴミ箱の名前 位置は知っているものとします
      SimObj *trashbox = getObj("trashbox_1");
      Vector3d pos;
      trashbox->getPosition(pos);

      // ゴミ箱の近くに移動します
      m_time = goToObj(pos, m_vel*4, 40.0, evt.time());
      m_state = 4;
    }
    break;
  }

    // ゴミを持ってゴミ箱に向かっている状態
  case 4: {

    // ゴミ箱に到着
    if(evt.time() >= m_time){
      m_my->setWheelVelocity(0.0, 0.0);

      // grasp中のパーツを取得します
      CParts *parts = m_my->getParts("RARM_LINK7");
      
      // releaseします
      parts->releaseObj();

      // ゴミが捨てられるまで少し待つ
      sleep(1);

      // 捨てたゴミをゴミ候補から削除
      std::vector<std::string>::iterator it;
      it = std::find(m_trashes.begin(), m_trashes.end(), m_tname);
      m_trashes.erase(it);
      
      // 関節の回転を始める
      m_my->setJointVelocity("RARM_JOINT1", m_jvel, 0.0);
      m_time = DEG2RAD(50) / m_jvel + evt.time() + 1.0;      

      m_state = 5;
    }
    break;
  }
    // ゴミを捨てて関節を戻している状態
  case 5: {

    // 関節が元に戻った
    if(evt.time() >= m_time){

      // 関節の回転を止める
      m_my->setJointVelocity("RARM_JOINT1", 0.0, 0.0);

      // 最初にいた方向に体を回転させます
      m_time = rotateTowardObj(m_inipos, m_vel, evt.time());
      m_state = 6;
    }
    break;
  }
    // 元に場所に戻る方向に回転している状態
  case 6: {

    if(evt.time() >= m_time){
      m_my->setWheelVelocity(0.0, 0.0);
      
      // 最初にいた場所に移動します
      m_time = goToObj(m_inipos, m_vel*4, 5.0, evt.time());
      m_state = 7;
    }
    break;
  }

    //  元の場所に向かっている状態
  case 7: {

    // 元の場所に到着
    if(evt.time() >= m_time){
      m_my->setWheelVelocity(0.0, 0.0);
      
      // 最初の方向に回転(z軸と仮定)
      m_time = rotateTowardObj(Vector3d(0.0, 0.0, 10000.0), m_vel, evt.time());
      m_state = 8;
    }
    break;
  }
    // 元の向きに回転している状態
  case 8: {
    
    if(evt.time() >= m_time){
      // 回転を止める
      m_my->setWheelVelocity(0.0, 0.0);

      // 最初の状態に戻る
      m_state = 0;
    }
  }
  }
  return 0.1;      
}  
  
void MyController::onRecvMsg(RecvMsgEvent &evt)
{  
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

