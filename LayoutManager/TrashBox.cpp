#include "ControllerEvent.h"  
#include "Controller.h"  
#include "Logger.h"  
  
class MyController : public Controller {  
public:  
  void onInit(InitEvent &evt);  
  double onAction(ActionEvent&);  
  void onRecvMsg(RecvMsgEvent &evt); 
  void onCollision(CollisionEvent &evt); 

private:
  SimObj *m_my;
  std::vector<std::string> m_entities;
};  
  
void MyController::onInit(InitEvent &evt) {  
  m_my = getObj(myname());
  getAllEntities(m_entities);
  
}  
  
double MyController::onAction(ActionEvent &evt) 
{  
  // 自分の位置取得
  Vector3d myPos;
  m_my->getPosition(myPos);
  
  int entSize = m_entities.size();
  for(int i = 0; i < entSize; i++){

    // ロボットまたはゴミ箱の場合は除く
    if(m_entities[i] == "robot_000"  ||
       m_entities[i] == "trashbox_0" ||
       m_entities[i] == "trashbox_1" ||
       m_entities[i] == "trashbox_2"){
      continue;
    }
    // エンティティ取得
    SimObj *ent = getObj(m_entities[i].c_str());

    // 位置取得
    Vector3d pos;
    ent->getPosition(pos);

    // 自分の位置からターゲットを結ぶベクトル
    pos -= myPos;
    if(pos.length() < 35){
      // ゴミを捨てる
      ent->setPosition(myPos);
    }
  }

  return 1.0;      
}  
  
void MyController::onRecvMsg(RecvMsgEvent &evt) {  
}  

void MyController::onCollision(CollisionEvent &evt) { 
}
  
extern "C" Controller * createController() {  
  return new MyController;  
}  

