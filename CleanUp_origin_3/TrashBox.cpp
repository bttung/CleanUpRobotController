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
  
  // ゴミ箱のサイズ(この範囲でreleaseしなければゴミを捨てられない)
  double tboxSize_x, tboxSize_z;

  // ゴミが入ったとされる高さ方向の範囲(y方向)
  double tboxMin_y, tboxMax_y;

};  
  
void MyController::onInit(InitEvent &evt) {  
  m_my = getObj(myname());
  getAllEntities(m_entities);

  // ゴミの大きさ
  // この範囲でゴミをreleaseするとゴミを捨てたと判定
  tboxSize_x  = 20.0;
  tboxSize_z  = 40.5; 
  tboxMin_y    = 40.0;
  tboxMax_y    = 1000.0;
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
    Vector3d tpos;
    ent->getPosition(tpos);

    // ゴミ箱からゴミを結ぶベクトル
    Vector3d vec(tpos.x()-myPos.x(), tpos.y()-myPos.y(), tpos.z()-myPos.z());
    
    // ゴミがゴミ箱の中に入ったかどうか判定
    if(abs(vec.x()) < tboxSize_x/2.0 &&
       abs(vec.z()) < tboxSize_z/2.0 &&
       tpos.y() < tboxMax_y     &&
       tpos.y() > tboxMin_y     ){
      
      // ゴミがリリースされているか確認
      if(!ent->getIsGrasped()){	
				// ゴミを捨てる
				//ent->setPosition(myPos);
				tpos.y(tpos.y() /2);
				ent->setPosition(tpos);
				usleep(500000);
				tpos.y(0.0);
				ent->setPosition(tpos);
				LOG_MSG(("Clean Up succeeded !"));
      }
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

