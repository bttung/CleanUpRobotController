#include "ControllerEvent.h"  
#include "Controller.h"  
#include "Logger.h"  
#include <algorithm>

#define PI 3.1415926535

//角度からラジアンに変換します
#define DEG2RAD(DEG) ( (PI) * (DEG) / 180.0 )   
#define CAPTURE_DISTANCE 50

class MyController : public Controller {  
public:  
  void onInit(InitEvent &evt);  
  double onAction(ActionEvent&);  
  void onRecvMsg(RecvMsgEvent &evt); 
  void onCollision(CollisionEvent &evt); 
	void setRobotHeadingAngle(double angle);
	void setRobotPosition(double x, double z);
  
private:
  RobotObj *m_my;
	ViewImage* m_view;
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

  srand((unsigned)time( NULL ));

  // 車輪の回転速度
  m_vel = 0.3;

  // 関節の回転速度
  m_jvel = 0.6;
}  
  
double MyController::onAction(ActionEvent &evt)
{  
	static int iImage = -1;
	static int ii = -20;

	if(ii < 21){

		SimObj *my = getObj(myname());

		//変数の宣言
		double x;
		double y;
		double z;
		double r;
		double thetaDEG;
		double thetaRAD;

		thetaDEG = (ii * 9) - 180;
		thetaRAD = DEG2RAD(thetaDEG);
		r = CAPTURE_DISTANCE;

		//座標と角度を設定
		x = r*sin(thetaRAD);
		y = my->y();
		z = r*cos(thetaRAD);

		LOG_MSG(("x : %f", x));
		LOG_MSG(("y : %f", y));
		LOG_MSG(("z : %f", z));
		LOG_MSG(("thetaDEG : %f", thetaDEG));
		LOG_MSG(("thetaRAD : %f", thetaRAD));
		LOG_MSG(("r : %f", r));
		
		my->setPosition(x,y,z);
		my->setAxisAndAngle(0.0, 1.0, 0.0, thetaRAD + PI);

		if(m_view != NULL) {

			// ビット深度24,画像サイズ320X240の画像を取得します
			ViewImage *img = m_view->captureView(1, COLORBIT_24, IMAGE_320X240);
			if (img != NULL) {

				//Windows BMP 形式で保存します
				char fname[256];
				sprintf(fname, "s%03d.bmp", iImage);
				img->saveAsWindowsBMP(fname);

				//必要なくなったら削除します
				delete img;

				LOG_MSG(("captureIMG : %d", iImage));
			}
		}
	}
  return 0.5;      
}  
  
void MyController::onRecvMsg(RecvMsgEvent &evt)
{  
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

void MyController::onCollision(CollisionEvent &evt) 
{
}

extern "C" Controller * createController() {  
  return new MyController;  
}  

