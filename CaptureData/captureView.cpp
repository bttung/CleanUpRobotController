#include <Controller.h>
#include <ControllerEvent.h>
#include <Logger.h>
#include <ViewImage.h>
#include <math.h>

#define PI 3.141592
#define DEG2RAD(DEG) ( (PI) * (DEG) / 180.0 )

class RobotController : public Controller
{
public:
  void onInit(InitEvent &evt);
  double onAction(ActionEvent &evt);
  void onRecvMsg(RecvMsgEvent &evt);

private:
  ViewService* m_view;
  double vel;
};

void RobotController::onInit(InitEvent &evt)
{
  // 移動速度
  vel = 10.0;
  // サービスに接続
  m_view = (ViewService*)connectToService("SIGViewer");
}

//定期的に呼び出される関数
double RobotController::onAction(ActionEvent &evt)
{
  return 10.0;
}

//メッセージ受信時に呼び出される関数
void RobotController::onRecvMsg(RecvMsgEvent &evt)
{
  static int iImage = 0;

  //取得したメッセージを表示します
  std::string msg = evt.getMsg();
  LOG_MSG(("msg : %s", msg.c_str()));

  //メッセージ"capture"を受信するとcaptureViewを行います
  if (msg == "capture") {
    if(m_view != NULL) {

      // ビット深度24,画像サイズ320X240の画像を取得します
      ViewImage *img = m_view->captureView(2, COLORBIT_24, IMAGE_320X240);
      if (img != NULL) {

        // 画像データを取得します
        char *buf = img->getBuffer();

        //Windows BMP 形式で保存します
        char fname[256];
        sprintf(fname, "view%03d.bmp", iImage++);
        img->saveAsWindowsBMP(fname);

        //必要なくなったら削除します
        delete img;
      }else{
	LOG_MSG(("img is NULL \n"));
      }

    }else{
      LOG_MSG(("m_view is NULL \n"));
    }
  }

  //メッセージ"rotation"を受信すると回転します
  if (msg == "rotation"){

    SimObj *my = getObj(myname());

    // 自分のy軸周りの回転を得ます(クオータニオン)
    double qy = my->qy();

    //くオータニオンから回転角(ラジアン)を導出します
    double theta = 2*asin(qy);

    //体全体を回転させます
    double y = theta + DEG2RAD(45);
    if( y >= PI)
      y = y - 2 * PI;

    my->setAxisAndAngle(0, 1.0, 0, y);
   }

  //メッセージ"move"を受信すると自分の向いている方向に進みます
  if (msg == "move"){

    SimObj *my = getObj(myname());

    //自分の位置を得ます
    Vector3d pos;
    my->getPosition(pos);

    //y軸周りの自分の回転を得ます（クオータニオン）
    double qy = my->qy();

    //クオータニオンから回転角を導出します
    double theta = 2*asin(qy);

    //移動距離
    double dx = 0.0;
    double dz = 0.0;

    //移動する方向を決定します
    dx = sin(theta) * vel;
    dz = cos(theta) * vel;

    //移動します
    my->setPosition( pos.x() + dx, pos.y() , pos.z() + dz );
  }
}

extern "C" Controller * createController ()
{
  return new RobotController;
}
