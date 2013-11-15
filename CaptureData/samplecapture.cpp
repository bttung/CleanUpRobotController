#include <Controller.h>
#include <ControllerEvent.h>
#include <SimObj.h>
#include <Logger.h>
#include <ViewImage.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define PI 3.141592
#define DEG2RAD(DEG) ( (PI) * (DEG) / 180.0 )
#define CAPTURE_DISTANCE 20

using namespace std;

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
	m_view = (ViewService*)connectToService("SIGViewer", 9005);
}

//定期的に呼び出される関数
double RobotController::onAction(ActionEvent &evt)
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
		int depth_C[320*240];

		thetaDEG = (ii * 9) - 180;
		thetaRAD = DEG2RAD(thetaDEG);
		r = CAPTURE_DISTANCE + 15;


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

			
		//デバッグ用		
		double xn;
		double yn;
		double zn;
		xn = my->x();
		yn = my->y();
		zn = my->z();
		LOG_MSG(("xn : %f", xn));
		LOG_MSG(("yn : %f", yn));
		LOG_MSG(("zn : %f", zn));
		

		my->setPosition(x,y,z);
		my->setAxisAndAngle(0.0, 1.0, 0.0, thetaRAD + PI);
		

		xn = my->x();
		yn = my->y();
		zn = my->z();
		LOG_MSG(("xn : %f", xn));
		LOG_MSG(("yn : %f", yn));
		LOG_MSG(("zn : %f", zn));


		



		if(m_view != NULL) {

			// ビット深度24,画像サイズ320X240の画像を取得します
			ViewImage *img = m_view->captureView(4, COLORBIT_24, IMAGE_320X240);
			if (img != NULL) {

				// 画像データを取得します
				char *buf = img->getBuffer();
				std::string cap_data;
				for(int i=0;i<320*240*3;i++){
					cap_data += buf[i];
				}

				char view_fname[256];
				sprintf(view_fname, "view%03d.txt", iImage);
				FILE *fwV = fopen(view_fname , "w");
				for(int i=0;i<320*240*3;i++){
					fprintf(fwV, "%03d \n", (unsigned char)buf[i]);
				}
				fclose(fwV);





				//Windows BMP 形式で保存します
				char fname[256];
				sprintf(fname, "s%03d.bmp", iImage);
				img->saveAsWindowsBMP(fname);

				//必要なくなったら削除します
				delete img;

				LOG_MSG(("captureIMG : %d", iImage));

			        //中央カメラの深度画像を取得します。
				ViewImage *img_C1 = m_view->distanceSensor2D(0.0,255.0,4);  
				ViewImage *img_C2 = m_view->distanceSensor2D(255.0,510.0,4);  
				ViewImage *img_C3 = m_view->distanceSensor2D(510.0,765.0,4); 
				ViewImage *img_C4 = m_view->distanceSensor2D(510.0,765.0,4);
			        //DepthImage *img = distanceSensor2D();
			        //取得した深度画像の高さと幅取得
			        int height_C = img_C1->getHeight();
			        int width_C = img_C1->getWidth();
				
	       	                //視野角取得(高さ方向）
			        double fov_C = 60.0; //my->FOV();
			        //焦点距離の計算（pixel単位）
	          		double fl_C = (img_C1->getHeight()/2)/tan(DEG2RAD(fov_C/2));	
				
				img_C1->saveAsWindowsBMP("2D_depth_C1.bmp");
				img_C2->saveAsWindowsBMP("2D_depth_C2.bmp");	
				img_C3->saveAsWindowsBMP("2D_depth_C3.bmp");
				img_C4->saveAsWindowsBMP("2D_depth_C4.bmp");				
				printf("saved img_C1,2,3,4 \n");		
				
				char *distance_C1 = img_C1->getBuffer();	
				char *distance_C2 = img_C2->getBuffer();
				char *distance_C3 = img_C3->getBuffer();
				char *distance_C4 = img_C4->getBuffer();
				for(int i=0; i<240; i++){
					for(int j=0; j<320; j++){
						depth_C[i*320+j] = (unsigned char)distance_C1[i*320+j] + (unsigned char)distance_C2[i*320+j] + (unsigned char)distance_C3[i*320+j] + (unsigned char)distance_C4[i*320+j];
					}
				}
				


				char depth_fname[256];
				sprintf(depth_fname, "depth%03d.txt", iImage++);
				
				FILE *fwC = fopen(depth_fname , "w");
				for(int i=0; i<240; i++){
					for(int j=0; j<320; j++){
						fprintf(fwC, "%04d \n", depth_C[i*320+j]);			
					}
				}
				fclose(fwC);

				delete img_C1;
				delete img_C2;
				delete img_C3;
				delete img_C4;

			}
		}

	}


	if(ii ==  21){
		LOG_MSG(("capture conpleted"));

	}



	ii++;

	return 1.0;
}

//メッセージ受信時に呼び出される関数
void RobotController::onRecvMsg(RecvMsgEvent &evt)
{
	static int iImage = 0;

	//取得したメッセージを表示します
	std::string msg = evt.getMsg();
	LOG_MSG(("msg : %s", msg.c_str()));
}

extern "C" Controller * createController ()
{
	return new RobotController;
}
