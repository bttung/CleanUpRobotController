double MyController::rotateTowardObj(Vector3d pos, double velocity, double now)
{  	
  	// 自分の回転を得る
  	Rotation myRot;
  	m_my->getRotation(myRot);
      
	// y軸の回転角度を得る(x,z方向の回転は無いと仮定)
	double qw = myRot.qw();
	double qy = myRot.qy();
	double theta = 2 * acos(fabs(qw));
	if (qw * qy < 0) {
    theta = -1 * theta;
	}
	printf("ロボットが向いている角度 current theta: %lf(deg) \n",  theta * 180 / PI);

	// 自分の位置の取得
  	Vector3d myPos;
  	m_my->getPosition(myPos);
	printf("ロボットの現在位置: x: %lf, z %lf \n", myPos.x(), myPos.z());

  	// 自分の位置からターゲットを結ぶベクトル
  	Vector3d tmpPos = pos;  
	tmpPos -= myPos;

	// 近すぎるなら，回転なし
	double dis = tmpPos.x() * tmpPos.x() + tmpPos.z() * tmpPos.z();
	if (dis < 1.0) {
		return 0.0;
	}	
  
  	// z方向からの角度 
	double rate = tmpPos.x() / tmpPos.z();
	double targetAngle = atan(rate);
	if (tmpPos.z() < 0) {
		targetAngle += PI;
	}
	printf("回転する角度 targetAngle: %lf(deg) 結ぶベクトル tmpPos.x: %lf, tmpPos.z: %lf, rate: %lf \n", targetAngle*180.0/PI, tmpPos.x(), tmpPos.z(), rate);

  targetAngle -= theta;
	if (targetAngle > PI) {
		targetAngle = targetAngle - 2 * PI;
	}
	printf("targetAngle: %lf(deg) currentAngle: %lf(deg) \n", targetAngle*180.0/PI, theta * 180.0 / PI);
	

  if (targetAngle == 0.0) {
	printf("donot need to rotate \n");
    return 0.0;
  } else {
    // 回転すべき円周距離
    double distance = m_distance * PI * fabs(targetAngle) / (2 * PI);
    // 車輪の半径から移動速度を得る
    double vel = m_radius * velocity;
    // 回転時間(u秒)
    double time = distance / vel;
	printf("rotateTime: %lf \n", time);	    
    
	// 車輪回転開始
    if (targetAngle > 0.0) {
      m_my->setWheelVelocity(-velocity, velocity);
    } else {
      m_my->setWheelVelocity(velocity, -velocity);
    }

	return now + time;
  }
  }