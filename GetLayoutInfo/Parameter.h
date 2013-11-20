// Service Name
#define LAYOUT_MANAGE_SERVICE_NAME	"LayoutManager"

// Entity Type
#define OBJECT		"Object"
#define OBSTACLE	"Obstacle"
#define ROBOT		"Robot"

// define Message use when ask Entity Position
//#define	START_ASK_POS_MSG		"StartAskPosition"
//#define ASK_ENTITY_POS_MSG		"AskEntityPosition"
#define	ASK_OBJECT_POS_MSG			"AskObjectPosition"
#define	ASK_OBSTACLE_POS_MSG		"AskObstaclePosition"
#define	ASK_ROBOT_POS_MSG			"AskRobotPosition"
//#define ANS_ENTITY_POS_MSG		"AnswerEntityPosition"
#define	ANS_OBJECT_POS_MSG			"AnswerObjectPosition"
#define	ANS_OBSTACLE_POS_MSG		"AnswerObstaclePosition"
#define	ANS_ROBOT_POS_MSG			"AnswerRobotPosition"

// define Message use when each request finish
#define START_SET_POSITION_MSG		"StartSetPosition"
#define	SET_OBJECT_POS_MSG			"SetObjectPosition"
#define	SET_OBSTACLE_POS_MSG		"SetObstaclePosition"
#define	SET_ROBOT_POS_MSG			"SetRobotPosition"
#define	REQUEST_OBJECT_POS_MSG		"RequestObjectPosition"
#define	REQUEST_OBSTACLE_POS_MSG	"RequestObstaclePosition"
#define	REQUEST_ROBOT_POS_MSG		"RequestRobotPosition"

// define Message use when each request finish
#define	FIN_ASK_OBJECT_POS_MSG		"FinishAskObjectPosition"
#define	FIN_ASK_OBSTACLE_POS_MSG	"FinishAskObstaclePosition"
#define	FIN_SET_OBJECT_POS_MSG		"FinishSetObjectPosition"
#define	FIN_SET_OBSTACLE_POS_MSG	"FinishSetObstaclePosition"
#define FIN_ASK_ROBOT_POS_MSG		"FinishAskRobotPosition"
#define FIN_SET_ROBOT_POS_MSG		"FinishSetRobotPosition"

// define General Message use for Entity
#define	START_ASK_POS_MSG			"StartAskPosition"
#define ASK_ENTITY_POS_MSG			"AskEntityPosition"
#define ANS_ENTITY_POS_MSG			"AnswerEntityPosition"
#define FIN_ASK_POS_MSG				"FinishAskPosition"

#define START_SET_POS_MSG			"StartSetPosition"
#define SET_ENTITY_POS_MSG			"SetEntityPosition"
#define REQ_ENTITY_POS_MSG			"RequestEntityPosition"
#define FIN_SET_POS_MSG				"FinishSetPosition"

#define PI 3.1415926535
// DEG to RADIAN
#define DEG2RAD(DEG) ( (PI) * (DEG) / 180.0 )

