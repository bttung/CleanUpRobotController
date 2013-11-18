#include "PositionManager.h"

void PositionManager::UpdatePosition(Entity entity) {
	printf("ccc \n");	
	SimObj *simObj = getObj(entity.name);
	printf("ddd\n");
	Vector3d pos(entity.x, entity.y, entity.z);
	printf("eee\n");
	simObj->setPosition(pos);
	printf("ggg\n");	
	return;
}
