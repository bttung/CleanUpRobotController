#include "PositionManager.h"

void PositionManager::UpdatePosition(Entity entity) {
	SimObj *simObj = getObj(entity.name);
	
	Vector3d pos(entity.x, entity.y, entity.z);

	simObj->setPosition(pos);

	return;
}