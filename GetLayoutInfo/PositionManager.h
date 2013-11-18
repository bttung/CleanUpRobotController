#include "ControllerEvent.h"  
#include "Controller.h"
#include "SimObj.h"
#include "Parameter.h"
#include "Entity.h"

using namespace std;

class PositionManager: public Controller
{
public:
	void UpdatePosition(Entity entity);
};
