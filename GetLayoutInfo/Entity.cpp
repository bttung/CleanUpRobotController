#include "Entity.h"

Entity::Entity() {
	printf("this is constructor \n");
}

Entity::~Entity() {
	printf("this is destructor \n");
}

void Entity::PrintToConsole() {
	printf("entity: %d	%s	%lf	%lf	%lf	\n", id, name.c_str(), x, y, z);
	return;
}

/*void Entity::GetEntityInfo(char* msg) {
	printf("aaa\n");
	char entityName[256];
	printf("bbb \n");
	printf("msg: %s \n", msg);
	sscanf(msg, "%d %s %lf %lf %lf", &id, entityName, &x, &y, &z);
	name = string(entityName);
	cout << "name: " << name << endl;
	return;
}*/
