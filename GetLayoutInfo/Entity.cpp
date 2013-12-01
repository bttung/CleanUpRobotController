#include "Entity.h"

void Entity::ToConsole() {
	printf("entity: %d	%s	%lf	%lf	%lf	\n", id, name.c_str(), x, y, z);
	return;
}

void Entity::GetEntityInfo(char* msg) {
	char entityName[256];
	printf("msg: %s \n", msg);
	sscanf(msg, "%d %s %lf %lf %lf", &id, entityName, &x, &y, &z);
	name = string(entityName);
	cout << "name: " << name << endl;
	return;
}
