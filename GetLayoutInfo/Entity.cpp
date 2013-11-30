#include "Entity.h"

void Entity::ToConsole() {
	printf("entity: %d	%s	%lf	%lf	%lf	\n", id, name.c_str(), x, y, z);
	return;
}

void Entity::GetEntityInfo(std::string str) {
	char entityName[256];
	sscanf(str.c_str(), "%d	%s	%lf	%lf	%lf", &id, entityName, &x, &y, &z);
	name = string(entityName);	
	return;
}

void Entity::GetEntityInfo(char* msg) {
	string str = msg;
	GetEntityInfo(str);
	return;
}
