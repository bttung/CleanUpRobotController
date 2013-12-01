#pragma once

#include <string>

using namespace std;

class Entity
{
public:
	int id;
	string name;
	double x;
	double y;
	double z;

public:
	Entity() {};
	~Entity() {};
public:
	void ToConsole();
	void GetEntityInfo(char* msg);
};