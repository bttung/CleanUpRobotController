#pragma once

#include <string>

using namespace std;

class Entity
{
public:
	int id;
	std::string name;
	double x;
	double y;
	double z;

public:
	Entity() {};
	~Entity() {};
public:
	void ToConsole();
	void GetEntityInfo(string str);
	void GetEntityInfo(char* msg);
};