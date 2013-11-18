#pragma once

#include <string>
#include <stdio.h>
#include <iostream>

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
