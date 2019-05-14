#pragma once
#include"package.h"
#include"order.h"
#include<map>
#include<string>

using namespace std;
#define EARTH_RADIUS 6378137//地球半径
#define PI 3.14159265358979323846 //圆周率
#define speed 250 //快递员速度

class Util {
public:
	//将id转为int并输出
	static int char_to_int(char *a);
	static int string_to_int(string a);
	//将表中时间转为与早上8点的差
	static int char_to_time(char *a);
	//输出两点间距离
	static double get_distance(Position pos1, Position pos2);
	//输出行驶dis所需时间
	static int get_travel_time(double dis);
	static double rad(double d);
	static double relatedness_calc(pair<PointOrder, int> order1, pair<PointOrder, int> order2,
		double dis_factor, double time_factor, double weight_factor, double vehicle_factor, double type_factor);
};
