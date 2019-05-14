#pragma once
#include"package.h"
#define MAX_DS 13024126.5
#define MIN_DS 0.0
#define MAX_DT 90.0
#define MIN_DT 0.0

typedef class Order *PointOrder;

class Order {
public:
	Position position;
	Package *package;
	PointOrder next;
	PointOrder prior;
	int wait_time, depart_time, current_weight, type, cost, visit;
	double prior_dis, next_dis, Dst;
	Order(Package *p, int i);
	static bool check_order(char *id);
	void dis_Dst_update(double prior_dis, double next_dis);
	int dep_calc(PointOrder prior_order, int delta);
	double Dst_calc(double dis, int dep_time, double ds_factor, double dt_factor);
};
