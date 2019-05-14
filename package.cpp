#include "package.h"
#include"order.h"
#include<stdio.h>
#include<string.h>
#include<cmath>


Position::Position(){
	lng = 0;
	lat = 0;
}
Package::Package(){
	origin = Position();
	dest = Position();
	weight = 0;
	start_time = 0;
	end_time = 720;
	cost = 0;
	current_dis = 0.0;
	best_dis = MAX_DS;
	Dst = 0.0;
}

int Package::service_time(){
	return round(3 * sqrt(weight) + 5);
}