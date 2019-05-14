#include"util.h"
#include"solution.h"
#include<cmath>
#include<string.h>


int Util::char_to_time(char *a) {
	int time, hour, minute;
	hour = (a[0] - 48) * 10 + a[1] - 48;
	minute = (a[3] - 48) * 10 + a[4] - 48;
	time = (hour - 8) * 60 + minute;
	return time;
}

int Util::char_to_int(char *a) {
	int i = 1, id = 0;
	while (a[i] != '\0') {
		id = id * 10 + (a[i] - 48);
		i++;
	}
	return id;
}

int Util::string_to_int(string a) {
	int i = 1, id = 0;
	while (a[i] != '\0') {
		id = id * 10 + (a[i] - 48);
		i++;
	}
	return id;
}

double Util::rad(double d)
{
	return d * PI / 180.0;
}

double Util::relatedness_calc(pair<PointOrder, int> order1, pair<PointOrder, int> order2,
	double dis_factor, double time_factor, double weight_factor, double vehicle_factor, double type_factor)
{
	double c_ij = double(Util::get_travel_time(Util::get_distance(order1.first->position, order2.first->position))) / WORK_TIME;
	double t_ij = double(abs((order1.first->depart_time - order2.first->depart_time))) / WORK_TIME;
	double q_ij = double(abs((order1.first->current_weight - order2.first->current_weight))) / CAPACITY;
	double same_vehicle, same_type;
	if (order1.second == order2.second)
		same_vehicle = vehicle_factor * 1;
	else
		same_vehicle = 0;
	if ((order1.first->package->id[0] == 'E'&&order2.first->package->id[0] == 'E') ||
		(order1.first->package->id[0] == 'F'&&order2.first->package->id[0] == 'F'))
		same_type = type_factor * 1;
	else
		same_type = 0;
	if (strcmp(order1.first->package->id, order2.first->package->id) == 0)
		return 0;
	else
		return 1 / (dis_factor * c_ij + time_factor * t_ij + weight_factor * q_ij) + same_type + same_vehicle;
}

//double Util::get_distance(Position a, Position b) {
//	double Dlon = (a.lng - b.lng) / 2, Dlat = (a.lat - b.lat) / 2, dis = 0, arcsin = 0;
//	if (Dlon == 0 && Dlat == 0) {
//		return 0;
//	}
//	arcsin = asin(sqrt(pow(sin(Dlat*PI / 180), 2) + cos(a.lat*PI / 180)*cos(b.lat*PI / 180)*pow(sin(Dlon*PI / 180), 2)));
//	dis = 2 * 6378137 * arcsin;
//	return dis;
//}

double Util::get_distance(Position pos1, Position pos2) {
	double lat1 = pos1.lat, lat2 = pos2.lat, lng1 = pos1.lng, lng2 = pos2.lng;
	double radLat1 = rad(lat1);
	double radLat2 = rad(lat2);
	double a = radLat1 - radLat2;
	double b = rad(lng1) - rad(lng2);

	double dst = 2 * asin((sqrt(pow(sin(a / 2), 2) + cos(radLat1) * cos(radLat2) * pow(sin(b / 2), 2))));

	dst = dst * EARTH_RADIUS;
	dst = round(dst * 10000) / 10000;
	return dst;
}

int Util::get_travel_time(double dis) {
	return round(dis / speed);
}
