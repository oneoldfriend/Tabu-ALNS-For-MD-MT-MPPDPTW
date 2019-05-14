#ifndef _COURIOR_H
#define _COURIOR_H
#include"path.h"
#include<vector>

typedef class Courior *PointCourior;

using namespace std;

class Courior {
public:
	Path *path;
	int cost, wait_time, penalty;
	vector<int> arc_list;
	Courior();
	void delete_path();
	void order_min_pos(Order *origin_order, Order *dest_order, Order *best_origin, Order *best_dest, int sec_flag, int noise_flag, int method_flag);
	void insert_update(PointOrder order);
	bool check_feasible(PointOrder origin, PointOrder dest);
	void order_info_calc(PointOrder order);
	int bound_calc(PointOrder order);
	int get_cost();
	int positionBound_calc(PointOrder origin, PointOrder dest);
	static void order_info_copy(Order *dest, Order *source);
	void path_update();
	bool check_feasible_updated();
	static void order_swap(Order *dest, Order *source);
	double get_dis();
	double get_load();
};

#endif