#ifndef _ALNS_H_
#define _ALNS_H_
#include"solution.h"
#include"courior.h"
#include<list>
#include<map>
#include<vector>
#include<iostream>
#define sigma1 5
#define sigma2 3
#define sigma3 1
#define LIST_SIZE 3000
#define DEL_NUM 10
#define INS_NUM 4
#define roulette 0.05
#define rate 0.98
#define seg 1
#define pure_oprt 0
#define debug 0
#define ALNS_THRESHOLD 10000
#define TABU_THRESHOLD 10
#define TABU_OPRT 3
#define TABU_TENURE 20
#define MOVE_SIZE_MAX 0.04
#define MOVE_SIZE_MIN 0.05
#define ROUTE_OPRT 4
#define REGRET_K 3
#define PRUNING_LEVEL 0
#define ALLOW_LATENESS 0

using namespace std;

class DeleteList {
public:
	char id[6];
	DeleteList();
};
class OperatorInfo{
public:
	int times, score;
	double prob;
	OperatorInfo();
};
class Alns{
public:
	static Solution best;
	static Solution initial;
	static Solution news;
	static double tabu_size_rate;
	static double list_size_rate;
	static int eco_num;
	static int oto_num;
	static map<string, double> depot_lb;
	static map<string, double> alns_touched_order;
	static map<string, double> tabu_touched_order;
	static OperatorInfo del_operator[DEL_NUM];
	static OperatorInfo ins_operator[INS_NUM];
	static OperatorInfo tabu_operator[TABU_OPRT];
	static DeleteList delete_list[LIST_SIZE];
	static bool route_sort_ascend(const pair<int, double> &a, const pair<int, double> &b);
	static bool route_sort_descend(const pair<int, double> &a, const pair<int, double> &b);
	static bool pair_sort_ascend(const pair<string, double> &a, const pair<string, double> &b);
	static bool pair_sort_descend(const pair<string, double> &a, const pair<string, double> &b);
	static int noise();
	static int roulette_wheel(int flag);
	static void update_prob(int del_num, int ins_num);
	static void tabu_oprt_update();
	static void solution_copy(Solution *dest, Solution *src);
	static void alns(int input_eco_num, int input_oto_num, int input_run_num);
	static void removal_oprt_prob_reward();
	static void tabu(vector<PointCourior> routes, FILE *fp, int *best_cost, double start_time);
	static map<string, int> two_opt_tabu;
	static map<string, int> reinsert_tabu;
	static map<string, int> swap_tabu;
	static map<string, int> global_tabu;
	static double get_list_size_rate(double new_rate);
	static void tabu_selection(vector<PointCourior> *tabu_route);
	static void cost_selection(vector<PointCourior> *tabu_route);
	static void distance_selection(vector<PointCourior> *tabu_route);
	static void load_selection(vector<PointCourior> *tabu_route);
	static void get_depot_lb();
	static double scale_rate_adjust(int scale);
};

#endif
