#ifndef _SOLUTION_H_
#define _SOLUTION_H_
#include"courior.h"
#include<vector>
#include<string>


#define COURIOR_NUM 1000
#define LIST_UPPER 2700
#define LIST_LOWER 300
#define MAXCOST 999999
#define WORK_TIME 720
#define CAPACITY 140

using namespace std;

class Solution {
public:
	Courior courior[COURIOR_NUM];
	Solution();
	int vehicleNumber, cost, travelTime, waitTime, penalty, idleTime;
	//��greedy insert��ó�ʼ��.
	void get_initial();
	//�����һ��·����ͷ��㶩������ɾ����
	void random_removal();
	void worst_cost_removal();
	void spatiotemporal_removal();
	void temporal_removal();
	void spatio_removal();
	void shaw_removal();
	void dis_related_removal();
	void time_related_removal();
	void depot_related_removal();
	void wait_time_removal();
	//��ɾ�����еĶ����ӽ���ɾ��
	void list_delete(int dyna_size);
	//̰������ɾ�����еĶ���.
	void greedy_insert();
	//�κ�̰������ɾ�����еĶ���.
	void sec_greedy_insert();
	//̰��������������ɾ�����еĶ���
	void greedy_insert_with_noise();
	//�κ�̰��������������ɾ�����еĶ���
	void sec_greedy_insert_with_noise();
	void regret_insertion(int k);
	void package_info_calc_update();
	void calcAttributes();
	static void two_opt(vector<PointCourior> routes);
	static void reinsertion(vector<PointCourior> routes);
	static void swap(vector<PointCourior> routes);
	static bool relatedness_descend_sort(const pair<string, double> &a, const pair<string, double> &b);
};

#endif 
