#pragma once
#include"package.h"
#include"alns.h"
#include<stdio.h>
#include<cstdlib>

#define DEPOT_NUM 124
#define NET_NUM 124
#define SHOP_NUM 613
#define DELV_NUM 9214
#define ECO_NUM 9214
#define OTO_NUM 3111

class FileIO {
public:
	static Position netpoint[NET_NUM + 1];
	static Position shop[SHOP_NUM + 1];
	static Position delivery[DELV_NUM + 1];
	static Package eco_order[ECO_NUM + 1];
	static Package oto_order[OTO_NUM + 1];
	static map<string, Package*> order_map;
	//����,���͵�,�̻��ص���Ϣ����
	static void netpoint_input(string data_path);
	static void delivery_input(string data_path);
	static void shop_input(string data_path);
	//������Ϣ����
	static void eorder_input(string data_path);
	static void otoorder_input(string data_path);
	//���solution��csv�ļ�
	static void solution_output(Solution s, FILE *fp);
	static void solution_input(Solution s, FILE *fp, int size);
};
