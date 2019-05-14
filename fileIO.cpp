#include "fileIO.h"
#include "util.h"
#include "alns.h"
#include <stdio.h>
#include <string.h>

Position FileIO::netpoint[NET_NUM + 1];
Position FileIO::shop[SHOP_NUM + 1];
Position FileIO::delivery[DELV_NUM + 1];
Package FileIO::eco_order[ECO_NUM + 1];
Package FileIO::oto_order[OTO_NUM + 1];
map<string, Package *> FileIO::order_map;

void FileIO::netpoint_input(string data_path)
{
	int count = 0;
	string file_name = "new_1.csv";
	file_name = data_path + file_name;
	FILE *fp = fopen(file_name.c_str(), "r");
	while (count != NET_NUM)
	{
		char idchar[10], c;
		int i = 0, id = 0;
		while (1)
		{
			fscanf(fp, "%c", &idchar[i]);
			if (idchar[i] == ',')
			{
				idchar[i] = '\0';
				break;
			}
			i++;
		}
		id = Util::char_to_int(idchar);
		fscanf(fp, "%lf", &netpoint[id].lng);
		fscanf(fp, "%c", &c);
		fscanf(fp, "%lf\n", &netpoint[id].lat);
		count++;
	}
	fclose(fp);
}

void FileIO::delivery_input(string data_path)
{
	int count = 0;
	string file_name = "new_2.csv";
	file_name = data_path + file_name;
	FILE *fp = fopen(file_name.c_str(), "r");
	while (count != DELV_NUM)
	{
		char idchar[10], c;
		int i = 0, id = 0;
		while (1)
		{
			fscanf(fp, "%c", &idchar[i]);
			if (idchar[i] == ',')
			{
				idchar[i] = '\0';
				break;
			}
			i++;
		}
		id = Util::char_to_int(idchar);
		fscanf(fp, "%lf", &delivery[id].lng);
		fscanf(fp, "%c", &c);
		fscanf(fp, "%lf\n", &delivery[id].lat);
		count++;
	}
	fclose(fp);
}

void FileIO::shop_input(string data_path)
{
	int count = 0;
	string file_name = "new_3.csv";
	file_name = data_path + file_name;
	FILE *fp = fopen(file_name.c_str(), "r");
	while (count != SHOP_NUM)
	{
		char idchar[10], c;
		int i = 0, id = 0;
		while (1)
		{
			fscanf(fp, "%c", &idchar[i]);
			if (idchar[i] == ',')
			{
				idchar[i] = '\0';
				break;
			}
			i++;
		}
		id = Util::char_to_int(idchar);
		fscanf(fp, "%lf", &shop[id].lng);
		fscanf(fp, "%c", &c);
		fscanf(fp, "%lf\n", &shop[id].lat);
		count++;
	}
	fclose(fp);
}

void FileIO::eorder_input(string data_path)
{
	int count = 0;
	string file_name = "new_4.csv";
	file_name = data_path + file_name;
	FILE *fp = fopen(file_name.c_str(), "r");
	while (count != ECO_NUM)
	{
		char idchar[10];
		int i = 0, id = 0, orderid = 0;
		while (1)
		{
			fscanf(fp, "%c", &idchar[i]);
			if (idchar[i] == ',')
			{
				idchar[i] = '\0';
				break;
			}
			i++;
		}
		orderid = Util::char_to_int(idchar);
		strcpy(eco_order[orderid].id, idchar);
		order_map[idchar] = &eco_order[orderid];
		i = 0;
		while (1)
		{
			fscanf(fp, "%c", &idchar[i]);
			if (idchar[i] == ',')
			{
				idchar[i] = '\0';
				break;
			}
			i++;
		}
		id = Util::char_to_int(idchar);
		eco_order[orderid].dest = delivery[id];
		i = 0;
		while (1)
		{
			fscanf(fp, "%c", &idchar[i]);
			if (idchar[i] == ',')
			{
				idchar[i] = '\0';
				break;
			}
			i++;
		}
		id = Util::char_to_int(idchar);
		eco_order[orderid].origin = netpoint[id];
		strcpy(eco_order[orderid].depot_id, idchar);
		fscanf(fp, "%d\n", &eco_order[orderid].weight);
		//eco_order[orderid].weight = 0;
		count++;
	}
	fclose(fp);
}

void FileIO::otoorder_input(string data_path)
{
	int count = 0;
	string file_name = "new_5.csv";
	file_name = data_path + file_name;
	FILE *fp = fopen(file_name.c_str(), "r");
	while (count != OTO_NUM)
	{
		char idchar[10], dest[10], origin[10], start[10], end[10];
		int i = 0, id = 0, orderid = 0;
		while (1)
		{
			fscanf(fp, "%c", &idchar[i]);
			if (idchar[i] == ',')
			{
				idchar[i] = '\0';
				break;
			}
			i++;
		}
		orderid = Util::char_to_int(idchar);
		strcpy(oto_order[orderid].id, idchar);
		order_map[idchar] = &oto_order[orderid];
		i = 0;
		while (1)
		{
			fscanf(fp, "%c", &dest[i]);
			if (dest[i] == ',')
			{
				dest[i] = '\0';
				break;
			}
			i++;
		}
		id = Util::char_to_int(dest);
		oto_order[orderid].dest = delivery[id];
		i = 0;
		while (1)
		{
			fscanf(fp, "%c", &origin[i]);
			if (origin[i] == ',')
			{
				origin[i] = '\0';
				break;
			}
			i++;
		}
		id = Util::char_to_int(origin);
		oto_order[orderid].origin = shop[id];
		strcpy(oto_order[orderid].depot_id, origin);
		i = 0;
		while (1)
		{
			fscanf(fp, "%c", &start[i]);
			if (start[i] == ',')
			{
				start[i] = '\0';
				break;
			}
			i++;
		}
		oto_order[orderid].start_time = Util::char_to_time(start);
		i = 0;
		while (1)
		{
			fscanf(fp, "%c", &end[i]);
			if (end[i] == ',')
			{
				end[i] = '\0';
				break;
			}
			i++;
		}
		oto_order[orderid].end_time = Util::char_to_time(end);
		fscanf(fp, "%d\n", &oto_order[orderid].weight);
		double dis = Util::get_distance(oto_order[orderid].origin, oto_order[orderid].dest);
		if (Util::get_travel_time(dis) + oto_order[orderid].service_time() > oto_order[orderid].end_time - oto_order[orderid].start_time ||
			oto_order[orderid].start_time + Util::get_travel_time(dis) + oto_order[orderid].service_time() > 720)
			oto_order[orderid].weight = 0;
		//oto_order[orderid].weight = 0;
		count++;
	}
	fclose(fp);
}

void FileIO::solution_input(Solution s, FILE *fp, int size)
{
	int count = 0, eco_visit[9215], oto_visit[4823];
	memset(eco_visit, 0, sizeof(eco_visit));
	memset(oto_visit, 0, sizeof(oto_visit));
	while (count < size * 2)
	{
		int path_num = 0, package_num = 0, i = 0;
		char idchar[10];
		while (1)
		{
			fscanf(fp, "%c", &idchar[i]);
			if (idchar[i] == ',')
			{
				idchar[i] = '\0';
				break;
			}
			i++;
		}
		path_num = Util::char_to_int(idchar) - 1;
		for (i = 0; i < 5; i++)
		{
			fscanf(fp, "%c", &idchar[i]);
		}
		fscanf(fp, "%c\n", &idchar[5]);
		idchar[5] = '\0';
		package_num = Util::char_to_int(idchar);
		if (Order::check_order(idchar))
		{
			if (eco_visit[package_num])
			{
				PointOrder order = new Order(&eco_order[package_num], 1);
				s.courior[path_num].path->append(order);
			}
			else
			{
				PointOrder order = new Order(&eco_order[package_num], 0);
				s.courior[path_num].path->append(order);
				eco_visit[package_num]++;
			}
		}
		else
		{
			if (oto_visit[package_num])
			{
				PointOrder order = new Order(&oto_order[package_num], 1);
				s.courior[path_num].path->append(order);
			}
			else
			{
				PointOrder order = new Order(&oto_order[package_num], 0);
				s.courior[path_num].path->append(order);
				oto_visit[package_num]++;
			}
		}
		count++;
	}
}

void FileIO::solution_output(Solution s, FILE *fp)
{
	for (int i = 0; i < COURIOR_NUM; i++)
	{
		int j = i + 1;
		char id_char[6];
		id_char[0] = 'D';
		id_char[1] = j / 1000 + 48;
		id_char[2] = j % 1000 / 100 + 48;
		id_char[3] = j % 100 / 10 + 48;
		id_char[4] = j % 10 + 48;
		id_char[5] = '\0';
		PointOrder p = s.courior[i].path->head;
		while (p != NULL)
		{
			fprintf(fp, "%s,%s\n", id_char, p->package->id);
			p = p->next;
		}
	}
}