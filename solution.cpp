#include "solution.h"
#include "alns.h"
#include "fileIO.h"
#include "util.h"
#include "courior.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <cmath>
#include <time.h>
#include <utility>

#define random(a, b) (rand() % (b - a + 1) + a)

#define fix_size 3000

using namespace std;

Solution::Solution()
{
	for (int i = 0; i < COURIOR_NUM; i++)
	{
		courior[i] = Courior();
	}
	vehicleNumber = 0;
	travelTime = 0;
	cost = 0;
	waitTime = 0;
	penalty = 0;
	idleTime = 0;
}

void Solution::get_initial()
{
	int eco_count = 1, oto_count = 1;
	while (eco_count < Alns::eco_num + 1 || oto_count < Alns::oto_num + 1)
	{
		float choose = rand() / float(RAND_MAX);
		//choose = 0.6;
		int best_cost = 1000000, path_num = 0, best_path = 0;
		PointOrder origin = NULL, dest = NULL, best_origin = NULL, best_dest = NULL;
		if (choose < 0.5)
		{
			if (eco_count >= ECO_NUM + 1)
				continue;
		}
		else
		{
			if (oto_count >= OTO_NUM + 1)
				continue;
			if (FileIO::oto_order[oto_count].weight == 0)
			{
				oto_count++;
				continue;
			}
		}
		while (path_num < COURIOR_NUM)
		{ //±éÀúËùÓÐÂ·¾¶,Ñ°ÕÒ¶©µ¥×î¼ÑÎ»ÖÃ
			if (choose < 0.5)
			{
				origin = new Order(&FileIO::eco_order[eco_count], 0);
				dest = new Order(&FileIO::eco_order[eco_count], 1); //³õÊ¼»¯µçÉÌ°ü¹üµÄÆðµãºÍÖÕµã¶©µ¥
			}
			else
			{
				origin = new Order(&FileIO::oto_order[oto_count], 0);
				dest = new Order(&FileIO::oto_order[oto_count], 1); //³õÊ¼»¯oto°ü¹üµÄÆðµãºÍÖÕµã¶©µ¥
			}
			courior[path_num].order_min_pos(origin, dest, NULL, NULL, 0, 0, 1); //ÕÒµ½µ±Ç°Â·¾¶¶©µ¥×î¼ÑºÏ·¨Î»ÖÃ
			if (best_cost > origin->cost + dest->cost)
			{ //¼ì²éÊÇ·ñÎª×îÖÕ×î¼ÑÎ»ÖÃ
				delete best_origin;
				delete best_dest;
				best_cost = origin->cost + dest->cost;
				best_origin = origin;
				best_dest = dest;
				best_path = path_num;
			}
			else
			{
				delete origin;
				delete dest;
			}
			path_num++;
		}
		//²åÈë¶©µ¥²¢¸üÐÂ
		printf("insert %s\n", best_origin->package->id);
		courior[best_path].path->insert_order(best_dest);
		courior[best_path].path->insert_order(best_origin);
		courior[best_path].path_update();
		if (choose < 0.5)
		{
			eco_count++;
		}
		else
		{
			oto_count++;
		}
	}
}

void Solution::random_removal()
{ //Ëæ»úÑ¡È¡Ò»ÌõÂ·¾¶µÄÍ·½áµã,²¢½«ÆäÉ¾³ý¼ÓÈëÉ¾³ý±í
	int scale = ceil((Alns::oto_num + Alns::eco_num) * Alns::list_size_rate), del_index = 0;
	vector<string> order_stack;
	for (int i = 0; i < COURIOR_NUM; i++)
	{
		PointOrder p = this->courior[i].path->head;
		while (p != NULL)
		{
			if (p->type)
			{
				p = p->next;
				continue;
			}
			else
			{
				order_stack.push_back(p->package->id);
			}
			p = p->next;
		}
	}
	while (del_index < scale)
	{
		int index = random(0, order_stack.size() - 1), count = 0;
		for (vector<string>::iterator iter = order_stack.begin();; ++iter)
		{
			if (count == index)
			{
				strcpy(Alns::delete_list[del_index++].id, iter->c_str());
				order_stack.erase(iter);
				break;
			}
			else
				count++;
		}
	}
}

void Solution::spatiotemporal_removal()
{
	map<string, double> Ds_map;
	list<pair<string, double> > Ds_list;
	for (int i = 0; i < COURIOR_NUM; i++)
	{
		PointOrder p = this->courior[i].path->head;
		while (p != NULL)
		{
			if (p->type)
			{
				Ds_map[p->package->id] += p->Dst_calc(p->prior_dis + p->next_dis, p->depart_time, 1, 1);
			}
			else
			{
				Ds_map[p->package->id] = p->Dst_calc(p->prior_dis + p->next_dis, p->depart_time, 1, 1);
			}
			p = p->next;
		}
	}
	for (map<string, double>::iterator iter = Ds_map.begin(); iter != Ds_map.end(); ++iter)
	{
		Ds_list.push_back(make_pair(iter->first, iter->second));
	}
	Ds_list.sort(relatedness_descend_sort);
	int count = 0, scale = ceil((Alns::oto_num + Alns::eco_num) * Alns::list_size_rate);
	for (list<pair<string, double> >::iterator iter = Ds_list.begin(); count < scale; ++iter)
	{
		strcpy(Alns::delete_list[count++].id, iter->first.c_str());
	}
}

void Solution::temporal_removal()
{
	map<string, double> Ds_map;
	list<pair<string, double> > Ds_list;
	for (int i = 0; i < COURIOR_NUM; i++)
	{
		PointOrder p = this->courior[i].path->head;
		while (p != NULL)
		{
			if (p->type)
			{
				Ds_map[p->package->id] += p->Dst_calc(p->prior_dis + p->next_dis, p->depart_time, 1, 0);
			}
			else
			{
				Ds_map[p->package->id] = p->Dst_calc(p->prior_dis + p->next_dis, p->depart_time, 1, 0);
			}
			p = p->next;
		}
	}
	for (map<string, double>::iterator iter = Ds_map.begin(); iter != Ds_map.end(); ++iter)
	{
		Ds_list.push_back(make_pair(iter->first, iter->second));
	}
	Ds_list.sort(relatedness_descend_sort);
	int count = 0, scale = ceil((Alns::oto_num + Alns::eco_num) * Alns::list_size_rate);
	for (list<pair<string, double> >::iterator iter = Ds_list.begin(); count < scale; ++iter)
	{
		strcpy(Alns::delete_list[count++].id, iter->first.c_str());
	}
}

void Solution::spatio_removal()
{
	map<string, double> Ds_map;
	list<pair<string, double> > Ds_list;
	for (int i = 0; i < COURIOR_NUM; i++)
	{
		PointOrder p = this->courior[i].path->head;
		while (p != NULL)
		{
			if (p->type)
			{
				Ds_map[p->package->id] += p->Dst_calc(p->prior_dis + p->next_dis, p->depart_time, 0, 1);
			}
			else
			{
				Ds_map[p->package->id] = p->Dst_calc(p->prior_dis + p->next_dis, p->depart_time, 0, 1);
			}
			p = p->next;
		}
	}
	for (map<string, double>::iterator iter = Ds_map.begin(); iter != Ds_map.end(); ++iter)
	{
		Ds_list.push_back(make_pair(iter->first, iter->second));
	}
	Ds_list.sort(relatedness_descend_sort);
	int count = 0, scale = ceil((Alns::oto_num + Alns::eco_num) * Alns::list_size_rate);
	for (list<pair<string, double> >::iterator iter = Ds_list.begin(); count < scale; ++iter)
	{
		strcpy(Alns::delete_list[count++].id, iter->first.c_str());
	}
}

void Solution::shaw_removal()
{
	map<string, int> package_courior_map;
	map<string, PointOrder> package_origin_map;
	map<string, PointOrder> package_dest_map;
	for (int i = 0; i < COURIOR_NUM; i++)
	{
		PointOrder p = this->courior[i].path->head;
		while (p != NULL)
		{
			if (p->type)
			{
				package_dest_map[p->package->id] = p;
				p = p->next;
				continue;
			}
			package_courior_map[p->package->id] = i;
			package_origin_map[p->package->id] = p;
			p = p->next;
		}
	}
	srand((int)time(0));
	int choose = random(1, package_courior_map.size()), count = 0, chosen_courior = 0;
	PointOrder chosen_order_origin = NULL, chosen_order_dest = NULL;
	for (map<string, int>::iterator iter = package_courior_map.begin(); count++ < choose; ++iter)
	{
		chosen_order_origin = package_origin_map[iter->first];
		chosen_order_dest = package_dest_map[iter->first];
		chosen_courior = iter->second;
	}
	map<string, double> relatedness_map;
	list<pair<string, double> > relatedness_list;
	for (int i = 0; i < COURIOR_NUM; i++)
	{
		PointOrder p = this->courior[i].path->head;
		while (p != NULL)
		{
			if (p->type)
			{
				relatedness_map[p->package->id] += Util::relatedness_calc(make_pair(p, i), make_pair(chosen_order_dest, chosen_courior),
																		  1, 1, 1, 1, 1);
			}
			else
			{
				relatedness_map[p->package->id] = Util::relatedness_calc(make_pair(p, i), make_pair(chosen_order_origin, chosen_courior),
																		 1, 1, 1, 1, 1);
			}
			p = p->next;
		}
	}
	for (map<string, double>::iterator iter = relatedness_map.begin(); iter != relatedness_map.end(); ++iter)
	{
		relatedness_list.push_back(make_pair(iter->first, iter->second));
	}
	relatedness_list.sort(relatedness_descend_sort);
	count = 0;
	int scale = ceil((Alns::oto_num + Alns::eco_num) * Alns::list_size_rate);
	strcpy(Alns::delete_list[count++].id, chosen_order_origin->package->id);
	for (list<pair<string, double> >::iterator iter = relatedness_list.begin(); count < scale; ++iter)
	{
		strcpy(Alns::delete_list[count++].id, iter->first.c_str());
	}
}

void Solution::dis_related_removal()
{
	map<PointOrder, int> package_courior_map;
	for (int i = 0; i < COURIOR_NUM; i++)
	{
		PointOrder p = this->courior[i].path->head;
		while (p != NULL)
		{
			if (p->type)
			{
				p = p->next;
				continue;
			}
			package_courior_map[p] = i;
			p = p->next;
		}
	}
	srand((int)time(0));
	int choose = random(1, package_courior_map.size()), count = 0, chosen_courior = 0;
	PointOrder chosen_order = NULL;
	for (map<PointOrder, int>::iterator iter = package_courior_map.begin(); count++ < choose; ++iter)
	{
		chosen_order = iter->first;
		chosen_courior = iter->second;
	}
	map<string, double> relatedness_map;
	list<pair<string, double> > relatedness_list;
	for (int i = 0; i < COURIOR_NUM; i++)
	{
		PointOrder p = this->courior[i].path->head;
		while (p != NULL)
		{
			if (p->type)
			{
				relatedness_map[p->package->id] += Util::relatedness_calc(make_pair(p, i), make_pair(chosen_order, chosen_courior),
																		  1, 0, 0, 1, 1);
			}
			else
			{
				relatedness_map[p->package->id] = Util::relatedness_calc(make_pair(p, i), make_pair(chosen_order, chosen_courior),
																		 1, 0, 0, 1, 1);
			}
			p = p->next;
		}
	}
	for (map<string, double>::iterator iter = relatedness_map.begin(); iter != relatedness_map.end(); ++iter)
	{
		relatedness_list.push_back(make_pair(iter->first, iter->second));
	}
	count = 0;
	relatedness_list.sort(relatedness_descend_sort);
	int scale = ceil((Alns::oto_num + Alns::eco_num) * Alns::list_size_rate);
	strcpy(Alns::delete_list[count++].id, chosen_order->package->id);
	for (list<pair<string, double> >::iterator iter = relatedness_list.begin(); count < scale; ++iter)
	{
		strcpy(Alns::delete_list[count++].id, iter->first.c_str());
	}
}

void Solution::time_related_removal()
{
	map<PointOrder, int> package_courior_map;
	for (int i = 0; i < COURIOR_NUM; i++)
	{
		PointOrder p = this->courior[i].path->head;
		while (p != NULL)
		{
			if (p->type)
			{
				p = p->next;
				continue;
			}
			package_courior_map[p] = i;
			p = p->next;
		}
	}
	srand((int)time(0));
	int choose = random(1, package_courior_map.size()), count = 0, chosen_courior = 0;
	PointOrder chosen_order = NULL;
	for (map<PointOrder, int>::iterator iter = package_courior_map.begin(); count++ < choose; ++iter)
	{
		chosen_order = iter->first;
		chosen_courior = iter->second;
	}
	map<string, double> relatedness_map;
	list<pair<string, double> > relatedness_list;
	for (int i = 0; i < COURIOR_NUM; i++)
	{
		PointOrder p = this->courior[i].path->head;
		while (p != NULL)
		{
			if (p->type)
			{
				relatedness_map[p->package->id] += Util::relatedness_calc(make_pair(p, i), make_pair(chosen_order, chosen_courior),
																		  0, 1, 0, 1, 1);
			}
			else
			{
				relatedness_map[p->package->id] = Util::relatedness_calc(make_pair(p, i), make_pair(chosen_order, chosen_courior),
																		 0, 1, 0, 1, 1);
			}
			p = p->next;
		}
	}
	for (map<string, double>::iterator iter = relatedness_map.begin(); iter != relatedness_map.end(); ++iter)
	{
		relatedness_list.push_back(make_pair(iter->first, iter->second));
	}
	relatedness_list.sort(relatedness_descend_sort);
	count = 0;
	int scale = ceil((Alns::oto_num + Alns::eco_num) * Alns::list_size_rate);
	strcpy(Alns::delete_list[count++].id, chosen_order->package->id);
	for (list<pair<string, double> >::iterator iter = relatedness_list.begin(); count < scale; ++iter)
	{
		strcpy(Alns::delete_list[count++].id, iter->first.c_str());
	}
}

void Solution::depot_related_removal()
{
	map<string, double> depot_route;
	list<pair<string, double> > depot_list;
	map<string, double> order_route;
	map<string, list<pair<string, double> > > depot_order_list;
	for (int i = 0; i < COURIOR_NUM; i++)
	{
		PointOrder p = this->courior[i].path->head;
		while (p != NULL)
		{
			if (Order::check_order(p->package->id) == false)
			{
				p = p->next;
				continue;
			}
			if (p->type)
			{
				order_route[p->package->id] += Util::get_distance(p->position, p->prior->position);
				depot_route[p->package->depot_id] += order_route[p->package->id];
				depot_order_list[p->package->depot_id].push_back(make_pair(p->package->id, order_route[p->package->id]));
			}
			else
			{
				if (p->prior != NULL)
				{
					order_route[p->package->id] = Util::get_distance(p->position, p->prior->position);
				}
				else
				{
					order_route[p->package->id] = 0;
				}
			}
			p = p->next;
		}
	}
	for (map<string, double>::iterator iter = depot_route.begin(); iter != depot_route.end(); ++iter)
	{
		double order_num = depot_order_list[iter->first].size();
		depot_list.push_back(make_pair(iter->first, (iter->second - Alns::depot_lb[iter->first]) / order_num));
	}
	depot_list.sort(relatedness_descend_sort);
	for (map<string, list<pair<string, double> > >::iterator iter = depot_order_list.begin(); iter != depot_order_list.end(); ++iter)
	{
		iter->second.sort(relatedness_descend_sort);
	}
	int count = 0, scale = ceil((Alns::oto_num + Alns::eco_num) * Alns::list_size_rate);
	for (list<pair<string, double> >::iterator depot_iter = depot_list.begin(); depot_iter != depot_list.end(); ++depot_iter)
	{
		for (list<pair<string, double> >::iterator order_iter = depot_order_list[depot_iter->first].begin(); order_iter != depot_order_list[depot_iter->first].end(); ++order_iter)
		{
			strcpy(Alns::delete_list[count++].id, order_iter->first.c_str());
			if (count == scale)
				break;
		}
		if (count == scale)
			break;
	}
}

void Solution::wait_time_removal()
{
	list<pair<string, double> > wait_time_list;
	for (int i = 0; i < COURIOR_NUM; i++)
	{
		PointOrder p = this->courior[i].path->head;
		while (p != NULL)
		{
			if (Order::check_order(p->package->id) != true && p->type == 0)
			{
				wait_time_list.push_back(make_pair(p->package->id, double(p->wait_time)));
			}
			p = p->next;
		}
	}
	wait_time_list.sort(relatedness_descend_sort);
	int count = 0, scale = ceil((Alns::oto_num + Alns::eco_num) * Alns::list_size_rate);
	for (list<pair<string, double> >::iterator iter = wait_time_list.begin(); iter != wait_time_list.end(); ++iter)
	{
		strcpy(Alns::delete_list[count++].id, iter->first.c_str());
		if (count == scale)
			break;
	}
}

void Solution::worst_cost_removal()
{
	int count = 0, scale = ceil((Alns::oto_num + Alns::eco_num) * Alns::list_size_rate);
	list<pair<string, double> > order_list;
	for (int i = 0; i < COURIOR_NUM; i++)
	{
		PointOrder p = this->courior[i].path->head;
		while (p != NULL)
		{
			if (p->type == 0)
			{
				order_list.push_back(make_pair(p->package->id, double(p->package->cost)));
			}
			p = p->next;
		}
	}
	order_list.sort(relatedness_descend_sort);
	for (list<pair<string, double> >::iterator iter = order_list.begin(); count < scale; ++iter)
	{
		strcpy(Alns::delete_list[count++].id, iter->first.c_str());
	}
}

void Solution::list_delete(int order_num)
{ //½«É¾³ý±íÖÐµÄ¶©µ¥´ÓÂ·¾¶ÖÐÉ¾³ý
	for (int i = 0; i < COURIOR_NUM; i++)
	{
		PointOrder p = courior[i].path->head, temp = NULL;
		while (p != NULL)
		{
			temp = p->next;
			for (int j = 0; j < order_num; j++)
			{
				if (strcmp(p->package->id, Alns::delete_list[j].id) == 0)
				{
					courior[i].path->delete_order(p);
					delete p;
					break;
				}
			}
			p = temp;
		}
		courior[i].path_update();
	}
}

void Solution::greedy_insert()
{
	for (int i = 0; i < LIST_SIZE; i++)
	{
		if (Alns::delete_list[i].id[0] == '\0')
			continue;
		int id = Util::char_to_int(Alns::delete_list[i].id);
		int best_cost = 1000000, path_num = 0, best_path = 0, bound = 0;
		PointOrder best_origin = NULL, best_dest = NULL;
		while (path_num < COURIOR_NUM)
		{ //±éÀúËùÓÐÂ·¾¶,Ñ°ÕÒ¶©µ¥×î¼ÑÎ»ÖÃ
			//¼ì²é²åÈë¶©µ¥µÄÀàÐÍ²¢³õÊ¼»¯
			PointOrder origin = NULL, dest = NULL;
			if (Order::check_order(Alns::delete_list[i].id))
			{
				origin = new Order(&FileIO::eco_order[id], 0);
				dest = new Order(&FileIO::eco_order[id], 1);
			}
			else
			{
				origin = new Order(&FileIO::oto_order[id], 0);
				dest = new Order(&FileIO::oto_order[id], 1);
			}
			if (PRUNING_LEVEL == 2)
			{
				bound = courior[path_num].bound_calc(origin);
				if (best_cost < bound)
				{
					path_num++;
					delete origin;
					delete dest;
					continue;
				}
			}
			courior[path_num].order_min_pos(origin, dest, NULL, NULL, 0, 0, 1); //ÕÒµ½µ±Ç°Â·¾¶¶©µ¥×î¼ÑºÏ·¨Î»ÖÃ
			if (best_cost > origin->cost + dest->cost)
			{
				delete best_origin;
				delete best_dest;
				best_cost = origin->cost + dest->cost;
				best_origin = origin;
				best_dest = dest;
				best_path = path_num;
			}
			else
			{
				delete origin;
				delete dest;
			}
			path_num++;
		}
		courior[best_path].path->insert_order(best_dest);
		courior[best_path].path->insert_order(best_origin);
		courior[best_path].path_update();
		Alns::delete_list[i].id[0] = '\0';
		if (debug)
		{
			printf("²åÈë¶©µ¥%s\n", best_origin->package->id);
		}
	}
}

void Solution::sec_greedy_insert()
{
	for (int i = 0; i < LIST_SIZE; i++)
	{
		if (Alns::delete_list[i].id[0] == '\0')
			continue;
		int id = Util::char_to_int(Alns::delete_list[i].id);
		int best_cost = 100000000, sec_path = 0, path_num = 0, bound = 0;
		PointOrder best_origin = NULL, best_dest = NULL, sec_origin = NULL, sec_dest = NULL;
		while (path_num < COURIOR_NUM)
		{ //±éÀúËùÓÐÂ·¾¶,Ñ°ÕÒ¶©µ¥×î¼ÑÎ»ÖÃ
			//¼ì²é²åÈë¶©µ¥µÄÀàÐÍ²¢³õÊ¼»¯
			PointOrder origin = NULL, dest = NULL;
			if (Order::check_order(Alns::delete_list[i].id))
			{
				origin = new Order(&FileIO::eco_order[id], 0);
				dest = new Order(&FileIO::eco_order[id], 1);
			}
			else
			{
				origin = new Order(&FileIO::oto_order[id], 0);
				dest = new Order(&FileIO::oto_order[id], 1);
			}
			if (PRUNING_LEVEL == 2)
			{
				bound = courior[path_num].bound_calc(origin);
				if (best_cost < bound)
				{
					path_num++;
					delete origin;
					delete dest;
					continue;
				}
			}
			courior[path_num].order_min_pos(origin, dest, NULL, NULL, 0, 0, 1); //ÕÒµ½µ±Ç°Â·¾¶¶©µ¥×î¼ÑºÏ·¨Î»ÖÃ
			if (best_cost > origin->cost + dest->cost)
			{
				delete best_origin;
				delete best_dest;
				best_cost = origin->cost + dest->cost;
				best_origin = origin;
				best_dest = dest;
			}
			else
			{
				delete origin;
				delete dest;
			}
			path_num++;
		}
		best_cost = 10000000;
		path_num = 0;
		while (path_num < COURIOR_NUM)
		{ //±éÀúËùÓÐÂ·¾¶,Ñ°ÕÒ¶©µ¥×î¼ÑÎ»ÖÃ//¼ì²é²åÈë¶©µ¥µÄÀàÐÍ²¢³õÊ¼»¯
			PointOrder origin = NULL, dest = NULL;
			if (Order::check_order(Alns::delete_list[i].id))
			{
				origin = new Order(&FileIO::eco_order[id], 0);
				dest = new Order(&FileIO::eco_order[id], 1);
			}
			else
			{
				origin = new Order(&FileIO::oto_order[id], 0);
				dest = new Order(&FileIO::oto_order[id], 1);
			}
			if (PRUNING_LEVEL == 2)
			{
				bound = courior[path_num].bound_calc(origin);
				if (best_cost < bound)
				{
					path_num++;
					delete origin;
					delete dest;
					continue;
				}
			}
			courior[path_num].order_min_pos(origin, dest, best_origin, best_dest, 1, 0, 1); //ÕÒµ½µ±Ç°Â·¾¶¶©µ¥´Î¼ÑºÏ·¨Î»ÖÃ
			if (best_cost > origin->cost + dest->cost)
			{
				delete sec_origin;
				delete sec_dest;
				best_cost = origin->cost + dest->cost;
				sec_origin = origin;
				sec_dest = dest;
				sec_path = path_num;
			}
			else
			{
				delete origin;
				delete dest;
			}
			path_num++;
		}
		courior[sec_path].path->insert_order(sec_dest);
		courior[sec_path].path->insert_order(sec_origin);
		courior[sec_path].path_update();
		Alns::delete_list[i].id[0] = '\0';
		if (debug)
		{
			printf("²åÈë¶©µ¥%s\n", sec_origin->package->id);
		}
	}
}

void Solution::greedy_insert_with_noise()
{
	for (int i = 0; i < LIST_SIZE; i++)
	{
		if (Alns::delete_list[i].id[0] == '\0')
			continue;
		int id = Util::char_to_int(Alns::delete_list[i].id);
		int best_cost = 1000000, path_num = 0, best_path = 0, bound = 0;
		PointOrder best_origin = NULL, best_dest = NULL;
		while (path_num < COURIOR_NUM)
		{ //±éÀúËùÓÐÂ·¾¶,Ñ°ÕÒ¶©µ¥×î¼ÑÎ»ÖÃ
			//¼ì²é²åÈë¶©µ¥µÄÀàÐÍ²¢³õÊ¼»¯
			PointOrder origin = NULL, dest = NULL;
			if (Order::check_order(Alns::delete_list[i].id))
			{
				origin = new Order(&FileIO::eco_order[id], 0);
				dest = new Order(&FileIO::eco_order[id], 1);
			}
			else
			{
				origin = new Order(&FileIO::oto_order[id], 0);
				dest = new Order(&FileIO::oto_order[id], 1);
			}
			if (PRUNING_LEVEL == 2)
			{
				bound = courior[path_num].bound_calc(origin);
				if (best_cost < bound)
				{
					path_num++;
					delete origin;
					delete dest;
					continue;
				}
			}
			courior[path_num].order_min_pos(origin, dest, NULL, NULL, 0, 1, 1); //ÕÒµ½µ±Ç°Â·¾¶¶©µ¥×î¼ÑºÏ·¨Î»ÖÃ
			if (best_cost > origin->cost + dest->cost)
			{
				delete best_origin;
				delete best_dest;
				best_cost = origin->cost + dest->cost;
				best_origin = origin;
				best_dest = dest;
				best_path = path_num;
			}
			else
			{
				delete origin;
				delete dest;
			}
			path_num++;
		}
		courior[best_path].path->insert_order(best_dest);
		courior[best_path].path->insert_order(best_origin);
		courior[best_path].path_update();
		Alns::delete_list[i].id[0] = '\0';
		if (debug)
		{
			printf("²åÈë¶©µ¥%s\n", best_origin->package->id);
		}
	}
}

void Solution::sec_greedy_insert_with_noise()
{
	for (int i = 0; i < LIST_SIZE; i++)
	{
		if (Alns::delete_list[i].id[0] == '\0')
			continue;
		int id = Util::char_to_int(Alns::delete_list[i].id);
		int best_cost = 100000000, sec_path = 0, path_num = 0, bound = 0;
		PointOrder best_origin = NULL, best_dest = NULL, sec_origin = NULL, sec_dest = NULL;
		while (path_num < COURIOR_NUM)
		{ //±éÀúËùÓÐÂ·¾¶,Ñ°ÕÒ¶©µ¥×î¼ÑÎ»ÖÃ
			//¼ì²é²åÈë¶©µ¥µÄÀàÐÍ²¢³õÊ¼»¯
			PointOrder origin = NULL, dest = NULL;
			if (Order::check_order(Alns::delete_list[i].id))
			{
				origin = new Order(&FileIO::eco_order[id], 0);
				dest = new Order(&FileIO::eco_order[id], 1);
			}
			else
			{
				origin = new Order(&FileIO::oto_order[id], 0);
				dest = new Order(&FileIO::oto_order[id], 1);
			}
			if (PRUNING_LEVEL == 2)
			{
				bound = courior[path_num].bound_calc(origin);
				if (best_cost < bound)
				{
					path_num++;
					delete origin;
					delete dest;
					continue;
				}
			}
			courior[path_num].order_min_pos(origin, dest, NULL, NULL, 0, 1, 1); //ÕÒµ½µ±Ç°Â·¾¶¶©µ¥×î¼ÑºÏ·¨Î»ÖÃ
			if (best_cost > origin->cost + dest->cost)
			{
				delete best_origin;
				delete best_dest;
				best_cost = origin->cost + dest->cost;
				best_origin = origin;
				best_dest = dest;
			}
			else
			{
				delete origin;
				delete dest;
			}
			path_num++;
		}
		best_cost = 10000000;
		path_num = 0;
		while (path_num < COURIOR_NUM)
		{ //±éÀúËùÓÐÂ·¾¶,Ñ°ÕÒ¶©µ¥×î¼ÑÎ»ÖÃ
			//¼ì²é²åÈë¶©µ¥µÄÀàÐÍ²¢³õÊ¼»¯
			PointOrder origin = NULL, dest = NULL;
			if (Order::check_order(Alns::delete_list[i].id))
			{
				origin = new Order(&FileIO::eco_order[id], 0);
				dest = new Order(&FileIO::eco_order[id], 1);
			}
			else
			{
				origin = new Order(&FileIO::oto_order[id], 0);
				dest = new Order(&FileIO::oto_order[id], 1);
			}
			if (PRUNING_LEVEL == 2)
			{
				bound = courior[path_num].bound_calc(origin);
				if (best_cost < bound)
				{
					path_num++;
					delete origin;
					delete dest;
					continue;
				}
			}
			courior[path_num].order_min_pos(origin, dest, best_origin, best_dest, 1, 1, 1); //ÕÒµ½µ±Ç°Â·¾¶¶©µ¥´Î¼ÑºÏ·¨Î»ÖÃ
			if (best_cost > origin->cost + dest->cost)
			{
				delete sec_origin;
				delete sec_dest;
				best_cost = origin->cost + dest->cost;
				sec_origin = origin;
				sec_dest = dest;
				sec_path = path_num;
			}
			else
			{
				delete origin;
				delete dest;
			}
			path_num++;
		}
		courior[sec_path].path->insert_order(sec_dest);
		courior[sec_path].path->insert_order(sec_origin);
		courior[sec_path].path_update();
		Alns::delete_list[i].id[0] = '\0';
		if (debug)
		{
			printf("²åÈë¶©µ¥%s\n", sec_origin->package->id);
		}
	}
}

void Solution::regret_insertion(int k)
{
	map<string, pair<pair<PointOrder, PointOrder>, int> > order_position_map;
	for (int order_inserted = 0; order_inserted < round((Alns::oto_num + Alns::eco_num) * Alns::list_size_rate); order_inserted++)
	{
		list<pair<string, double> > order_regret_list;
		for (int i = 0; i < LIST_SIZE; i++)
		{
			if (Alns::delete_list[i].id[0] == '\0')
				continue;
			int id = Util::char_to_int(Alns::delete_list[i].id);
			int best_cost = 1000000, path_num = 0, best_path = 0, bound = 0;
			PointOrder best_origin = NULL, best_dest = NULL;
			list<int> regret_list;
			while (path_num < COURIOR_NUM)
			{ //±éÀúËùÓÐÂ·¾¶,Ñ°ÕÒ¶©µ¥×î¼ÑÎ»ÖÃ
				//¼ì²é²åÈë¶©µ¥µÄÀàÐÍ²¢³õÊ¼»¯
				PointOrder origin = NULL, dest = NULL;
				if (Order::check_order(Alns::delete_list[i].id))
				{
					origin = new Order(&FileIO::eco_order[id], 0);
					dest = new Order(&FileIO::eco_order[id], 1);
				}
				else
				{
					origin = new Order(&FileIO::oto_order[id], 0);
					dest = new Order(&FileIO::oto_order[id], 1);
				}
				if (PRUNING_LEVEL == 2)
				{
					bound = courior[path_num].bound_calc(origin);
					if (best_cost < bound)
					{
						path_num++;
						delete origin;
						delete dest;
						continue;
					}
				}
				courior[path_num].order_min_pos(origin, dest, NULL, NULL, 0, 0, 1); //ÕÒµ½µ±Ç°Â·¾¶¶©µ¥×î¼ÑºÏ·¨Î»ÖÃ
				regret_list.push_back(origin->cost + dest->cost);
				if (best_cost > origin->cost + dest->cost)
				{
					delete best_origin;
					delete best_dest;
					best_cost = origin->cost + dest->cost;
					best_origin = origin;
					best_dest = dest;
					best_path = path_num;
				}
				else
				{
					delete origin;
					delete dest;
				}
				path_num++;
			}
			regret_list.sort(); //ascend
			int regret_value = 0, step = 0, best_regret = regret_list.front();
			for (list<int>::iterator iter = regret_list.begin(); step++ <= k; ++iter)
			{
				regret_value += *iter - best_regret;
			}
			order_regret_list.push_back(make_pair(best_origin->package->id, (double)regret_value));
			delete order_position_map[best_origin->package->id].first.first;
			delete order_position_map[best_origin->package->id].first.second;
			order_position_map[best_origin->package->id] = make_pair(make_pair(best_origin, best_dest), best_path);
		}
		order_regret_list.sort(relatedness_descend_sort);
		string regret_order_id = order_regret_list.front().first;
		int path_inseted = order_position_map[regret_order_id].second;
		this->courior[path_inseted].path->insert_order(order_position_map[regret_order_id].first.second);
		this->courior[path_inseted].path->insert_order(order_position_map[regret_order_id].first.first);
		this->courior[path_inseted].path_update();
		for (int del_index = 0; del_index < round((Alns::oto_num + Alns::eco_num) * Alns::list_size_rate); del_index++)
		{
			if (strcmp(Alns::delete_list[del_index].id, regret_order_id.c_str()) == 0)
				Alns::delete_list[del_index].id[0] = '\0';
		}
	}
}

void Solution::package_info_calc_update()
{
	for (int i = 0; i < COURIOR_NUM; i++)
	{
		PointOrder p = courior[i].path->head;
		PointOrder origin_prior = NULL, origin_next = NULL, dest_prior = NULL, dest_next = NULL;
		while (p != NULL)
		{
			//¶ÔpËùÖ¸¶©µ¥µÄcost½øÐÐ¸üÐÂ
			if (p->type == 0)
			{
				int old_cost = 0, new_cost = 0;
				PointOrder origin = p, dest = NULL;
				p = p->next;
				//ÕÒµ½¶ÔÓ¦µÄdest
				while (strcmp(p->package->id, origin->package->id) != 0)
				{
					p = p->next;
				}
				dest = p;
				//±£´æ¶©µ¥ÆðµãÖÕµãµÄÇ°ºó¼ÌÐÅÏ¢.
				origin_prior = origin->prior;
				origin_next = origin->next;
				dest_prior = dest->prior;
				dest_next = dest->next;
				//É¾³ýoriginºÍdest
				courior[i].path->delete_order(origin);
				courior[i].path->delete_order(dest);
				//¸üÐÂ²¢¼ÆËãµ±Ç°Â·¾¶cost
				courior[i].path_update();
				old_cost = courior[i].get_cost();
				//»¹Ô­ÆðµãÖÕµãÇ°ºó¼ÌÐÅÏ¢
				origin->prior = origin_prior;
				origin->next = origin_next;
				dest->prior = dest_prior;
				dest->next = dest_next;
				//²åÈëoriginºÍdest
				courior[i].path->insert_order(dest);
				courior[i].path->insert_order(origin);
				//¸üÐÂ²¢¼ÆËãµ±Ç°Â·¾¶cost
				courior[i].path_update();
				new_cost = courior[i].get_cost();
				//½«ÐÂcostÓë¾Écost×ö²îµÃµ½¶ÔÓ¦¶©µ¥µÄcost
				origin->cost = 0;
				dest->cost = new_cost - old_cost;
				dest->package->cost = dest->cost;
				//½«p»¹Ô­
				p = origin;
			}
			if (p->prior != NULL)
			{
				p->prior_dis = Util::get_distance(p->prior->position, p->position);
			}
			else
			{
				p->prior_dis = 0;
			}
			if (p->next != NULL)
			{
				p->next_dis = Util::get_distance(p->position, p->next->position);
			}
			else
			{
				p->next_dis = 0;
			}
			if (p->type)
				p->package->cost = p->cost;
			p->dis_Dst_update(p->prior_dis, p->next_dis);
			p = p->next;
		}
	}
}

void Solution::two_opt(vector<PointCourior> routes)
{
	int best_cost = MAXCOST, best_order[2] = {0, 0}, best_comb = -1, doable_route = 0;
	Courior *best_route = NULL;
	for (vector<PointCourior>::iterator iter = routes.begin(); iter != routes.end(); ++iter)
	{
		int feasible = 0;
		Courior *courior = *iter;
		courior->arc_list.clear();
		PointOrder first_route = courior->path->head;
		courior->cost = courior->cost;
		while (first_route != courior->path->tail)
		{
			if (first_route->type == 0)
			{
				feasible += 1;
			}
			else
			{
				feasible -= 1;
			}
			if (feasible == 0)
			{
				courior->arc_list.push_back(first_route->visit);
			}
			first_route = first_route->next;
		}
		if (courior->arc_list.size() >= 2)
			doable_route++;
	}
	if (doable_route <= 0)
		return;
	for (vector<PointCourior>::iterator iter = routes.begin(); iter != routes.end(); ++iter)
	{
		Courior *courior = *iter;
		if (courior->arc_list.size() < 2)
			continue;
		for (std::vector<int>::iterator first_arc = courior->arc_list.begin(); first_arc != courior->arc_list.end(); ++first_arc)
		{
			PointOrder p = courior->path->head;
			if (first_arc == courior->arc_list.end())
				break;
			std::vector<int>::iterator second_arc = ++first_arc;
			--first_arc;
			for (; second_arc != courior->arc_list.end(); ++second_arc)
			{
				int delta_cost = 0;
				PointOrder first_order = courior->path->head;
				PointOrder second_order = courior->path->head;
				while (first_order->visit != *first_arc)
				{
					first_order = first_order->next;
				}
				while (second_order->visit != *second_arc)
				{
					second_order = second_order->next;
				}
				string order1_str = first_order->package->id, order2_str = second_order->package->id;
				string comb_str = order1_str + order2_str;
				if (Alns::two_opt_tabu[comb_str] > 0 || Alns::global_tabu[order1_str] > 0 || Alns::global_tabu[order2_str] > 0)
					continue;
				PointOrder first_head = courior->path->head,
						   first_tail = first_order,
						   second_head = first_order->next,
						   second_tail = second_order,
						   third_head = second_order->next,
						   third_tail = courior->path->tail;
				int combination = 0, origin_cost = courior->get_cost();
				//³¢ÊÔ5ÖÖ×éºÏ
				while (combination < 5)
				{
					switch (combination)
					{
					case 0:
					{
						//1->3->2
						first_head->prior = NULL;
						first_tail->next = third_head;
						third_head->prior = first_tail;
						third_tail->next = second_head;
						second_head->prior = third_tail;
						second_tail->next = NULL;
						courior->path->head = first_head;
						courior->path->tail = second_tail;
						break;
					}
					case 1:
					{
						//2->1->3
						second_head->prior = NULL;
						second_tail->next = first_head;
						first_head->prior = second_tail;
						first_tail->next = third_head;
						third_head->prior = first_tail;
						third_tail->next = NULL;
						courior->path->head = second_head;
						courior->path->tail = third_tail;
						break;
					}
					case 2:
					{
						//2->3->1
						second_head->prior = NULL;
						second_tail->next = third_head;
						third_head->prior = second_tail;
						third_tail->next = first_head;
						first_head->prior = third_tail;
						first_tail->next = NULL;
						courior->path->head = second_head;
						courior->path->tail = first_tail;
						break;
					}
					case 3:
					{
						//3->1->2
						third_head->prior = NULL;
						third_tail->next = first_head;
						first_head->prior = third_tail;
						first_tail->next = second_head;
						second_head->prior = first_tail;
						second_tail->next = NULL;
						courior->path->head = third_head;
						courior->path->tail = second_tail;
						break;
					}
					case 4:
					{
						//3->2->1
						third_head->prior = NULL;
						third_tail->next = second_head;
						second_head->prior = third_tail;
						second_tail->next = first_head;
						first_head->prior = second_tail;
						first_tail->next = NULL;
						courior->path->head = third_head;
						courior->path->tail = first_tail;
						break;
					}
					default:
						break;
					}
					courior->path_update();
					if (courior->check_feasible_updated())
					{
						delta_cost = courior->cost - origin_cost;
						if (best_cost > delta_cost)
						{
							best_cost = delta_cost;
							best_route = courior;
							best_order[0] = *first_arc;
							best_order[1] = *second_arc;
							best_comb = combination;
						}
					}
					combination++;
				}
				first_head->prior = NULL;
				first_tail->next = second_head;
				second_head->prior = first_tail;
				second_tail->next = third_head;
				third_head->prior = second_tail;
				third_tail->next = NULL;
				courior->path->head = first_head;
				courior->path->tail = third_tail;
				courior->path_update();
			}
		}
	}
	if (best_comb == -1)
	{
		printf("two-opt cannot find non-tabu feasible move!\n");
		return;
	}
	PointOrder first_order = best_route->path->head;
	PointOrder second_order = best_route->path->head;
	while (first_order->visit != best_order[0])
	{
		first_order = first_order->next;
	}
	while (second_order->visit != best_order[1])
	{
		second_order = second_order->next;
	}
	PointOrder first_head = best_route->path->head,
			   first_tail = first_order,
			   second_head = first_order->next,
			   second_tail = second_order,
			   third_head = second_order->next,
			   third_tail = best_route->path->tail;
	//°Ñ²Ù×÷µÄ¶©µ¥¼ÓÈëdelete_list
	switch (best_comb)
	{
	case 0:
	{
		//1->3->2
		first_head->prior = NULL;
		first_tail->next = third_head;
		third_head->prior = first_tail;
		third_tail->next = second_head;
		second_head->prior = third_tail;
		second_tail->next = NULL;
		best_route->path->head = first_head;
		best_route->path->tail = second_tail;
		string order1_str = first_tail->package->id, order2_str = third_tail->package->id;
		string comb_str1 = order1_str + order2_str;
		string comb_str2 = order2_str + order1_str;
		Alns::two_opt_tabu[comb_str1] = TABU_TENURE;
		Alns::two_opt_tabu[comb_str2] = TABU_TENURE;
		Alns::tabu_touched_order[order1_str]++;
		Alns::tabu_touched_order[order2_str]++;
		break;
	}
	case 1:
	{
		//2->1->3
		second_head->prior = NULL;
		second_tail->next = first_head;
		first_head->prior = second_tail;
		first_tail->next = third_head;
		third_head->prior = first_tail;
		third_tail->next = NULL;
		best_route->path->head = second_head;
		best_route->path->tail = third_tail;
		string order1_str = second_tail->package->id, order2_str = first_tail->package->id;
		string comb_str1 = order1_str + order2_str;
		string comb_str2 = order2_str + order1_str;
		Alns::two_opt_tabu[comb_str1] = TABU_TENURE;
		Alns::two_opt_tabu[comb_str2] = TABU_TENURE;
		Alns::tabu_touched_order[order1_str]++;
		Alns::tabu_touched_order[order2_str]++;
		break;
	}
	case 2:
	{
		//2->3->1
		second_head->prior = NULL;
		second_tail->next = third_head;
		third_head->prior = second_tail;
		third_tail->next = first_head;
		first_head->prior = third_tail;
		first_tail->next = NULL;
		best_route->path->head = second_head;
		best_route->path->tail = first_tail;
		string order1_str = second_tail->package->id, order2_str = third_tail->package->id;
		string comb_str1 = order1_str + order2_str;
		string comb_str2 = order2_str + order1_str;
		Alns::two_opt_tabu[comb_str1] = TABU_TENURE;
		Alns::two_opt_tabu[comb_str2] = TABU_TENURE;
		Alns::tabu_touched_order[order1_str]++;
		Alns::tabu_touched_order[order2_str]++;
		break;
	}
	case 3:
	{
		//3->1->2
		third_head->prior = NULL;
		third_tail->next = first_head;
		first_head->prior = third_tail;
		first_tail->next = second_head;
		second_head->prior = first_tail;
		second_tail->next = NULL;
		best_route->path->head = third_head;
		best_route->path->tail = second_tail;
		string order1_str = third_tail->package->id, order2_str = first_tail->package->id;
		string comb_str1 = order1_str + order2_str;
		string comb_str2 = order2_str + order1_str;
		Alns::two_opt_tabu[comb_str1] = TABU_TENURE;
		Alns::two_opt_tabu[comb_str2] = TABU_TENURE;
		Alns::tabu_touched_order[order1_str]++;
		Alns::tabu_touched_order[order2_str]++;
		break;
	}
	case 4:
	{
		//3->2->1
		third_head->prior = NULL;
		third_tail->next = second_head;
		second_head->prior = third_tail;
		second_tail->next = first_head;
		first_head->prior = second_tail;
		first_tail->next = NULL;
		best_route->path->head = third_head;
		best_route->path->tail = first_tail;
		string order1_str = third_tail->package->id, order2_str = second_tail->package->id;
		string comb_str1 = order1_str + order2_str;
		string comb_str2 = order2_str + order1_str;
		Alns::two_opt_tabu[comb_str1] = TABU_TENURE;
		Alns::two_opt_tabu[comb_str2] = TABU_TENURE;
		Alns::tabu_touched_order[order1_str]++;
		Alns::tabu_touched_order[order2_str]++;
		break;
	}
	default:
		break;
	}
	best_route->path_update();
}

void Solution::reinsertion(vector<PointCourior> routes)
{
	int best_cost = MAXCOST;
	PointOrder best_origin = NULL, best_dest = NULL;
	Courior *src_courior = NULL, *best_courior = NULL;
	map<PointOrder, pair<PointOrder, PointOrder> > order_pos;
	for (vector<PointCourior>::iterator iter_courior = routes.begin(); iter_courior != routes.end(); ++iter_courior)
	{
		Courior *courior = *iter_courior;
		PointOrder order = courior->path->head;
		while (order != courior->path->tail)
		{
			PointOrder order_origin = NULL, order_dest = NULL;
			if (order->type || Alns::reinsert_tabu[order->package->id] > 0 || Alns::global_tabu[order->package->id] > 0)
			{
				order = order->next;
				continue;
			}
			else
			{
				order_origin = order, order_dest = order->next;
				while (order_dest->package != order_origin->package)
					order_dest = order_dest->next;
			}
			PointOrder origin_prior = order_origin->prior, origin_next = order_origin->next,
					   dest_prior = order_dest->prior, dest_next = order_dest->next;
			int path_old_cost = courior->cost;
			courior->path->remove_order(order_origin);
			courior->path->remove_order(order_dest);
			courior->path_update();
			int delta_cost1 = courior->cost - path_old_cost, bound = 0;
			for (vector<PointCourior>::iterator target_courior = routes.begin(); target_courior != routes.end(); ++target_courior)
			{
				Courior *insert_courior = *target_courior;
				if (PRUNING_LEVEL == 2)
				{
					bound = insert_courior->bound_calc(order_origin);
					if (best_cost < delta_cost1 + bound)
					{
						continue;
					}
				}
				insert_courior->order_min_pos(order_origin, order_dest, NULL, NULL, 0, 0, 1); //ÕÒµ½µ±Ç°Â·¾¶¶©µ¥×î¼ÑºÏ·¨Î»ÖÃ
				if ((order_dest->prior == NULL && order_dest->next == NULL) ||
					(order_origin->prior == NULL && order_origin->next == NULL))
					continue;
				int old_cost = insert_courior->cost;
				insert_courior->path->insert_order(order_dest);
				insert_courior->path->insert_order(order_origin);
				insert_courior->path_update();
				int delta_cost2 = insert_courior->cost - old_cost;
				if (best_cost > delta_cost1 + delta_cost2)
				{
					best_origin = order_origin;
					best_dest = order_dest;
					src_courior = courior;
					best_courior = insert_courior;
					best_cost = delta_cost1 + delta_cost2;
					order_pos[order_origin] = make_pair(order_origin->prior, order_origin->next);
					order_pos[order_dest] = make_pair(order_dest->prior, order_dest->next);
				}
				insert_courior->path->remove_order(order_origin);
				insert_courior->path->remove_order(order_dest);
				insert_courior->path_update();
			}
			order_origin->prior = origin_prior;
			order_origin->next = origin_next;
			order_dest->prior = dest_prior;
			order_dest->next = dest_next;
			courior->path->insert_order(order_dest);
			courior->path->insert_order(order_origin);
			courior->path_update();
			order = order->next;
		}
	}
	if (best_origin == NULL || best_dest == NULL)
	{
		printf("reinsertion cannot find non-tabu feasible move!\n");
		return;
	}
	Alns::tabu_touched_order[best_origin->package->id]++;
	Alns::reinsert_tabu[best_origin->package->id] = TABU_TENURE;
	src_courior->path->remove_order(best_dest);
	src_courior->path->remove_order(best_origin);
	src_courior->path_update();
	best_origin->prior = order_pos[best_origin].first;
	best_origin->next = order_pos[best_origin].second;
	best_dest->prior = order_pos[best_dest].first;
	best_dest->next = order_pos[best_dest].second;
	best_courior->path->insert_order(best_dest);
	best_courior->path->insert_order(best_origin);
	best_courior->path_update();
}

void Solution::swap(vector<PointCourior> routes)
{
	int best_cost = MAXCOST;
	PointOrder best_order1_origin = NULL, best_order1_dest = NULL, best_order2_origin = NULL, best_order2_dest = NULL;
	Courior *best_route1 = NULL, *best_route2 = NULL;
	for (vector<PointCourior>::iterator iter_route1 = routes.begin(); iter_route1 != routes.end(); ++iter_route1)
	{
		Courior *courior1 = *iter_route1;
		PointOrder order1_origin = courior1->path->head, order1_dest = NULL;
		int route1_old_cost = courior1->cost;
		while (order1_origin != NULL)
		{
			if (Alns::global_tabu[order1_origin->package->id] > 0)
			{
				order1_origin = order1_origin->next;
				continue;
			}
			if (order1_origin->type)
			{
				order1_origin = order1_origin->next;
				continue;
			}
			else
			{
				order1_dest = order1_origin->next;
				while (order1_dest->package->id != order1_origin->package->id)
					order1_dest = order1_dest->next;
			}
			for (vector<PointCourior>::iterator iter_route2 = routes.begin(); iter_route2 != routes.end(); ++iter_route2)
			{
				Courior *courior2 = *iter_route2;
				int route2_old_cost = courior2->cost;
				PointOrder order2_origin = courior2->path->head, order2_dest = NULL;
				while (order2_origin != NULL)
				{
					if (Alns::global_tabu[order2_origin->package->id] > 0)
					{
						order2_origin = order2_origin->next;
						continue;
					}
					string order1 = order1_origin->package->id, order2 = order2_origin->package->id;
					string str_comb = order1 + order2;
					if (order1_origin->package->id == order2_origin->package->id || order2_origin->type || Alns::swap_tabu[str_comb] > 0)
					{
						order2_origin = order2_origin->next;
						continue;
					}
					else
					{
						order2_dest = order2_origin->next;
						while (order2_dest->package->id != order2_origin->package->id)
							order2_dest = order2_dest->next;
					}
					Courior::order_swap(order1_origin, order2_origin);
					Courior::order_swap(order1_dest, order2_dest);
					courior1->path_update();
					courior2->path_update();
					if (courior1->check_feasible_updated() && courior2->check_feasible_updated())
					{
						int delta = courior1->cost + courior2->cost - (route1_old_cost + route2_old_cost);
						if (best_cost > delta)
						{
							best_cost = delta;
							best_order1_origin = order1_origin;
							best_order1_dest = order1_dest;
							best_order2_origin = order2_origin;
							best_order2_dest = order2_dest;
							best_route1 = courior1;
							best_route2 = courior2;
						}
					}
					Courior::order_swap(order1_origin, order2_origin);
					Courior::order_swap(order1_dest, order2_dest);
					courior1->path_update();
					courior2->path_update();
					order2_origin = order2_origin->next;
				}
			}
			order1_origin = order1_origin->next;
		}
	}
	if (best_route1 == NULL || best_route2 == NULL)
	{
		printf("swap cannot find non-tabu feasible move!\n");
		return;
	}
	Courior::order_swap(best_order1_origin, best_order2_origin);
	Courior::order_swap(best_order1_dest, best_order2_dest);
	best_route1->path_update();
	best_route2->path_update();
	string str1 = best_order1_origin->package->id, str2 = best_order2_origin->package->id;
	string str_comb1 = str1 + str2, str_comb2 = str2 + str1;
	Alns::swap_tabu[str_comb1] = TABU_TENURE;
	Alns::swap_tabu[str_comb2] = TABU_TENURE;
	Alns::tabu_touched_order[str1]++;
	Alns::tabu_touched_order[str2]++;
}

bool Solution::relatedness_descend_sort(const pair<string, double> &a, const pair<string, double> &b)
{
	return a.second > b.second;
}

void Solution::calcAttributes()
{
	vehicleNumber = 0;
	cost = 0;
	travelTime = 0;
	waitTime = 0;
	penalty = 0;
	idleTime = 0;
	for (int i = 0; i < COURIOR_NUM; i++)
	{
		PointOrder p = this->courior[i].path->head;
		if (p == NULL)
		{
			continue;
		}
		else
		{
			this->vehicleNumber++;
		}
		cost += this->courior[i].cost;
		waitTime += this->courior[i].wait_time;
		penalty += this->courior[i].penalty;
		idleTime += WORK_TIME - this->courior[i].path->tail->depart_time + this->courior[i].wait_time;
	}
	travelTime = cost - waitTime - penalty;
}
