#include "courior.h"
#include "alns.h"
#include "fileIO.h"
#include "util.h"
#include "order.h"
#include <string.h>
#include <stdlib.h>
#include <cmath>

Courior::Courior()
{
	path = new Path();
	path->head = NULL;
	path->tail = NULL;
	cost = 0;
	wait_time = 0;
	penalty = 0;
}

void Courior::order_min_pos(Order *origin_order, Order *dest_order, //������Ҫ��ȡλ�õĶ���
							Order *best_origin, Order *best_dest,   //����Ѱ�Ҵ���λ��ʱ������λ����Ϣ
							int sec_flag,							//�Ƿ���ҪѰ�Ҵ���λ��(����operator��Ҫ)
							int noise_flag,
							int method_flag)
{ //�Ƿ���Ҫ����(����operator��Ҫ)
	int best_cost = 1000000;
	double best_Dst = 1000000.0;
	PointOrder temp_dest_order = new Order(dest_order->package, dest_order->type);
	PointOrder temp_origin_order = new Order(origin_order->package, origin_order->type);
	PointOrder origin_p = NULL;
	PointOrder dest_p = NULL;
	do
	{ //��������������λ��
		if (origin_p == NULL)
		{ //�״β���,���ǲ���λ��Ϊͷ�������
			origin_p = path->head;
			//���¼�����붩�����ǰ���
			temp_origin_order->next = origin_p;
			temp_origin_order->prior = NULL;
		}
		else
		{ //����λ�÷�ͷ���
			temp_origin_order->next = origin_p->next;
			temp_origin_order->prior = origin_p;
		}
		order_info_calc(temp_origin_order);
		dest_p = temp_origin_order;
		PointOrder temp = temp_origin_order->next;
		while (dest_p != NULL)
		{ //���������յ����λ��
			PointOrder temp_p = dest_p->next;
			temp_dest_order->next = dest_p->next;
			temp_dest_order->prior = dest_p;
			if (PRUNING_LEVEL == 0)
			{
				if (!sec_flag)
				{
					int bound = positionBound_calc(temp_origin_order, temp_dest_order);
					if (best_cost < bound)
					{
						dest_p = temp_p;
						continue;
					}
				}
			}
			if (strcmp(temp_dest_order->prior->package->id, temp_dest_order->package->id) == 0)
			{
				//������յ���������,���޸������ȷ��ָ���յ�
				temp_origin_order->next = temp_dest_order;
				temp_origin_order->next_dis = Util::get_distance(temp_origin_order->position, temp_dest_order->position);
			}
			else
			{
				//��������ָ��ԭ���
				temp_origin_order->next = temp;
				temp_origin_order->next_dis = Util::get_distance(temp_origin_order->position, temp->position);
			}
			if (sec_flag)
			{ //����ҪѰ�Ҵ���λ��,�򽫵�ǰλ�������������λ�ñȽ�
				if ((temp_origin_order->next == best_origin->next) && (temp_origin_order->prior == best_origin->prior) &&
					(temp_dest_order->prior == best_dest->prior) && (temp_dest_order->next == best_dest->next))
				{
					dest_p = temp_p;
					continue;
				}
			}
			order_info_calc(temp_dest_order);
			if (check_feasible(temp_origin_order, temp_dest_order))
			{ //��������
				if (best_cost > temp_origin_order->cost + temp_dest_order->cost + noise_flag * Alns::noise())
				{
					best_cost = temp_origin_order->cost + temp_dest_order->cost + noise_flag * Alns::noise();
					//��temp��������Ϣ���Ƹ�����Ķ���
					order_info_copy(origin_order, temp_origin_order);
					order_info_copy(dest_order, temp_dest_order);
					if (strcmp(dest_order->prior->package->id, dest_order->package->id) == 0)
					{
						//������յ���������,���޸����붩����λ����Ϣȷ��ǰ�����ȷ
						dest_order->prior = origin_order;
						origin_order->next = dest_order;
					}
				}
			}
			dest_p = temp_p;
		}
		if (origin_p == NULL)
			break;
		origin_p = origin_p->next;
	} while (origin_p != NULL);
	delete temp_dest_order;
	delete temp_origin_order;
}

void Courior::insert_update(PointOrder order)
{
	if (order->next != NULL)
	{ //�����ں�������
		//����delta
		int delta = order->next->dep_calc(order, 0) - order->next->depart_time;
		PointOrder p = order->next;
		if (order->type == 0)
		{ //�����붩��Ϊ���,����²��붩��֮��Ķ����ĵ�ǰ���غ��뿪ʱ��
			while (strcmp(p->package->id, order->package->id) != 0)
			{
				p->current_weight += order->package->weight;
				if (Order::check_order(p->package->id))
				{
					p->depart_time += delta;
				}
				else
				{
					if (p->type)
					{
						p->depart_time += delta;
					}
					else
					{
						//���oto��������ʱ�䴰�������������delta������Ӧ�޸�
						int old_dep = p->depart_time;
						p->depart_time = p->dep_calc(p->prior, 0);
						delta = p->depart_time - old_dep;
					}
				}
				p = p->next;
			}
		}
		while ((p != NULL))
		{ //���¶Ժ����������뿪ʱ���Ӱ��
			if (Order::check_order(p->package->id))
			{
				p->depart_time += delta;
			}
			else
			{
				if (p->type)
				{
					p->depart_time += delta;
				}
				else
				{
					//���oto��������ʱ�䴰�������������delta������Ӧ�޸�
					int old_dep = p->depart_time;
					p->depart_time = p->dep_calc(p->prior, 0);
					delta = p->depart_time - old_dep;
				}
			}
			p = p->next;
		}
	}
}

bool Courior::check_feasible(PointOrder origin, PointOrder dest)
{
	//��ʼ�����붩���Ժ���������Ӱ��delta
	int delta = 0, penalty = 0, tail_deptime = 0;
	origin->cost = 0;
	if (origin->next != NULL)
	{
		delta = origin->next->dep_calc(origin, 0) - origin->next->depart_time;
	}
	PointOrder p = origin->next;
	//����������
	if (origin->current_weight > CAPACITY)
	{
		return false;
	}
	if (origin->depart_time > origin->package->end_time)
	{
		return false;
	}
	if (!Order::check_order(origin->package->id))
	{
		if (origin->depart_time > origin->package->start_time)
		{
			if (ALLOW_START_LATENESS)
			{
				penalty += 5 * (origin->depart_time - origin->package->start_time);
			}
			else
			{
				return false;
			}
		}
	}
	//���㲢������������󵽲����յ�ǰ�Ķ������������뿪ʱ��
	while ((p != dest->next) && (p != dest))
	{
		//����������������
		if (p->current_weight + origin->package->weight > CAPACITY)
		{
			return false;
		}
		int new_dep = p->depart_time;
		//��������������뿪ʱ��
		if (Order::check_order(p->package->id))
		{
			new_dep += delta;
		}
		else
		{
			//ȷ������oto�����뿪ʱ������ʱ�䴰
			int old_dep = p->depart_time;
			if (p->type)
			{
				new_dep += delta;
				//��鲢������붩�����Ƿ��ú�����oto�������յ�����ͷ�
				if (new_dep - p->package->service_time() > p->package->end_time)
				{
					if (ALLOW_END_LATENESS)
					{
						penalty += 5 * (new_dep - p->package->service_time() - p->package->end_time);
					}
					else
					{
						return false;
					}
				}
			}
			else
			{
				//��Ϊû��ʵ�ʲ��붩��,�ڼ���p���뿪ʱ��ǰ��Ҫȷ��p��ǰ����ȷ
				if (p == origin->next)
				{
					new_dep += delta;
				}
				else
				{
					new_dep = p->dep_calc(p->prior, delta);
				}
				//��鲢������붩�����Ƿ��ú�����oto�������������ͷ�
				if (new_dep > p->package->start_time)
				{
					if (ALLOW_START_LATENESS)
					{
						penalty += 5 * (new_dep - p->package->start_time);
					}
					else
					{
						return false;
					}
				}
			}
			delta = new_dep - old_dep;
		}
		//�������������뿪ʱ��
		if (new_dep > p->package->end_time)
		{
			return false;
		}
		p = p->next;
	}
	dest->Dst = dest->Dst_calc(dest->prior_dis, dest->depart_time + delta, 0.5, 0.5);
	//����յ���뿪ʱ��
	if (dest->depart_time + delta - dest->package->service_time() > dest->package->end_time)
	{
		if (ALLOW_END_LATENESS)
		{
			penalty += 5 * (dest->depart_time + delta - dest->package->service_time() - dest->package->end_time);
		}
		else
		{
			return false;
		}
	}
	if (dest->next != NULL)
	{
		//��������յ��Ժ���������Ӱ��delta������delta
		delta = dest->next->dep_calc(dest, delta) - dest->next->depart_time;
	}
	else
	{
		//���ΪNULL,��destΪ·��β�ڵ�
		tail_deptime = dest->depart_time + delta;
	}
	p = dest->next;
	//���㲢����������յ��ĺ����������뿪ʱ��
	while (p != NULL)
	{
		int new_dep = p->depart_time;
		//��������������뿪ʱ��
		if (Order::check_order(p->package->id))
		{
			new_dep += delta;
		}
		else
		{
			//ȷ������oto��������ʱ������ʱ�䴰
			int old_dep = p->depart_time;
			//��Ϊû��ʵ�ʲ��붩��,�ڼ���p���뿪ʱ��ǰ��Ҫȷ��p��ǰ����ȷ
			if (p->type)
			{
				new_dep += delta;
				//��鲢������붩�����Ƿ��ú�����oto�������յ�����ͷ�
				if (new_dep - p->package->service_time() > p->package->end_time)
				{
					if (ALLOW_END_LATENESS)
					{
						penalty += 5 * (new_dep - p->package->service_time() - p->package->end_time);
					}
					else
					{
						return false;
					}
				}
			}
			else
			{
				//��Ϊû��ʵ�ʲ��붩��,�ڼ���p���뿪ʱ��ǰ��Ҫȷ��p��ǰ����ȷ
				if (p == dest->next)
				{
					new_dep += delta;
				}
				else
				{
					new_dep = p->dep_calc(p->prior, delta);
				}
				//��鲢������붩�����Ƿ��ú�����oto�������������ͷ�
				if (new_dep > p->package->start_time)
				{
					if (ALLOW_START_LATENESS)
					{
						penalty += 5 * (new_dep - p->package->start_time);
					}
					else
					{
						return false;
					}
				}
			}
			delta = new_dep - old_dep;
		}
		if (new_dep > p->package->end_time)
		{
			return false;
		}
		if (p->next == NULL)
		{
			//����ǰ�ڵ�Ϊβ�ڵ�,���¼���뿪ʱ��
			tail_deptime = new_dep;
		}
		p = p->next;
	}
	//����originǰ��penalty
	p = path->head;
	while ((p != origin->next) && (p != dest->next))
	{
		if (!Order::check_order(p->package->id))
		{
			if (p->type)
			{
				if (p->depart_time - p->package->service_time() > p->package->end_time)
				{
					penalty += 5 * (p->depart_time - p->package->service_time() - p->package->end_time);
				}
			}
			else
			{
				if (p->depart_time > p->package->start_time)
				{
					penalty += 5 * (p->depart_time - p->package->start_time);
				}
			}
		}
		p = p->next;
	}
	//������붩����cost���ڲ�����cost��ȥ����ǰ��cost
	dest->cost = tail_deptime + penalty - this->get_cost();
	return true;
}

int Courior::bound_calc(PointOrder order)
{
	if (path->head == NULL)
	{
		return 0;
	}
	double o_min = MAX_DS, d_min = MAX_DS;
	int wt = 0, bound = 0;
	PointOrder p = path->head;
	while (p != NULL)
	{
		double t_o = Util::get_distance(order->package->origin, p->position), t_d = Util::get_distance(order->package->dest, p->position);
		if (o_min > t_o)
			o_min = t_o;
		if (d_min > t_d)
			d_min = t_d;
		wt += p->wait_time;
		p = p->next;
	}
	return bound = Util::get_travel_time(o_min + d_min) - wt;
}

int Courior::positionBound_calc(PointOrder origin, PointOrder dest)
{
	double o_i, o_j, t_ij, d_m, d_n, t_mn;
	if (origin->prior != NULL && origin->next != NULL)
	{
		o_i = Util::get_distance(origin->position, origin->prior->position);
		o_j = Util::get_distance(origin->position, origin->next->position);
		t_ij = Util::get_distance(origin->next->position, origin->prior->position);
	}
	else
	{
		t_ij = 0;
		if (origin->prior == NULL)
		{
			o_i = 0;
		}
		else
		{
			o_i = Util::get_distance(origin->position, origin->prior->position);
		}
		if (origin->next == NULL)
		{
			o_j = 0;
		}
		else
		{
			o_j = Util::get_distance(origin->position, origin->next->position);
		}
	}
	if (dest->prior != NULL && dest->next != NULL)
	{
		d_m = Util::get_distance(dest->position, dest->prior->position);
		d_n = Util::get_distance(dest->position, dest->next->position);
		t_mn = Util::get_distance(dest->next->position, dest->prior->position);
	}
	else
	{
		d_m = Util::get_distance(dest->position, dest->prior->position);
		d_n = 0;
		t_mn = 0;
	}
	int delta_o = Util::get_travel_time(o_i + o_j - t_ij), delta_d = Util::get_travel_time(d_m + d_n - t_mn);
	return delta_o + delta_d;
}

int Courior::get_cost()
{
	if (path->head == NULL)
		return 0;
	int cost = 0, penalty = 0, wait_time = 0;
	PointOrder p = path->head;
	while (p != NULL)
	{
		//��鶩������
		if (!Order::check_order(p->package->id))
		{
			//��Ϊoto����
			if (p->type)
			{ //��������յ��Ƿ����ʱ�䴰penalty
				if (p->depart_time - p->package->service_time() > p->package->end_time)
					penalty += 5 * (p->depart_time - p->package->service_time() - p->package->end_time);
			}
			else
			{
				penalty += 5 * (p->depart_time - p->package->start_time);
			}
		}
		wait_time += p->wait_time;
		p = p->next;
	}
	cost += path->tail->depart_time + penalty;
	this->cost = cost;
	this->penalty = penalty;
	this->wait_time = wait_time;
	return cost;
}

void Courior::delete_path()
{
	PointOrder p = path->head;
	if (p == NULL)
	{
		return;
	}
	while (p != NULL)
	{
		PointOrder temp = p->next;
		delete p;
		p = temp;
	}
}

void Courior::order_info_calc(PointOrder order)
{
	if (order->prior == NULL)
	{
		//����Ϊͷ���
		if (Order::check_order(order->package->id))
		{
			order->prior_dis = 0;
		}
		else
		{
			//��oto����Ѱ�������������
			if (order->type)
			{
				order->prior_dis = 0;
			}
			else
			{
				double dis = Util::get_distance(order->position, FileIO::netpoint[1]);
				for (int i = 2; i < NET_NUM; i++)
				{
					double temp_dis = Util::get_distance(order->position, FileIO::netpoint[i]);
					if (dis > temp_dis)
						dis = temp_dis;
				}
				order->prior_dis = dis;
			}
		}
		order->current_weight = order->package->weight;
	}
	else
	{
		//������ǰ�̶����ľ���͵�ǰ����
		order->prior_dis = Util::get_distance(order->prior->position, order->position);
		if (order->type == 0)
		{ //����Ϊ���
			order->current_weight = order->prior->current_weight + order->package->weight;
		}
		else
		{ //����Ϊ�յ�
			if (strcmp(order->prior->package->id, order->package->id) == 0)
			{
				order->current_weight = order->prior->current_weight - order->package->weight;
			}
			else
			{
				order->current_weight = order->prior->current_weight;
			}
		}
		//�������̵ľ���
		if (order->next == NULL)
		{
			order->next_dis = 0;
		}
		else
		{
			order->next_dis = Util::get_distance(order->position, order->next->position);
		}
	}
	order->depart_time = order->dep_calc(order->prior, 0);
	order->Dst = order->Dst_calc(order->prior_dis, order->depart_time, 0.5, 0.5);
}

void Courior::order_info_copy(Order *dest, Order *source)
{
	dest->package = source->package;
	dest->current_weight = source->current_weight;
	dest->depart_time = source->depart_time;
	dest->next = source->next;
	dest->wait_time = source->wait_time;
	dest->Dst = source->Dst;
	dest->next_dis = source->next_dis;
	dest->position = source->position;
	dest->prior = source->prior;
	dest->prior_dis = source->prior_dis;
	dest->type = source->type;
	dest->cost = source->cost;
}

void Courior::path_update()
{
	PointOrder p = path->head;
	while (p != NULL)
	{
		if (p->prior == NULL)
		{ //��ǰ��Ϊ��
			//����deptime�͵�ǰ����
			double closest_depot_dis = MAX_DS;
			for (int i = 1; i <= DEPOT_NUM; i++)
			{
				double dis = Util::get_distance(FileIO::netpoint[i], p->position);
				if (dis < closest_depot_dis)
					closest_depot_dis = dis;
			}
			p->depart_time = Util::get_travel_time(closest_depot_dis);
			if (!Order::check_order(p->package->id))
			{
				if (p->depart_time < p->package->start_time)
				{
					p->wait_time = p->package->start_time - p->depart_time;
					p->depart_time = p->package->start_time;
				}
				else
					p->wait_time = 0;
			}
			else
				p->wait_time = 0;
			p->current_weight = p->package->weight;
			p->visit = 0;
			p->prior_dis = 0;
		}
		else
		{ //ǰ�̷ǿ�
			//������ǰ�̵ľ���
			double dis = Util::get_distance(p->position, p->prior->position);
			p->prior_dis = dis;
			p->prior->next_dis = dis;
			//����ǰ��deptime���㵱ǰdeptime
			p->depart_time = p->prior->depart_time + Util::get_travel_time(dis);
			if (p->type == 0)
			{
				//������ʱ�䴰����������
				if (p->depart_time < p->package->start_time)
				{
					p->wait_time = p->package->start_time - p->depart_time;
					p->depart_time = p->package->start_time;
				}
				else
					p->wait_time = 0;
				p->current_weight = p->prior->current_weight + p->package->weight;
			}
			else
			{
				//�����յ㴦��ʱ�䲢��������
				p->depart_time += p->package->service_time();
				p->current_weight = p->prior->current_weight - p->package->weight;
			}
			p->visit = p->prior->visit + 1;
		}
		p->Dst = p->Dst_calc(p->prior_dis, p->depart_time, 0.5, 0.5);
		if (p->next == NULL)
		{
			this->path->tail = p;
			p->next_dis = 0;
		}
		p = p->next;
	}
	this->cost = this->get_cost();
}

bool Courior::check_feasible_updated()
{
	PointOrder p = this->path->head;
	if (this->path->tail->depart_time > WORK_TIME)
	{
		return false;
	}
	while (p != NULL)
	{
		if (p->current_weight > CAPACITY)
			return false;
		if (p->depart_time > p->package->end_time)
			return false;
		p = p->next;
	}
	return true;
}

void Courior::order_swap(Order *dest, Order *source)
{
	if (dest == NULL || source == NULL)
		return;
	PointOrder exchange_order = new Order(source->package, 0);
	exchange_order->package = source->package; //����������Ϣ
	exchange_order->type = source->type;
	exchange_order->position = source->position;
	source->package = dest->package;
	source->type = dest->type;
	source->position = dest->position;
	dest->package = exchange_order->package;
	dest->type = exchange_order->type;
	dest->position = exchange_order->position;
	//delete exchange_order->package;
	delete exchange_order;
}

double Courior::get_dis()
{
	PointOrder p = this->path->head->next;
	double dis_cum = 0;
	int order_count = 0;
	while (p != NULL)
	{
		if (p->type)
			order_count++;
		dis_cum += Util::get_distance(p->prior->position, p->position);
		p = p->next;
	}
	return dis_cum / order_count;
}

double Courior::get_load()
{
	PointOrder p = this->path->head->next;
	double empty_load_cum = 0, dis_cum = 0;
	int order_count = 0;
	while (p != NULL)
	{
		if (p->type)
			order_count++;
		empty_load_cum += CAPACITY - p->current_weight;
		dis_cum += Util::get_distance(p->prior->position, p->position);
		p = p->next;
	}
	return empty_load_cum / order_count;
}
