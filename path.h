#ifndef _PATH_H_
#define _PATH_H_
#include"order.h"

class Path {
public:
	PointOrder head;
	PointOrder tail;
	Path();
	void new_insert(PointOrder order);
	void insert_order(PointOrder order);
	void delete_order (PointOrder order);
	void remove_order(PointOrder order);
	void append(PointOrder order);
};

#endif