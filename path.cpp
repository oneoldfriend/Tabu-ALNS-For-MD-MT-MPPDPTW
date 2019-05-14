#include"path.h"
#include<stdlib.h>


Path::Path() {
	head = NULL;
	tail = NULL;
}

void Path::insert_order(PointOrder order) {
	if (order->prior == NULL) {
		head = order;
	}
	else {
		order->prior->next = order;
	}
	if (order->next == NULL) {
		tail = order;
	}
	else {
		order->next->prior = order;
	}
}

void Path::delete_order(PointOrder order) {
	if (order == head)
		head = order->next;
	else
		order->prior->next = order->next;
	if (order == tail)
		tail = order->prior;
	else
		order->next->prior = order->prior;
}

void Path::remove_order(PointOrder order)
{
	if (order == head)
		head = order->next;
	else
		order->prior->next = order->next;
	if (order == tail)
		tail = order->prior;
	else
		order->next->prior = order->prior;
	order->prior = NULL;
	order->next = NULL;
}

void Path::new_insert(PointOrder order) {
	if (head == NULL) {
		head = order;
		tail = order;
	}
	else{
		tail->next = order;
		order->prior = tail;
		tail = order;
	}
}

void Path::append(PointOrder order) {
	if (head == NULL) {
		head = order;
		tail = order;
	}
	else {
		tail->next = order;
		order->prior = tail;
		tail = order;
	}
}