#include<iostream>
#include<time.h>
#include"package.h"
#include"alns.h"
#include"fileIO.h"
#include"util.h"
using namespace std;


int main(int argc, char** argv){
	int input_eco_num = atoi(argv[1]);
	int input_oto_num = atoi(argv[2]);
	int input_run_num = atoi(argv[3]);
	srand(time(NULL));
	Alns::alns(input_eco_num, input_oto_num, input_run_num);
	system("pause");
}