#include <iostream>
#include "flag.h"

void Flag_1::add_flag()
                {
                    flag=flag+1;
                }

void Flag_1::zero_flag()
                {
                    flag = 0;
                }

void Flag_1::show_flag()
                {
                    std::cout << "flag" << flag<<std::endl;
                }


int Flag_1::judge_flag()
                {
                    if(flag == 20)
                    return 0;
                    else
                    return 1;
                }

Flag_1 flag_xx;

