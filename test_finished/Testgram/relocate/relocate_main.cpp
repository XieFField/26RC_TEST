/**
 * @brief �Ӿ��ض�λ���Գ���
 * @author Wu Jia
 */
#include "relocate_test.h"
#include <iostream>
#include <string>
#include <sstream>

float hoopX = 0.0f;  
float hoopY = 0.0f; 
Reposition reposition_test(hoopX, hoopY);

int main(void)
{

    std::string line;
    std::cout<<"���������Ϊ(0,0)"<<std::endl;
    while(true)
    {
        std::cout<<"��ǰ��������ľ���(��)�Լ�����ƫת�Ƕ�(��),����exit�˳�:"<<std::endl<<
        "(tips:����˳ʱ��ת�Ƕ�����,��ʱ��ת�Ƕȼ�С,����������ʱ����0��)"
        <<std::endl;
        
        std::getline(std::cin, line);
        if(line == "exit"|| line == "EXIT")
            break;
        std::istringstream iss(line);
        float distance, angle;

        if(!(iss >> distance >> angle)) {
            std::cout << "�����ʽ�������������룡" << std::endl;
            continue;
        }

        // ������ھ���ͽǶȵ��ض�λ����
        position2D newPos = reposition_test.Reposition_Calc_VPC(distance, angle);

        std::cout << "�ض�λ�������: (" << newPos.x << ", " << newPos.y << ")" << std::endl;
    }
    return 0;
}