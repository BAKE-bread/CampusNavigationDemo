#include "map_ope.cpp"

int main()
{
    system("chcp 936");
    campus_map_init(campus_map_str_int);
    campus_map_init(campus_map_int_str);
    Graph<std::string> g_str;
    g_str = campus_map_create_string();
    Graph<int> g_int;
    g_int = campus_map_create_int();

    while (true)
    {
        std::cout << "\t********************--------------------********************" << std::endl;
        std::cout << "\t�����������ţ�" << std::endl;
        std::cout << "\t1����ʾУ԰��ͼ�ĵص�͵�·��Ϣ��2�����ӡ�ɾ���ص���Ϣ��3�����ӡ�ɾ����·��Ϣ" << std::endl;
        std::cout << "\t4����ѯ���������ص������·����5����ѯ����n���ص�����·��" << std::endl;
        std::cout << "\t6����ѯ���ŵ���·���±ؾ��ؼ��ص�����·����7����ѯ�ص�����˳�����޵����·��" << std::endl;
        std::cout << "\t0���س�ʼ����ͼ���������˳�����" << std::endl;
        std::cout << "\t--------------------********************--------------------" << std::endl;
        std::cout << std::endl;
        std::string ope;
        std::cin >> ope;
        if (ope == "1")
            operation1(g_str);
        else if (ope == "2")
            operation2(g_str, g_int);
        else if (ope == "3")
            operation3(g_str, g_int);
        else if (ope == "4")
            operation4(g_str, g_int);
        else if (ope == "5")
            operation5(g_str, g_int);
        else if (ope == "6")
            operation6(g_str, g_int);
        else if (ope == "7")
            operation7(g_str, g_int);
        else if (ope == "0")
        {
            campus_map_init(campus_map_str_int);
            campus_map_init(campus_map_int_str);
            g_str = campus_map_create_string();
            g_int = campus_map_create_int();
            std::cout << "�س�ʼ����ͼ���" << std::endl;
            std::cout << std::endl;
        }
        else
            break;
        /*
        for (auto it = campus_map_int_str.begin(); it != campus_map_int_str.end(); it++)
            std::cout << it->first << ":" << it->second << std::endl;
        */
    }
    system("pause");
    return 0;
}