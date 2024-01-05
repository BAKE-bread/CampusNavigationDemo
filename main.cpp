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
        std::cout << "\t请输入操作编号：" << std::endl;
        std::cout << "\t1：显示校园地图的地点和道路信息；2：增加、删除地点信息；3：增加、删除道路信息" << std::endl;
        std::cout << "\t4：查询任意两个地点间的最短路径；5：查询经过n个地点的最短路径" << std::endl;
        std::cout << "\t6：查询最优导览路径下必经关键地点的最短路径；7：查询地点游览顺序受限的最短路径" << std::endl;
        std::cout << "\t0：重初始化地图；其他：退出程序" << std::endl;
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
            std::cout << "重初始化地图完成" << std::endl;
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