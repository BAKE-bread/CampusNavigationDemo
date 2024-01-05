#pragma once
#include "graph.hpp"
#include <algorithm>

int BASE = 14;

// typedef std::pair<std::string, int> pos;

std::map<std::string, int> campus_map_str_int;
std::map<int, std::string> campus_map_int_str;

void campus_map_init(std::map<std::string, int> &campus_map);
void campus_map_init(std::map<int, std::string> &campus_map);
Graph<int> campus_map_create_int();
Graph<std::string> campus_map_create_string();

void campus_map_init(std::map<std::string, int> &campus_map)
{
    campus_map["A"] = 1;
    campus_map["B"] = 2;
    campus_map["C"] = 3;
    campus_map["D"] = 4;
    campus_map["E"] = 5;
    campus_map["F"] = 6;
    campus_map["G"] = 7;
    campus_map["H"] = 8;
    campus_map["I"] = 9;
    campus_map["J"] = 10;
    campus_map["K"] = 11;
    campus_map["L"] = 12;
    campus_map["M"] = 13;
    campus_map["N"] = 14;
}
void campus_map_init(std::map<int, std::string> &campus_map)
{
    campus_map[1] = "A";
    campus_map[2] = "B";
    campus_map[3] = "C";
    campus_map[4] = "D";
    campus_map[5] = "E";
    campus_map[6] = "F";
    campus_map[7] = "G";
    campus_map[8] = "H";
    campus_map[9] = "I";
    campus_map[10] = "J";
    campus_map[11] = "K";
    campus_map[12] = "L";
    campus_map[13] = "M";
    campus_map[14] = "N";
}

Graph<int> campus_map_create_int()
{
    Graph<int> g;
    for (int i = 1; i <= BASE; i++)
    {
        g.add_vertex(i);
    }

    // 一次加边就够了
    g.add_edge(1, 4, 152);
    g.add_edge(4, 1, 152);
    g.add_edge(2, 3, 96);
    g.add_edge(3, 2, 96);
    g.add_edge(3, 4, 532);
    g.add_edge(4, 3, 532);
    g.add_edge(4, 5, 251);
    g.add_edge(5, 4, 251);

    g.add_edge(2, 6, 312);
    g.add_edge(6, 2, 312);
    g.add_edge(3, 6, 281);
    g.add_edge(6, 3, 281);
    g.add_edge(3, 7, 318);
    g.add_edge(7, 3, 318);
    g.add_edge(3, 8, 323);
    g.add_edge(8, 3, 323);
    g.add_edge(4, 8, 98);
    g.add_edge(8, 4, 98);
    g.add_edge(4, 9, 143);
    g.add_edge(9, 4, 143);
    g.add_edge(5, 10, 85);
    g.add_edge(10, 5, 85);

    g.add_edge(6, 7, 145);
    g.add_edge(7, 6, 145);
    g.add_edge(7, 8, 226);
    g.add_edge(8, 7, 226);
    g.add_edge(8, 9, 112);
    g.add_edge(9, 8, 112);
    g.add_edge(9, 10, 96);
    g.add_edge(10, 9, 96);
    g.add_edge(10, 11, 261);
    g.add_edge(11, 10, 261);

    g.add_edge(6, 8, 442);
    g.add_edge(8, 6, 442);
    g.add_edge(8, 12, 437);
    g.add_edge(12, 8, 437);
    g.add_edge(9, 12, 346);
    g.add_edge(12, 9, 346);
    g.add_edge(9, 13, 412);
    g.add_edge(13, 9, 412);
    g.add_edge(10, 13, 348);
    g.add_edge(13, 10, 348);

    g.add_edge(6, 12, 541);
    g.add_edge(12, 6, 541);
    g.add_edge(12, 13, 162);
    g.add_edge(13, 12, 162);
    g.add_edge(11, 13, 187);
    g.add_edge(13, 11, 187);
    g.add_edge(12, 14, 47);
    g.add_edge(14, 12, 47);
    g.add_edge(13, 14, 198);
    g.add_edge(14, 13, 198);

    return g;
}

Graph<std::string> campus_map_create_string()
{
    Graph<std::string> g;

    g.add_vertex("A");
    g.add_vertex("B");
    g.add_vertex("C");
    g.add_vertex("D");
    g.add_vertex("E");
    g.add_vertex("F");
    g.add_vertex("G");
    g.add_vertex("H");
    g.add_vertex("I");
    g.add_vertex("J");
    g.add_vertex("K");
    g.add_vertex("L");
    g.add_vertex("M");
    g.add_vertex("N");

    g.add_edge("A", "D", 152);

    g.add_edge("B", "C", 96);
    g.add_edge("C", "D", 532);
    g.add_edge("D", "E", 251);

    g.add_edge("B", "F", 312);
    g.add_edge("C", "F", 281);
    g.add_edge("C", "G", 318);
    g.add_edge("C", "H", 323);
    g.add_edge("D", "H", 98);
    g.add_edge("D", "I", 143);
    g.add_edge("E", "J", 85);

    g.add_edge("F", "G", 145);
    g.add_edge("G", "H", 226);
    g.add_edge("H", "I", 112);
    g.add_edge("I", "J", 96);
    g.add_edge("J", "K", 261);

    g.add_edge("F", "H", 442);
    g.add_edge("H", "L", 437);
    g.add_edge("I", "L", 346);
    g.add_edge("I", "M", 412);
    g.add_edge("J", "M", 348);

    g.add_edge("F", "L", 541);
    g.add_edge("L", "M", 162);
    g.add_edge("K", "M", 187);
    g.add_edge("L", "N", 47);
    g.add_edge("M", "N", 198);

    /*
    std::cout << "Number of vertices: " << g.num_vertices() << std::endl;
    std::cout << "Number of edges: " << g.num_edges() << std::endl;
    std::cout << "Largest degree: " << g.largest_degree() << std::endl;
    std::cout << "Vertices: ";
    for (auto vertex : g.get_vertices())
        std::cout << vertex << " ";
    std::cout << std::endl;
    */
    return g;
}
