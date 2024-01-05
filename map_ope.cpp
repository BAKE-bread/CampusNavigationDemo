#include "create_map.cpp"

void operation1(Graph<std::string> &g_str)
{
    g_str.show();
    std::cout << std::endl;
}

void operation2(Graph<std::string> &g_str, Graph<int> &g_int)
{

    std::cout << "按以下格式输入：（insert 地点；delete 地点；其他字符退出本操作）" << std::endl;

    std::string ope, tmp;
    std::cin >> ope >> tmp;

    if (ope == "insert")
    {
        if (!g_str.contains(tmp))
        {
            int cur_num_vertices = ++BASE;
            campus_map_str_int.insert({tmp, cur_num_vertices});
            campus_map_int_str[cur_num_vertices] = tmp;
            g_str.add_vertex(tmp);
            g_int.add_vertex(cur_num_vertices);
        }
        g_str.show();
    }
    else if (ope == "delete")
    {
        if (g_str.contains(tmp))
        {
            int vertex_number = campus_map_str_int[tmp];
            campus_map_str_int.erase(tmp);
            campus_map_int_str.erase(vertex_number);
            g_str.remove_vertex(tmp);
            g_int.remove_vertex(vertex_number);
            g_str.show();
        }
        else
        {
            std::cout << "不存在该地点，删除失败！" << std::endl;
        }
        std::cout << std::endl;
    }
}

void operation3(Graph<std::string> &g_str, Graph<int> &g_int)
{

    std::cout << "按以下格式输入：（insert 地点1 地点2 路径长度；delete 地点1 地点2；其他字符退出本操作）" << std::endl;

    std::string ope, tmp1, tmp2;
    int dis;
    std::cin >> ope;

    if (ope == "insert")
    {
        std::cin >> tmp1 >> tmp2 >> dis;
        // std::cout << (campus_map_str_int.find(tmp1) != campus_map_str_int.end()) << std::endl;
        // std::cout << (campus_map_str_int.find(tmp2) != campus_map_str_int.end()) << std::endl;
        if (!g_str.contains(tmp1))
        {
            int cur_num_vertices = ++BASE;
            campus_map_str_int.insert({tmp1, cur_num_vertices});
            campus_map_int_str[cur_num_vertices] = tmp1;
        }
        else if (!g_str.contains(tmp2))
        {
            int cur_num_vertices = ++BASE;
            campus_map_str_int.insert({tmp2, cur_num_vertices});
            campus_map_int_str[cur_num_vertices] = tmp2;
        }
        g_str.add_edge(tmp1, tmp2, dis);
        g_int.add_edge(campus_map_str_int[tmp1], campus_map_str_int[tmp2], dis);
        g_str.show();
    }
    else if (ope == "delete")
    {
        std::cin >> tmp1 >> tmp2;
        if (!g_str.contains(tmp1) || !g_str.contains(tmp2) || g_str.adj[tmp1].find(tmp2) == g_str.adj[tmp1].end())
        {
            std::cout << "不存在该路径，删除失败！" << std::endl;
        }
        else
        {
            if (g_str.adj[tmp1].size() > 1 && g_str.adj[tmp2].size() > 1)
            {
                g_str.remove_edge(tmp1, tmp2);
                g_int.remove_edge(campus_map_str_int[tmp1], campus_map_str_int[tmp2]);
                g_str.show();
            }
            else
            {
                std::cout << "删除后地图存在孤立点，删除失败！" << std::endl;
            }
        }
    }
    std::cout << std::endl;
}

void operation4(Graph<std::string> &g_str, Graph<int> &g_int)
{
    std::cout << "请输入源地点和目标地点：" << std::endl;
    std::string src, dest;
    std::cin >> src >> dest;

    std::map<std::string, std::pair<int, std::vector<std::string>>> shortest_map;
    shortest_map = g_str.dijkstra(src);
    /*
    std::cout << "最短路径：" << std::endl;
    for (auto u : shortest_map)
    {
        if (u.first == dest)
        {
            std::cout << src << " ";
            for (auto v : u.second.second)
            {
                std::cout << v << " ";
            }
            std::cout << u.second.first << std::endl;
        }
    }
    std::cout << std::endl;
    */
    std::pair<int, std::vector<std::string>> res;
    res = g_str.dijkstra(src, dest);
    std::cout << "最短路径：" << std::endl;
    for (auto v : res.second)
    {
        std::cout << v << " ";
    }
    std::cout << res.first << std::endl;
    std::cout << std::endl;
}

void dfs_op5(std::vector<std::vector<int>> &graph, int src, int dest, std::vector<int> &path, std::vector<bool> &visited, int path_len, int n, std::vector<int> &shortest_path, int &shortest_dis)
{
    visited[src] = true;
    path.push_back(src);

    if (src == dest && path.size() == n)
    {
        if (path_len < shortest_dis)
        {
            shortest_path = path;
            shortest_dis = path_len;
        }
    }
    else
    {
        for (int i = 0; i < graph[src].size(); i++)
        {
            if (!visited[i] && graph[src][i] != 0)
                dfs_op5(graph, i, dest, path, visited, path_len + graph[src][i], n, shortest_path, shortest_dis);
        }
    }

    path.pop_back();
    visited[src] = false;
}
void operation5(Graph<std::string> &g_str, Graph<int> &g_int)
{
    int MAX = campus_map_str_int.size() + 1;
    std::vector<std::vector<int>> G(MAX, std::vector<int>(MAX, 0)); // 存储各顶点间的距离
    g_int.get_adj_matrix(0);
    G = g_int.adj_matrix;

    // 读入源点和目标点
    std::cout << "请输入源地点和目标地点，以及必须经过的顶点数：" << std::endl;
    std::string tmp1, tmp2;
    std::cin >> tmp1 >> tmp2;
    int src = campus_map_str_int[tmp1], dest = campus_map_str_int[tmp2];

    // 必须经过的顶点数
    int n;
    std::cin >> n;

    std::vector<int> path;
    std::vector<bool> visited(MAX, false);
    int path_len = 0;
    std::vector<int> shortest_path;
    int shortest_dis = INF;
    dfs_op5(G, src, dest, path, visited, path_len, n, shortest_path, shortest_dis);

    if (shortest_dis != INF)
    {
        std::cout << "最短路径：" << std::endl;
        for (auto node : shortest_path)
        {
            std::cout << campus_map_int_str[node] << " ";
        }
        std::cout << shortest_dis << std::endl;
    }
    else
    {
        std::cout << "这两点之间无法通过" << n << "个顶点到达！" << std::endl;
    }
    std::cout << std::endl;
}

// 获取点a到点b最短路的中间路径（路径中不含a，b）
void get_path(int a, int b, std::vector<std::vector<int>> &path, std::vector<int> &anspath)
{
    if (path[a][b] == -1)
    {
        return;
    }
    else
    {
        int k = path[a][b];
        get_path(a, k, path, anspath);
        anspath.push_back(k);
        get_path(k, b, path, anspath);
    }
}
// 返回点src沿着当前must数组中指定点的顺序走到点dest的路径长度
int get_dis(int src, int dest, std::vector<std::vector<int>> &dist, std::vector<int> &must)
{
    int distance = dist[src][must[0]];
    if (dist[1][must[0]] == INF)
    {
        return INF + 1;
    }
    for (int i = 0; i < must.size() - 1; i++)
    {
        if (dist[must[i]][must[i + 1]] == INF)
            return INF + 1;
        distance += dist[must[i]][must[i + 1]];
    }
    if (dist[must[must.size() - 1]][dest] == INF)
    {
        return INF + 1;
    }
    distance += dist[must[must.size() - 1]][dest];
    return distance;
}
// 将点src沿着当前must数组中指定点的顺序走到点dest的路径保存到anspath数组
void save_path(int src, int dest, std::vector<int> &must, std::vector<int> &anspath, std::vector<std::vector<int>> &path)
{
    anspath.push_back(src);
    get_path(src, must[0], path, anspath);
    for (int i = 0; i < must.size() - 1; i++)
    {
        anspath.push_back(must[i]);
        get_path(must[i], must[i + 1], path, anspath);
    }
    anspath.push_back(must[must.size() - 1]);
    get_path(must[must.size() - 1], dest, path, anspath);
    anspath.push_back(dest);
}
void operation6(Graph<std::string> &g_str, Graph<int> &g_int)
{
    std::cout << "最小生成树：" << std::endl;
    Graph<std::string> mst_str = g_str.prim(campus_map_int_str[1]);
    mst_str.show();
    int n = campus_map_str_int.size(), MAX = campus_map_str_int.size() + 1;

    // std::cout << mst_int.is_only_mst(1) ? "Unique" : "NOT Unique" << std::endl;

    // 用户输入
    std::cout << "请输入关键地点数和地点：" << std::endl;
    int k;
    std::cin >> k;

    if (k == 1) // 只有一个关键点，起点即终点
    {
        std::string t;
        std::cin >> t;
        std::cout << "路径长度为零，请再次选择此功能后重新输入！" << std::endl;
    }
    else if (k == 2) // 有两个关键点，不是起点就是终点，直接调用dijkstra算法
    {
        std::string src, dest;
        std::cin >> src >> dest;
        std::map<std::string, std::pair<int, std::vector<std::string>>> shortest_map;
        Graph<std::string> mst_str = g_str.prim(campus_map_int_str[1]);
        shortest_map = mst_str.dijkstra(src);
        for (auto u : shortest_map)
        {
            if (u.first == dest)
            {
                std::cout << dest << " ";
                for (auto v : u.second.second)
                {
                    std::cout << v << " ";
                }
                std::cout << u.second.first << std::endl;
            }
        }
    }
    else
    {
        std::vector<int> key_place(k); // 存储指定点序列，含首尾
        std::vector<int> must(k - 2);  // 必经途径点

        std::string tmp;
        for (int i = 0; i < k; i++)
        {
            std::cin >> tmp;
            key_place[i] = campus_map_str_int[tmp];
        }

        sort(key_place.begin(), key_place.begin() + k);

        std::vector<std::vector<int>> dist(MAX, std::vector<int>(MAX, INF)); // 存储各顶点间的最短路
        std::vector<std::vector<int>> path(MAX, std::vector<int>(MAX, -1));  // 用于floyd存储路径

        Graph<int> mst_int = g_int.prim(1);
        mst_int.get_adj_matrix(INF);
        dist = mst_int.adj_matrix;

        // 使用floyd计算多源最短路径
        mst_int.floyd(dist, path);

        int shortest_dis = INF;
        std::vector<int> anspath;

        // 关键点全排列法计算最短路，仅适用于小规模图
        do
        {
            int src = key_place[0], dest = key_place[k - 1];
            for (int i = 0; i < k - 2; i++)
                must[i] = key_place[i + 1];

            int dis = get_dis(src, dest, dist, must);

            if (shortest_dis > dis) // 如果存在更短的路径则更新答案，并把路径存到anspath
            {
                shortest_dis = dis;
                anspath.clear();
                save_path(src, dest, must, anspath, path);
            }
        } while (next_permutation(key_place.begin(), key_place.begin() + k));
        if (shortest_dis == INF)
        {
            std::cout << "没有最佳路径，地点信息有误！" << std::endl;
        }
        else
        {
            std::cout << "最佳路径：" << std::endl;
            for (auto res : anspath)
            {
                std::cout << campus_map_int_str[res] << " ";
            }
            std::cout << shortest_dis << std::endl;
        }
    }
    std::cout << std::endl;
}

// 拓扑排序的方式检测是否存在环
bool is_cyclic(std::vector<std::vector<int>> &graph)
{
    int n = graph.size();
    std::vector<int> in_degree(n, 0);

    // 计算所有顶点的入度
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            if (graph[i][j])
            {
                in_degree[j]++;
            }
        }
    }

    std::queue<int> q;
    for (int i = 0; i < n; i++)
    {
        if (in_degree[i] == 0)
        {
            q.push(i);
        }
    }

    int cnt = 0;
    while (!q.empty())
    {
        int u = q.front();
        q.pop();
        for (int i = 0; i < n; i++)
        {
            if (graph[u][i] && --in_degree[i] == 0)
            {
                q.push(i);
            }
        }
        cnt++;
    }

    // 如果计数值不等于顶点数，表示存在环
    if (cnt != n)
    {
        return true;
    }
    else
    {
        return false;
    }
}
// DFS函数，用于生成所有简单路径
void dfs_op7(std::vector<std::vector<int>> &graph, int u, int dest, std::vector<int> &path, std::vector<bool> &visited, int cutoff_l, int cutoff_s, std::vector<std::vector<int>> &allpath)
{
    path.push_back(u);
    visited[u] = true;

    if (u == dest && path.size() <= cutoff_l + 1 && path.size() >= cutoff_s)
    {
        allpath.push_back(path);
    }
    else if (path.size() < cutoff_l + 1)
    {
        for (int v = 0; v < graph[u].size(); v++)
        {
            if (graph[u][v] != -1 && !visited[v])
            {
                dfs_op7(graph, v, dest, path, visited, cutoff_l, cutoff_s, allpath);
            }
        }
    }

    visited[u] = false;
    path.pop_back();
}
void all_simple_paths(std::vector<std::vector<int>> &graph, int src, int dest, int cutoff_l, int cutoff_s, std::vector<std::vector<int>> &allpath)
{
    if (!graph[src].empty() && !graph[dest].empty())
    {
        std::vector<int> path;
        std::vector<bool> visited(cutoff_l, false);
        dfs_op7(graph, src, dest, path, visited, cutoff_l - 1, cutoff_s, allpath);
    }
}
void operation7(Graph<std::string> &g_str, Graph<int> &g_int)
{
    int MAX = campus_map_str_int.size() + 1;

    std::cout << "请输入想参观的地点数和地点：" << std::endl;
    int k;
    std::cin >> k;
    std::string tmp;
    std::vector<int> nodes;
    for (int i = 0; i < k; i++)
    {
        std::cin >> tmp;
        nodes.push_back(campus_map_str_int[tmp]);
    }
    // 读入源点和目标点
    int src = nodes.front(), dest = nodes.back();
    nodes.erase(nodes.begin());
    nodes.pop_back();

    std::cout << "请输入游览顺序限制组数和地点：" << std::endl;
    int n;
    std::cin >> n;
    std::string tmp1, tmp2;
    std::vector<std::pair<int, int>> lim(n);
    std::vector<std::vector<int>> lim_edges(MAX, std::vector<int>(MAX, 0));
    for (int i = 0; i < n; i++)
    {
        std::cin >> tmp1 >> tmp2;
        lim_edges[campus_map_str_int[tmp1]][campus_map_str_int[tmp2]] = 1;

        lim[i].first = campus_map_str_int[tmp1];
        lim[i].second = campus_map_str_int[tmp2];
    }

    std::vector<std::vector<int>> G(MAX, std::vector<int>(MAX, -1));
    g_int.get_adj_matrix(-1);
    G = g_int.adj_matrix;

    if (std::find(nodes.begin(), nodes.end(), campus_map_str_int["北门"]) != nodes.end())
    {
        std::cout << "由于途经A需要两次经过D关节点，无法通达！" << std::endl;
        std::cout << std::endl;
    }
    else if (is_cyclic(lim_edges))
    {
        std::cout << "图中游览限制存在环而不可达！" << std::endl;
        std::cout << std::endl;
    }
    else
    {
        std::vector<std::vector<int>> allpath; // 存储所有简单路径
        all_simple_paths(G, src, dest, MAX, k, allpath);
        /*for (auto u : allpath)
        {
            for (auto v : u)
            {
                std::cout << v << " ";
            }
            std::cout << std::endl;
        }*/

        std::vector<int> shortest_path;
        int shortest_dis = INF;
        for (auto path : allpath)
        {
            bool all_in_path = true;

            for (auto p : nodes)
            {
                if (std::find(path.begin(), path.end(), p) == path.end())
                {
                    all_in_path = false;
                    break;
                }
            }
            if (!all_in_path)
                continue;
            // std::cout << "all_in_path1：" << all_in_path << std::endl;
            for (auto p : lim)
            {
                auto it1 = std::find(path.begin(), path.end(), p.first);
                auto it2 = std::find(path.begin(), path.end(), p.second);
                if (it1 > it2)
                {
                    all_in_path = false;
                    break;
                }
            }
            if (!all_in_path)
                continue;
            // std::cout << "all_in_path2：" << all_in_path << std::endl;

            if (all_in_path)
            {
                int path_len = 0;
                for (int i = 0; i < path.size() - 1; i++)
                {
                    int u = path[i];
                    int v = path[i + 1];
                    path_len += G[u][v];
                }
                if (path_len < shortest_dis)
                {
                    shortest_dis = path_len;
                    shortest_path = path;
                }
            }
        }
        if (shortest_dis == INF)
        {
            std::cout << "起点和终点由于路径死锁无法通达！" << std::endl;
        }
        else
        {
            std::cout << "最短路径：" << std::endl;
            std::cout << campus_map_int_str[src] << " ";
            for (auto node : shortest_path)
            {
                if (std::find(nodes.begin(), nodes.end(), node) != nodes.end())
                {
                    std::cout << campus_map_int_str[node] << " ";
                }
            }
            std::cout << campus_map_int_str[dest] << " ";
            std::cout << shortest_dis << std::endl;
        }
        std::cout << std::endl;
    }
}