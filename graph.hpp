#pragma once
#include "edge.hpp"
#include <iostream>
#include <vector>
#include <map>
#include <unordered_map>
#include <set>
#include <unordered_set>
#include <string>
#include <queue>
#include <stack>
#include <limits>
const int INF = 0x3f3f3f3f;

template <typename T>
class Graph
{
public:
    std::map<T, std::set<Edge<T>>> adj;     /* �ڽӱ� */
    std::vector<std::vector<T>> adj_matrix; /* �ڽӾ��󣬽���ʹ��get_adj_matrix����ʱ��� */

    bool contains(const T &u);             /* �ж϶���u�Ƿ���ͼ�� */
    bool adjacent(const T &u, const T &v); /* �ж϶���u��v�Ƿ����� */

    void add_vertex(const T &u);                            /* ��Ӷ��� */
    void add_edge(const T &u, const T &v, int weight);      /* ��ӱߺ�Ȩ�� */
    void change_weight(const T &u, const T &v, int weight); /* �޸�Ȩ�� */
    void remove_weight(const T &u, const T &v);             /* �Ƴ�Ȩ�� */
    void remove_vertex(const T &u);                         /* �Ƴ����� */
    void remove_edge(const T &u, const T &v);               /* �Ƴ��� */

    void get_adj_matrix(int num); /* �õ��ڽӾ��󣬽�int���� */

    int degree(const T &u);                 /* �󶥵�Ķ��� */
    int num_vertices();                     /* ��ͼ�ж�������� */
    int num_edges();                        /* ��ͼ�бߵ�����*/
    int largest_degree();                   /* ��ͼ�е������� */
    int get_weight(const T &u, const T &v); /* �õ�ĳ��������֮��ߵ�Ȩ�� */

    std::vector<T> get_vertices();               /* �õ�ͼ�����ж��� */
    std::map<T, int> get_neighbours(const T &u); /* �õ�����u�����б� */

    void show();

    void dft_recursion(const T &u, std::set<T> &visited, std::vector<T> &result); /* ������ȱ����ݹ鸨������ */
    std::vector<T> depth_first_rec(const T &u);                                   /* ������ȱ����ݹ鷨 */
    std::vector<T> depth_first_itr(const T &u);                                   /* ������ȱ���������*/
    std::vector<T> breadth_first(const T &u);                                     /* ������ȱ��������� */

    Graph<T> prim(T v); /* prim��С�������㷨 */

    std::map<T, std::pair<int, std::vector<T>>> dijkstra(T start);                        /* dijkstra���·���㷨��ȫ������ */
    std::pair<int, std::vector<T>> dijkstra(T start, T end);                              /* dijkstra���·���㷨����ʼ���� */
    void floyd(std::vector<std::vector<int>> &dist, std::vector<std::vector<int>> &path); /* floyd���·���㷨��ȫ������ */

    std::vector<T> topological_sort(); /* �������� */

    std::vector<std::vector<T>> get_connected_components();                                   /* ���ͼ�е���ͨ���� */
    void print_connected_components(const std::vector<std::vector<T>> &connected_components); /* ��ӡ��ͨ���� */
    std::vector<T> articulation_points(int choice);                                           /* ���ͼ�еĹؽڵ㣨�ָ�㣩*/

private:
    using P = std::pair<int, T>;
    void dft(T u, T root, T parent, std::set<T> &visited_vertices, /* ��ùؽڵ��Tarjan�㷨 */
             int &dfn_cnt, std::map<T, int> &dfn, std::map<T, int> &low, std::vector<T> &articulation_point_collection);
    void violent_solution(std::vector<T> &articulation_point_collection); /* ��ùؽڵ�ı�����ⷨ */
};

// ��������
template <typename T>
void Graph<T>::show()
{
    std::cout << "�ص���Ϣ��" << std::endl;
    for (const auto &u : adj)
        if (adj[u.first].size())
            std::cout << u.first << " ";
    std::cout << std::endl;

    std::cout << "��·��Ϣ��" << std::endl;
    int i = 1;
    for (const auto &u : adj)
        for (const auto &v : adj[u.first])
            std::cout << i++ << ": " << u.first << " " << v.vertex << " " << v.weight << std::endl;
    std::cout << std::endl;

    /*for (const auto &u : adj)
    {
        std::cout << "����" << u.first << ": ";
        for (const auto &v : adj[u.first])
            std::cout << "(���ڶ���: " << v.vertex << ", �ߵ�Ȩ��: " << v.weight << ") ";
        std::cout << std::endl;
    }*/
}

template <typename T>
bool Graph<T>::contains(const T &u)
{
    return adj.find(u) != adj.end();
}

template <typename T>
bool Graph<T>::adjacent(const T &u, const T &v)
{
    if (contains(u) && contains(v) && u != v)
    {
        for (auto edge : adj[u])
            if (edge.vertex == v)
                return true;
    }
    return false;
}

template <typename T>
void Graph<T>::add_vertex(const T &u)
{
    if (!contains(u))
    {
        std::set<Edge<T>> edge_list;
        adj[u] = edge_list;
    }
}

template <typename T>
void Graph<T>::add_edge(const T &u, const T &v, int weight)
{
    if (!adjacent(u, v))
    {
        adj[u].insert(Edge<T>(v, weight));
        adj[v].insert(Edge<T>(u, weight));
    }
    // �¼����޸�
    else
    {
        change_weight(u, v, weight);
    }
}

template <typename T>
void Graph<T>::change_weight(const T &u, const T &v, int weight)
{
    if (contains(u) && contains(v))
    {
        if (adj[u].find(Edge<T>(v)) != adj[u].end())
        {
            adj[u].erase(Edge<T>(v));
            adj[u].insert(Edge<T>(v, weight));
        }

        if (adj[v].find(Edge<T>(u)) != adj[v].end())
        {
            adj[v].erase(Edge<T>(u));
            adj[v].insert(Edge<T>(u, weight));
        }
    }
}

template <typename T>
void Graph<T>::remove_weight(const T &u, const T &v)
{
    if (contains(u) && contains(v))
    {
        if (adj[u].find(Edge<T>(v)) != adj[u].end())
        {
            adj[u].erase(Edge<T>(v));
            adj[u].insert(Edge<T>(v, 0));
        }

        if (adj[v].find(Edge<T>(u)) != adj[v].end())
        {
            adj[v].erase(Edge<T>(u));
            adj[v].insert(Edge<T>(u, 0));
        }
    }
}

template <typename T>
void Graph<T>::remove_vertex(const T &u)
{
    if (contains(u))
    {
        for (auto &vertex : adj)
        {
            if (vertex.second.find(Edge<T>(u)) != vertex.second.end())
            {
                vertex.second.erase(Edge<T>(u));
            }
        }
        adj.erase(u);
    }
}

template <typename T>
void Graph<T>::remove_edge(const T &u, const T &v)
{
    if (u == v || !contains(u) || !contains(v))
        return;

    if (adj[u].find(Edge<T>(v)) != adj[u].end())
    {
        adj[u].erase(Edge<T>(v));
        adj[v].erase(Edge<T>(u));
    }
}

// �¼����޸�
template <typename T>
void Graph<T>::get_adj_matrix(int num)
{
    int size = adj.size() + 1;
    adj_matrix.resize(size, std::vector<int>(size, num));

    for (const auto &u : adj)
    {
        for (const auto &v : adj[u.first])
        {
            adj_matrix[u.first][v.vertex] = v.weight;
            adj_matrix[v.vertex][u.first] = v.weight;
        }
    }
}

template <typename T>
int Graph<T>::degree(const T &u)
{
    if (contains(u))
    {
        return adj[u].size();
    }
    return -1; // ����Ϊ-1˵��ͼ��û�иö���
}

template <typename T>
int Graph<T>::num_vertices()
{
    return adj.size();
}

template <typename T>
int Graph<T>::num_edges()
{
    int count = 0;
    std::set<Edge<T>> vertex_set;

    for (auto vertex : adj)
    {
        vertex_set.insert(Edge<T>(vertex.first, 0));
        for (auto edge : vertex.second)
        {
            if (vertex_set.find(edge) != vertex_set.end())
                continue;
            count++;
        }
    }
    return count;
}

template <typename T>
int Graph<T>::largest_degree()
{
    if (num_vertices() == 0)
        return 0;

    unsigned max_degree = 0;
    for (auto vertex : adj)
    {
        if (vertex.second.size() > max_degree)
        {
            max_degree = vertex.second.size();
        }
    }
    return max_degree;
}

template <typename T>
int Graph<T>::get_weight(const T &u, const T &v)
{
    if (contains(u) && contains(v))
    {
        for (Edge<T> edge : adj[u])
        {
            if (edge.vertex == v)
            {
                return edge.weight;
            }
        }
    }
    return -1;
}

template <typename T>
std::vector<T> Graph<T>::get_vertices()
{
    std::vector<T> vertices;
    for (auto vertex : adj)
    {
        vertices.push_back(vertex.first);
    }
    return vertices;
}

template <typename T>
std::map<T, int> Graph<T>::get_neighbours(const T &u)
{
    std::map<T, int> neighbours;

    if (contains(u))
    {
        for (Edge<T> edge : adj[u])
        {
            neighbours[edge.vertex] = edge.weight;
        }
    }
    return neighbours;
}

// ����
template <typename T>
void Graph<T>::dft_recursion(const T &u, std::set<T> &visited, std::vector<T> &result)
{
    result.push_back(u);
    visited.insert(u);

    for (Edge<T> edge : adj[u])
    {
        if (visited.find(edge.vertex) == visited.end())
        {
            dft_recursion(edge.vertex, visited, result);
        }
    }
}

template <typename T>
std::vector<T> Graph<T>::depth_first_rec(const T &u)
{
    std::vector<T> result;
    std::set<T> visited;
    if (contains(u))
    {
        dft_recursion(u, visited, result);
    }
    return result;
}

template <typename T>
std::vector<T> Graph<T>::depth_first_itr(const T &u)
{
    std::vector<T> result;
    std::set<T> visited;
    std::stack<T> s;

    s.push(u);
    while (!s.empty())
    {
        T v = s.top();
        s.pop();

        if (visited.find(v) != visited.end())
            continue;

        visited.insert(v);
        result.push_back(v);

        for (auto w : adj[v])
        {
            if (visited.find(w.vertex) == visited.end())
            {
                s.push(w.vertex);
            }
        }
    }
    return result;
}

template <typename T>
std::vector<T> Graph<T>::breadth_first(const T &u)
{
    std::vector<T> result;
    std::set<T> visited;
    std::queue<T> q;

    q.push(u);
    while (!q.empty())
    {
        T v = q.front();
        q.pop();

        if (visited.find(v) != visited.end())
            continue;

        visited.insert(v);
        result.push_back(v);

        for (Edge<T> edge : adj[v])
        {
            if (visited.find(edge.vertex) == visited.end())
            {
                q.push(edge.vertex);
            }
        }
    }
    return result;
}

template <typename T>
Graph<T> Graph<T>::prim(T v)
{
    Graph<T> min_spanning_tree;      // ��С�������Ĵ���
    min_spanning_tree.add_vertex(v); // ������������Ӷ���v

    // ���ô�Ȩ�صĶ��У�����һ��Ԫ�أ�Ȩֵ�����д�С���������
    std::priority_queue<std::pair<int, std::pair<T, T>>, std::vector<std::pair<int, std::pair<T, T>>>, std::greater<std::pair<int, std::pair<T, T>>>> q;

    std::set<T> visited; // ���ü���visited������Ѿ����ʹ��Ķ���

    // ��ӣ���ӵ�Ԫ����һ��pair���ͣ���һ��ֵ��Ȩ�أ��ڶ���ֵҲ��pair
    // �ڶ���ֵ��pair�����һ��ֵ��u��ֻ���������д��ڵĶ��㣩, �ڶ���ֵ��v��ֻ����ԭͼ�д��ڵĵ㣩
    for (auto neighbour : adj[v])
    {
        q.push(std::make_pair(neighbour.weight, std::make_pair(v, neighbour.vertex)));
    }

    while (!q.empty())
    {
        // ����Ԫ�س���
        auto front = q.top();
        q.pop();

        T u = front.second.first;  // ��������������еĶ���u
        T v = front.second.second; // �����ԭͼ��, �������������еĶ���v

        // �������v�Ѿ����ʹ�����������ѭ��
        if (visited.find(v) != visited.end())
            continue;
        else
        {
            visited.insert(v);
        }

        // ��������������µĶ���v�Լ�v��u֮��ı�
        min_spanning_tree.add_vertex(v);
        min_spanning_tree.add_edge(u, v, front.first);

        // ���ν�����v��δ���ʹ����ھӷ������ȶ�����
        for (auto neighbour : adj[v])
        {
            if (visited.find(neighbour.vertex) == visited.end())
            {
                q.push(std::make_pair(neighbour.weight, std::make_pair(v, neighbour.vertex)));
            }
        }
    }
    return min_spanning_tree;
}

template <typename T>
std::map<T, std::pair<int, std::vector<T>>> Graph<T>::dijkstra(T start)
{
    std::map<T, int> dis;             // ����dis������ų�ʼ�㵽ͼ���κ�һ������ľ���
    std::map<T, std::vector<T>> path; // �¼����޸ģ��������·��

    // ���ô�Ȩ�صĶ��У���ÿ��pair�ĵ�һ��Ԫ�ؽ��д�С���������
    std::priority_queue<std::pair<int, T>, std::vector<std::pair<int, T>>, std::greater<std::pair<int, T>>> q;

    for (T vertex : get_vertices())
    {
        if (vertex == start)
        {
            dis[start] = 0; // ���ó�ʼ���㵽�Լ��ľ���Ϊ0
        }
        else
        {
            dis[vertex] = INT_MAX; // ���ó�ʼ���㵽��������ľ���Ϊ�����
        }
    }

    std::set<T> visited; // ���ü���visited������Ѿ����ʹ��Ķ���

    q.push(std::make_pair(0, start)); // ��ӣ���ӵ�Ԫ����һ��pair���ͣ���һ��ֵ��Ȩ�أ��ڶ���ֵ�Ƕ���

    while (!q.empty())
    {
        // ����Ԫ�س���
        auto front = q.top();
        q.pop();

        T u = front.second; // ��õ�ǰ����

        // ����ö����Ѿ����ʹ�����������ѭ����������뵽����visited�б�ʾ�Ѿ����ʹ�
        if (visited.find(u) != visited.end())
            continue;
        else
        {
            visited.insert(u);
        }

        // ��õ�����u�����·��"shortest_distance_to_u"������·�����뵽dis�����
        int shortest_distance_to_u = front.first;
        dis[u] = shortest_distance_to_u;

        // ���η��ʶ���u��δ���ʹ����ھ�
        for (auto v : adj[u])
        {
            if (visited.find(v.vertex) == visited.end())
            {
                // �Ӷ���u���ھ�v��·����Ϊ��distance_to_v��
                int distance_to_v = v.weight;
                int new_distance = shortest_distance_to_u + distance_to_v;

                // ����µľ���С�ڵ�ǰ��¼�ľ��룬��ô���¾����·��
                if (new_distance < dis[v.vertex])
                {
                    dis[v.vertex] = new_distance;
                    q.push(std::make_pair(new_distance, v.vertex));

                    // ����·��
                    path[v.vertex] = path[u];
                    path[v.vertex].push_back(v.vertex);
                }
            }
        }
    }

    // path�������
    std::map<T, std::pair<int, std::vector<T>>> res;

    for (auto &p : dis)
    {
        auto it = path.find(p.first);
        if (it != path.end())
        {
            res[p.first] = std::make_pair(p.second, it->second);
        }
    }
    return res;
}
template <typename T>
std::pair<int, std::vector<T>> Graph<T>::dijkstra(T start, T end)
{
    std::map<T, int> dist;
    std::map<T, T> prev;
    for (auto &p : adj)
    {
        dist[p.first] = INF;
    }
    dist[start] = 0;

    std::priority_queue<std::pair<int, T>, std::vector<std::pair<int, T>>, std::greater<std::pair<int, T>>> pq;
    pq.push({0, start});
    while (!pq.empty())
    {
        int d = pq.top().first;
        T v = pq.top().second;
        pq.pop();
        if (d > dist[v])
            continue;
        for (auto &e : adj[v])
        {
            if (dist[e.vertex] > dist[v] + e.weight)
            {
                dist[e.vertex] = dist[v] + e.weight;
                prev[e.vertex] = v;
                pq.push({dist[e.vertex], e.vertex});
            }
        }
    }

    std::vector<T> path;
    for (T v = end; v != start; v = prev[v])
    {
        path.push_back(v);
    }
    path.push_back(start);
    reverse(path.begin(), path.end());
    return {dist[end], path};
}

template <typename T>
void Graph<T>::floyd(std::vector<std::vector<int>> &dist, std::vector<std::vector<int>> &path)
{
    int n = num_vertices();
    for (int k = 0; k <= n; k++)
    {
        for (int i = 0; i <= n; i++)
        {
            for (int j = 0; j <= n; j++)
            {
                if (dist[i][j] > dist[i][k] + dist[k][j])
                {
                    dist[i][j] = dist[i][k] + dist[k][j];
                    path[i][j] = k;
                }
            }
        }
    }
}

template <typename T>
std::vector<T> Graph<T>::topological_sort()
{
    // ��ʼ�����ж�������Ϊ0
    std::map<T, int> indegree;
    for (auto &pair : adj)
    {
        for (const Edge<T> &edge : pair.second)
        {
            ++indegree[edge.vertex];
        }
    }

    // ���������Ϊ0�Ķ���������
    std::queue<T> q;
    for (auto &pair : adj)
    {
        if (indegree[pair.first] == 0)
        {
            q.push(pair.first);
        }
    }

    // ������������
    std::vector<T> result;
    while (!q.empty())
    {
        T u = q.front();
        q.pop();
        result.push_back(u);

        for (const Edge<T> &edge : adj[u])
        {
            --indegree[edge.vertex];
            if (indegree[edge.vertex] == 0)
            {
                q.push(edge.vertex);
            }
        }
    }

    // �������еĶ�����������ͼ�еĶ�������˵��ͼ�д��ڻ�
    std::vector<T> res;
    if (result.size() != adj.size())
    {
        std::cout << "ͼ�д��ڻ���" << std::endl;
        return res;
    }
    else
    {
        return result;
    }
}

template <typename T>
std::vector<std::vector<T>> Graph<T>::get_connected_components()
{
    std::set<T> visited_vertices;
    std::vector<std::vector<T>> connected_components;

    for (auto vertex : adj)
    {
        // ��ÿһ��δ���ʹ��Ķ������������ȱ���
        if (visited_vertices.find(vertex.first) == visited_vertices.end())
        {
            std::stack<T> s;
            s.push(vertex.first);

            // ����һ����ʱ����"connected_component"�����洢��ǰ��ͨ�����еĶ���
            std::vector<T> connected_component;

            // ������ȱ���
            while (!s.empty())
            {
                T u = s.top();
                s.pop();

                // ��δ���ʹ��Ķ��������ͨ����"connected_component"��
                if (visited_vertices.find(u) == visited_vertices.end())
                {
                    connected_component.push_back(u);
                    visited_vertices.insert(u);
                }

                // ��ǰ����δ���ʹ����ھ���ջ
                for (auto neighbour : adj[u])
                {
                    if (visited_vertices.find(neighbour.vertex) == visited_vertices.end())
                    {
                        s.push(neighbour.vertex);
                    }
                }
            }
            // ����ͨ�����ŵ���ͨ�����ļ���"connected_components"��
            connected_components.push_back(connected_component);
        }
    }
    return connected_components;
}

template <typename T>
void Graph<T>::print_connected_components(const std::vector<std::vector<T>> &connected_components)
{
    int number = connected_components.size();
    if (number == 1)
    {
        std::cout << "��ͼ����ͨͼ��ֻ��һ����ͨ����������������" << std::endl;
    }
    else if (number > 1)
    {
        std::cout << "ͼ�й���" << number << "����ͨ����" << std::endl;
        for (unsigned i = 0; i < connected_components.size(); i++)
        {
            std::cout << "��" << i + 1 << "����ͨ�����еĶ���ֱ�Ϊ:";
            for (unsigned j = 0; j < connected_components[i].size(); j++)
            {
                std::cout << " " << connected_components[i][j];
            }
            std::cout << std::endl;
        }
    }
}

template <typename T>
std::vector<T> Graph<T>::articulation_points(int choice)
{
    int dfn_cnt = 0;                              // ����������ȱ����Ĵ���
    std::vector<T> articulation_point_collection; // ��¼ͼ�г��ֵķָ��
    std::map<T, int> dfn;                         // ��¼������ȱ���˳��
    std::map<T, int> low;                         // ��¼��ĳ���ض�����Ϊ���������ܻ��ݵ�����������ȶ���
    std::set<T> visited_vertices;                 // ��¼�ѷ��ʹ��Ķ���

    if (choice == 1)
    {
        violent_solution(articulation_point_collection);
    }

    else if (choice == 2)
    {
        // ��δ���ʹ��Ķ������������ȱ�����ָ�㣨ʵ��������ÿһ����ͨ������ʹ��һ��������ȱ�����
        for (auto u : adj)
        {
            if (visited_vertices.find(u.first) == visited_vertices.end())
            {
                dft(u.first, u.first, u.first, visited_vertices, dfn_cnt, dfn, low, articulation_point_collection);
            }
        }
        int a = 1;
    }
    return articulation_point_collection;
}

template <typename T>
void Graph<T>::violent_solution(std::vector<T> &articulation_point_collection)
{
    unsigned original_number = get_connected_components().size(); // ���ԭ����ͼ����ͨ��������
    std::vector<T> vertices = get_vertices();                     // ���ͼ�е����ж���

    for (T vertex : vertices)
    {
        std::map<T, int> temp_neighbours = get_neighbours(vertex); // �ݴ�Ҫɾ���Ķ��㸽�����ھ�
        remove_vertex(vertex);                                     // ɾ������

        // ��ɾ�������ͨ����������ɾ��ǰ�ıȽ�
        unsigned current_number = get_connected_components().size();
        if (current_number > original_number)
        {
            articulation_point_collection.push_back(vertex);
        }

        // ��ӻض��㼰��Ӧ�ı�
        add_vertex(vertex);
        for (auto neighbour : temp_neighbours)
        {
            add_edge(vertex, neighbour.first, neighbour.second);
        }
    }
}

template <typename T>
void Graph<T>::dft(T u, T root, T parent, std::set<T> &visited_vertices,
                   int &dfn_cnt, std::map<T, int> &dfn, std::map<T, int> &low, std::vector<T> &articulation_point_collection)
{
    // ��¼������ȱ�������
    dfn_cnt++;
    dfn[u] = dfn_cnt;

    low[u] = dfn[u];            // ��ʼ��low[u]
    visited_vertices.insert(u); // ��ǵ�ǰ����Ϊ�ѷ���

    int n_subtree = 0;   // ��¼��������
    bool is_cut = false; // ��¼�ö����Ƿ�Ϊ�ؽڵ�

    for (auto edge : adj[u])
    {
        T v = edge.vertex;

        // ��(u,v)��Ϊ����ʱ
        if (visited_vertices.find(v) == visited_vertices.end())
        {
            n_subtree++;

            // ��u�ĺ���v����������ȱ�������ʱu��Ϊparent
            dft(v, root, u, visited_vertices, dfn_cnt, dfn, low, articulation_point_collection);

            // ��vΪ���ڵ�������ܷ��ʵ������ȱ�ȻҲ�ܴ�u���������ʵ�������������uֵ
            low[u] = std::min(low[u], low[v]);

            // ��vΪ���ڵ�������ܷ��ʵ������������Ϊu����vʱ������жϳ�����u���Ǹ��ڵ㣩Ϊ�ؽڵ�
            if (u != root && low[v] >= dfn[u])
            {
                is_cut = true;
            }
        }

        // ��(u,v)��Ϊ�ر�ʱ
        // ʹ��v��������ȱ�������������low[u]
        else if (v != parent)
        {
            low[u] = std::min(low[u], dfn[v]);
        }
    }

    // uΪ���ڵ��������������ڵ���2�����
    if (n_subtree >= 2 && u == root)
    {
        is_cut = true;
    }

    // ��¼�ؽڵ�
    if (is_cut)
    {
        articulation_point_collection.push_back(u);
    }
}
