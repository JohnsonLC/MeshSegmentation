#ifndef H_GRAGH_
#define H_GRAGH_

#include <vector>
#include <queue>
#include <map>

#include "logging.h"
#include "utils.h"

#define INF 2147483647

// 定义邻接表的顶点结构
struct Vertex {
    int v;
    double cost;

    Vertex() {}

    Vertex(int v, double cost) {
        this->v = v;
        this->cost = cost;
    }
};

// 定义放到最小堆中的节点
struct Node {
    int u;
    double dist;

    Node() {}

    Node(int u, double dist) {
        this->u = u;
        this->dist = dist;
    }

    bool operator < (const Node& A) const {
        return dist > A.dist;
    }
};

class Graph {
public:
    int mNumFaces_;

    std::vector<std::vector<double>> dist_;     // 两两之间最短距离矩阵
    Utils* utils_;
public:

    Graph() {}

    Graph(Utils* utils) {
        utils_ = utils;
        mNumFaces_ = utils_->mNumFaces_;

        // 为邻接矩阵预分配大小
        for (int i=0;i<mNumFaces_;i++) {
            dist_.push_back(std::vector<double>());
            for (int j=0;j<mNumFaces_;j++) {
                if (i == j) dist_[i].push_back(0);
                else dist_[i].push_back(INF);
            }
        }

        // 计算邻接矩阵
        GetGraph();
    }

    // 建立邻接矩阵
    void GetGraph() {
        for (auto m: utils_->edge_neighbour_) {
            int face_idx_1 = m.second[0];
            int face_idx_2 = m.second[1];

            // 计算两个三角面片的距离
            double dist = utils_->GetFacesDistance(face_idx_1, face_idx_2);
            // printf("dist of face %d and face %d: %f\n", face_idx_1, face_idx_2, dist);

            // 建立邻接矩阵
            dist_[face_idx_1][face_idx_2] = dist;
            dist_[face_idx_2][face_idx_1] = dist;
        }
    }

    // 利用floyd计算两两顶点间最小距离矩阵
    void floyd() {
        for (int k=0;k<mNumFaces_;k++) {
            for (int i=0;i<mNumFaces_;i++) {
                for (int j=0;j<mNumFaces_;j++) {
                    if (dist_[i][k] + dist_[k][j] < dist_[i][j]) {
                        dist_[i][j] = dist_[i][k] + dist_[k][j];
                    }
                }
            }
        }
    }

    // 利用堆优化的Dijkstra来求最短路
    void Dijkstra() {
        // 设置标记数组
        std::vector<double> dist; dist.resize(mNumFaces_);
        std::vector<bool> vis;  vis.resize(mNumFaces_);

        std::priority_queue<Node> q;

        for (int i=0;i<mNumFaces_;i++) {
            for (int j=0;j<mNumFaces_;j++) {
                dist[j] = INF;
                vis[j] = false;
            }
            dist[i] = 0;

            // Dijkstra的源
            q.push(Node(i, 0));

            while (!q.empty()) {
                Node cur = q.top(); q.pop();

                int u = cur.u;
                double d = cur.dist;

                // 标记已经遍历过
                vis[u] = true;

                // 获取当前面片的邻居数量
                int neighbour_num = utils_->face_neighbour_[u].size();
                if (neighbour_num != 3) {
                    // 警告之类
                    // **** TODO
                }

                for (int j=0;j<neighbour_num;j++) {
                    // 获取邻居的face索引数
                    int v = utils_->face_neighbour_[u][j];
                    if (vis[v]) continue;

                    if (dist[v] > d + dist_[u][v]) {
                        dist[v] = d + dist_[u][v];
                        q.push(Node(v, dist[v]));
                    }
                }
            }

            for (int j=0;j<mNumFaces_;j++) {
                dist_[i][j] = dist[j];
            }
        }
    }

    // 返回相隔最远的两个面的索引
    std::pair<int, int> GetMostFarFaces() {
        std::pair<int, int> res;

        double max_distance = -1;
        for (int i=0;i<mNumFaces_;i++) {
            for (int j=0;j<mNumFaces_;j++) {
                if (max_distance < dist_[i][j]) {
                    max_distance = dist_[i][j];
                    res = std::make_pair(i, j);
                }
            }
        }

        printf("most far faces: %d, %d, distance: %f\n", res.first, res.second, max_distance);

        return res;
    }
};

#endif