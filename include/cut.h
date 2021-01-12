#ifndef H_CUT_
#define H_CUT_

#include <string>
#include <queue>
#include <algorithm>

#include "logging.h"
#include "cluster.h"
#include "graph.h"

class CutSolver {
public:
    Utils* utils_;
    aiMesh* mesh_;
    Graph* graph_;
    FuzzClusterSolver* solver_;

    int mNumFaces_;
    // num_ = fNumVertices_ - 2
    int fNumVertices_;
    int num_;
    // 顶点集合中，a和b开始的索引
    int start_a_;
    int start_b_;

    int k_;

    // 网络流图
    std::vector<int> vertices_;
    std::vector<int> inv_vertices_;     // 倒排索引，由face_idx反算得到网络流图中的编号
    std::vector<std::map<int, double>> flow_graph_;

    std::vector<int> pre;   // 网络流的pre数组，用来记录增广路径

    // 获取最小割得到的两个点集，存放face_id
    std::vector<std::vector<int>> vertices_zone_;
    std::vector<int> face_zone_;
public: 
    CutSolver() {}

    CutSolver(Utils* utils, Graph* graph, FuzzClusterSolver* solver) {
        utils_ = utils;
        graph_ = graph;
        solver_ = solver;
        mesh_ = utils_->mesh_;

        mNumFaces_ = graph_->mNumFaces_;
        num_ = 0;
        fNumVertices_ = 0;

        k_ = solver_->k_;

        face_zone_ = solver_->face_zone_;
        vertices_zone_.resize(k_);
    }

    // 更新
    // 将增广路上的容量都减去最小值
    void update(double delta) {
        for (int i=num_+1;pre[i] != -1;i = pre[i]) {
            flow_graph_[pre[i]][i] -= delta;
            flow_graph_[i][pre[i]] += delta;
            /*
            if (flow_graph_[pre[i]][i] == 0) {
                printf("container from %d to %d become zero.\n", pre[i], i);
            }*/
        }
    }

    // 利用bfs求解增广路上最小增量
    double find_path_bfs() {
        std::vector<bool> vis;

        pre.clear();
        pre.resize(fNumVertices_);
        vis.resize(fNumVertices_);

        for (int i=0;i<fNumVertices_;i++) {
            pre[i] = -1;
            vis[i] = false;
        }

        std::queue<int> q;
        q.push(num_);
        vis[num_] = true;

        bool finish = false;
        while (!finish && !q.empty()) {
            int cur = q.front(); q.pop();

            // 遍历邻接的顶点
            for (auto next: flow_graph_[cur]) {
                // 获取邻接顶点的顶点编号和边容量
                int v = next.first;
                double cost = next.second;

                if (!vis[v] && cost) {
                    vis[v] = true;
                    pre[v] = cur;

                    if (v == num_ + 1) {
                        finish = true;
                        break;
                    }
                    q.push(v);
                }
            }
        }

        if (pre[num_ + 1] == -1) return 0;

        double mmin = INF;
        for (int i=num_+1;pre[i] != -1;i=pre[i]) {
            mmin = std::min(flow_graph_[pre[i]][i], mmin);
        }

        // printf("current update_flow: %f\n", mmin);
        return mmin;
    }

    // EK求解最大流问题
    void Solve() {
        // 遍历所有的fuzz空间
        for (int i=k_;i<k_ + k_*k_;i++) {
            if (solver_->cluster_[i].size()) {
                // 根据这个fuzz空间建立最小割图
                Build(i);

                // 求解当前图的最大流
                double max_flow = 0.0;
                double update_flow = 0.0;
                while (update_flow = find_path_bfs()) {
                    update(update_flow);
                    max_flow += update_flow;
                }

                // 分割点集
                int zone_A = (i - k_) / k_;
                int zone_B = (i - k_) % k_;

                std::vector<bool> vis(fNumVertices_, false);

                std::queue<int> q;
                q.push(num_);
                vis[num_] = true;

                while (!q.empty()) {
                    int cur = q.front(); q.pop();

                    for (auto next: flow_graph_[cur]) {
                        // 邻接顶点
                        int v = next.first;
                        double cost = next.second;

                        if (!vis[v] && cost) {
                            vis[v] = true;
                            // 这里面可能会存在问题 Bug
                            // 不同的顶点可能会被加入不同的最小割图中，然后就可能会加入不同的vertices_zone_中，也就是同一个顶点可能会多次出现在不同的vertices_zone_中
                            // 但应该对结果没有影响，因为face_zone_，每个顶点只对应一个区域，所以绘制不会有问题
                            vertices_zone_[zone_A].push_back(vertices_[v]);
                            face_zone_[vertices_[v]] = zone_A;
                        }
                    }
                }

                for (int i=0;i<num_;i++) {
                    if (!vis[i]) {
                        vertices_zone_[zone_B].push_back(vertices_[i]);
                        face_zone_[vertices_[i]] = zone_B;
                    }
                }
            }
        }
    }

    // 利用原网络构建用于最小割的网络, flow_graph_
    // zone_id 是一个 fuzz 区域的 id
    void Build(int zone_id) {
        // 在对每个fuzz zone的最小割求解时，要先清空原来的
        vertices_.clear();
        inv_vertices_.clear();
        flow_graph_.clear();

        // 标记不要添加重复顶点
        bool vis[mNumFaces_] = {false};
        int num = 0;

        // 倒排索引数组
        inv_vertices_.resize(mNumFaces_);

        // 得到fuzz区域相连的两个区域
        int zone_A = (zone_id - k_) / k_;
        int zone_B = (zone_id - k_) % k_;

        // 将模糊区域的顶点加入集合
        for (auto face_idx: solver_->cluster_[zone_id]) {
            if (vis[face_idx]) continue;

            vis[face_idx] = true;
            vertices_.push_back(face_idx);
            inv_vertices_[face_idx] = num;
            num++;
        }

        start_a_ = num;
        
        // 遍历A和B区域，将邻接部分的顶点加入集合
        // A
        for (auto face_idx: solver_->cluster_[zone_A]) {
            // 检测该面是否与模糊区域相邻，并获取相邻的边
            if (!CheckFuzzBoundary(face_idx, zone_id)) continue;

            if (vis[face_idx]) continue;

            vis[face_idx] = true;
            vertices_.push_back(face_idx);
            inv_vertices_[face_idx] = num;
            num++;
            
        }

        start_b_ = num;

        // B
        for (auto face_idx: solver_->cluster_[zone_B]) {
            // 检测该面是否与模糊区域相邻，并获取相邻的边
            if (!CheckFuzzBoundary(face_idx, zone_id)) continue;

            if (vis[face_idx]) continue;

            vis[face_idx] = true;
            vertices_.push_back(face_idx);
            inv_vertices_[face_idx] = num;
            num++;
        }

        // 上面顶点的数量 + 源点和汇点
        fNumVertices_ = num + 2;
        num_ = num;
        flow_graph_.resize(fNumVertices_);

        // 建立用于求解最小割的图
        // 对于所有点，两两之间按照公式建立容量(边)
        for (int i=0;i<num;i++) {
            // 找到该面片相邻的3个面
            std::vector<int> adjacent_faces = utils_->face_neighbour_[vertices_[i]];
            for (auto adj: adjacent_faces) {
                // 如果相邻面在之前统计的顶点中，则需要计算两者的边的容量
                if (vis[adj]) {
                    bool repeat = false;
                    // 检查是否重复
                    for (auto vertex: flow_graph_[i]) {
                        // 获取对应的face_idx
                        int face_idx = vertices_[vertex.first];

                        if (adj == face_idx) {
                            repeat = true;
                            break;
                        }
                    }

                    if (repeat) continue;

                    int idx_adj = inv_vertices_[adj];

                    double capacity = ComputeCapacity(i, idx_adj);
                    // 建立双向边
                    flow_graph_[i][idx_adj] = capacity;
                    flow_graph_[idx_adj][i] = capacity;
                }
            }
        }

        // S和A连接
        for (int i=start_a_;i<start_b_;i++) {
            // num对应s
            flow_graph_[num][i] = INF;
            flow_graph_[i][num] = INF;
        }

        // T和B连接
        for (int i=start_b_;i<num;i++) {
            // num+1 对应 T
            flow_graph_[num + 1][i] = INF;
            flow_graph_[i][num + 1] = INF;
        }
        
        // 完成最小割图的建立
    }

    // 计算最小割网络流图中两个顶点之间的容量
    double ComputeCapacity(int idx_1, int idx_2) {
        // 如果idx_1或idx_2是源点或者汇点
        if (IsSorT(idx_1) || IsSorT(idx_2)) {
            return INF;
        }

        // 获取face_idx
        int face_idx_1 = vertices_[idx_1];
        int face_idx_2 = vertices_[idx_2];

        // 计算角度距离
        double ang_dist = utils_->GetAngleDistance(face_idx_1, face_idx_2);
        double avg_angle_dist = utils_->avg_angle_dist_;

        return 1 / (1 + ang_dist / avg_angle_dist);
    }

    // 判断该索引是不是源点或汇点
    bool IsSorT(int idx) {
        if (idx == num_ + 1 || idx == num_ + 2) return true;
        return false;
    }

    // 检查该面是否与fuzz_zone邻接
    bool CheckFuzzBoundary(int face_idx, int zone_id) {
        // 获取该面的三个邻接面
        // 若邻接面中有属于fuzz_zone的，则是fuzz边界
        std::vector<int> adjacent_face = utils_->face_neighbour_[face_idx];

        for (auto adj_face_idx: adjacent_face) {
            // 如果邻接的面是fuzz_zone
            if (solver_->face_zone_[adj_face_idx] == zone_id) 
                return true;
        }

        return false;
    }

    // 检查该面是否为与fuzz_zone的邻接面，并且返回相邻的边
    std::vector<std::pair<int, int>> GetFuzzBoundary(int face_idx) {
        std::vector<std::pair<int, int>> adjacent_edges;

        // 获取该面的三个邻接面
        // 若邻接面中有属于fuzz_zone的，则是fuzz边界
        std::vector<int> adjacent_face = utils_->face_neighbour_[face_idx];

        for (auto adj_face_idx: adjacent_face) {
            // 如果邻接的面是fuzz_zone
            if (solver_->face_zone_[adj_face_idx] == -1) 
                adjacent_edges.push_back(utils_->edge_between_faces_[face_idx][adj_face_idx]);
        }

        return adjacent_edges;
    }
};

#endif