#ifndef H_CLUSTER_
#define H_CLUSTER_

#include "logging.h"
#include "graph.h"
#include "utils.h"

class FuzzClusterSolver {
public:
    // 聚类中心
    // int center_a_, center_b_;
    std::vector<int> centers_;
    // 各类中的元素（包括 fuzz 区域）
    std::vector<std::vector<int>> cluster_;
    // 记录每个face属于哪个区域
    std::vector<int> face_zone_;
    // 每个面片属于不同区域的概率, probabilities[i][j]表示 i 面片在第 j 个 center 的概率
    std::vector<std::vector<double>>  probabilities_;
    // 各个面片之间的最短距离
    Graph* graph_;

    int mNumFaces_;
    // 计算面片对不同区域的概率距离乘积和
    std::vector<double> sum_pds_;
    
    // 最大迭代次数
    int iter_max_;
    double eposilon_;
    int k_;
public:

    FuzzClusterSolver() {}

    // 初始化模糊聚类求解器
    void Init(Graph* graph, int k = 2, int iter = 20, double eposilon = 0.1) {
        graph_ = graph;

        mNumFaces_ = graph_->mNumFaces_;
        k_ = k;
        iter_max_ = iter;
        eposilon_ = eposilon;
        // printf("eposilon: %f\n", eposilon_);

        cluster_.resize(k_ + k_ * k_);
        // centers_.resize(k_);
        sum_pds_.resize(k_);
        for (int i=0;i<k_;i++) {
            sum_pds_[i] = INF;
        }
        face_zone_.resize(mNumFaces_);

        // 给聚类中心赋值
        if (k_ == 2) {
            std::pair<int, int> most_far_faces = graph_->GetMostFarFaces();
            centers_.push_back(most_far_faces.first);
            centers_.push_back(most_far_faces.second);
        }
        else {
            GetInitCenter();
        }

        // 计算最初的分类
        Cluster();
    }

    // 得到最初的聚类中心
    void GetInitCenter() {
        // 先选出第一个聚类中心
        double mmin = INF;
        int imin = -1;
        for (int i=0;i<mNumFaces_;i++) {
            double dsum = 0.0;
            for (int j=0;j<mNumFaces_;j++) {
                dsum += graph_->dist_[i][j];
            }
            
            if (dsum < mmin) {
                mmin = dsum;
                imin = i;
            }
        }

        // 加入第一个聚类中心
        centers_.push_back(imin);

        // 求得其他 k-1 个聚类中心
        for (int i=1;i<k_;i++) {
            // 求最小距离最大的点
            double mmax = -INF;
            int imax = -1;
            for (int j=0;j<mNumFaces_;j++) {
                // 求每个面距离不同聚类中心的最小距离
                double mmin = INF;
                for (int k=0;k<i;k++) {
                    double d = graph_->dist_[j][centers_[k]];
                    if (d < mmin) {
                        mmin = d;
                    }
                }

                if (mmin > mmax) {
                    mmax = mmin;
                    imax = j;
                }
            }

            centers_.push_back(imax);
        }

        // 求得 k 个聚类中心
        printf("Centers: \n");
        for (auto center: centers_) {
            printf("%d, ", center);
        }
        printf("\n");
    }    

    // 计算各个面片属于各类的概率
    void Cluster() {
        probabilities_.clear();
        for (int i=0;i<mNumFaces_;i++) {
            // 如果是某个中心，则直接设置该区域概率为1，其他概率为0
            std::vector<double> prob(k_, 0);
            bool has_equal = false;
            for (int j=0;j<k_;j++) {
                if (i == centers_[j]) {
                    prob[j] = 1;
                    has_equal = true;
                    break;
                }
            }

            if (!has_equal) {
                // 该面到所有center的距离
                std::vector<double> inv_dist(k_);
                for (int j=0;j<k_;j++) {
                    inv_dist[j] = 1 / graph_->dist_[i][centers_[j]];
                }  

                double sum = 0.0;
                for (int j=0;j<k_;j++) {
                    sum += inv_dist[j];
                }

                for (int j=0;j<k_;j++) {
                    double p = inv_dist[j] / sum;
                    prob[j] = p;
                }
            }

            probabilities_.push_back(prob);
        }
    }

    // 对应于论文中的，精化的距离概率计算，后续实现
    // void RecitifyCluster() {
    // 
    // }

    // 解决模糊聚类问题
    bool Solve() {
        // 如果第一次就直接选对了，就不用再继续了
        if (!UpdateCenter()) {
            Classitify();
            return true;
        }

        for (int i=0;i<iter_max_;i++) {
            // 计算各个面到各个区域的概率
            Cluster();
            // 重新计算种子
            if (!UpdateCenter()) break;
        }

        Classitify();

        return true;
    }

    // 统计fuzz区域和两个cluster的点
    // 仅仅对每个面找最大概率和次大概率即可，则该面的标签就根据这两个来确定
    void Classitify() {
        for (int i=0;i<mNumFaces_;i++) {
            for (int i=0;i<mNumFaces_;i++) {
                double max_1 = -INF;
                double max_2 = -INF;
                int imax_1 = -1;
                int imax_2 = -1;

                for (int j=0;j<k_;j++) {
                    if (max_1 < probabilities_[i][j]) {
                        max_2 = max_1;
                        imax_2 = imax_1;
                        max_1 = probabilities_[i][j];
                        imax_1 = j;
                    }
                    else if (max_2 < probabilities_[i][j]) {
                        max_2 = probabilities_[i][j];
                        imax_2 = j;
                    }
                }

                // 不属于模糊区域
                if (max_1 - max_2 > eposilon_) {
                    face_zone_[i] = imax_1;
                    cluster_[imax_1].push_back(i);
                }
                else {
                    face_zone_[i] = k_ + (imax_1 * k_ + imax_2);
                    cluster_[k_ + (imax_1 * k_ + imax_2)].push_back(i);
                }
            }
        }
    }

    // 更新聚类中心
    bool UpdateCenter() {
        bool bChange = false;

        // 这里直接暴力用了O(F2)的算法，后续可以考虑改进一下
        std::vector<double> min_pds = sum_pds_;
        for (int k=0;k<k_;k++) {
            bool bTmp = false;
            int imin = -1;

            for (int i=0;i<mNumFaces_;i++) {
                double sum_pd = 0.0;
                for (int j=0;j<mNumFaces_;j++) {
                    sum_pd += probabilities_[j][k] * graph_->dist_[i][j];
                }
                
                if (sum_pd < min_pds[k]) {
                    min_pds[k] = sum_pd;
                    imin = i;
                    bTmp = true;
                }
            }

            if (bTmp) {           
                // 保存当前的最小sum值
                sum_pds_[k] = min_pds[k];
                if (imin != centers_[k]) {
                    centers_[k] = imin;
                    bChange = true;
                }
            }
        }

        // printf("seed_a: %d\tsum_a: %f\tseed_b: %d\tsum_b: %f\n", center_a_, sum_pd_a_, center_b_, sum_pd_b_);
        printf("Update Center: \n");
        for (auto center: centers_) {
            printf("%d, ", center);
        }
        printf("\n");

        return bChange;
    }

};

#endif
