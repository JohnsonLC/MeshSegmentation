#ifndef H_UTILS_
#define H_UTILS_

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <string>
#include <vector>
#include <map>
#include <algorithm>

#include "logging.h"

class Utils {
public:
    int mNumFaces_;
    int mNumVertices_;

    double avg_geod_dist_;
    double avg_angle_dist_;

    aiMesh* mesh_;
    std::vector<aiVector3D> centers_, normals_;
    std::map<std::pair<int, int>, std::vector<int>> edge_neighbour_;
    std::vector<std::vector<int>> face_neighbour_;
    std::vector<std::map<int, std::pair<int, int>>> edge_between_faces_;
    
    // inv_index_[i]表示索引为i的顶点，所连接的三个面索引
    std::vector<std::vector<int>> inv_index_;
public:

    Utils() {}

    Utils(aiMesh* mesh) {
        mesh_ = mesh;
        mNumFaces_ = mesh_->mNumFaces;
        mNumVertices_ = mesh_->mNumVertices;

        avg_geod_dist_ = 0.0;
        avg_angle_dist_ = 0.0;

        normals_.resize(mNumFaces_);
        centers_.resize(mNumFaces_);
        inv_index_.resize(mNumVertices_);
        edge_between_faces_.resize(mNumFaces_);
        for (int i=0;i<mesh->mNumFaces;i++) {
            normals_[i] = GetFaceNormal(i);
            centers_[i] = GetFaceCenter(i);
            edge_between_faces_[i] = std::map<int, std::pair<int, int>>();

            aiFace face = mesh_->mFaces[i];
            if (face.mNumIndices != 3) {
                // 处理一下
                // **** TODO
            }
            for (int j=0;j<face.mNumIndices;j++) {
                inv_index_[face.mIndices[j]].push_back(i);
            }
        }
    }

    static void PrintUsage() {
        INFO_LOG("Usage: \n\t./mesh_segment [MODEL_PATH] [K_WAY] [EPOSILON] [DRAW_MODE]\n");
    }

    // 用于找到面邻接关系，即返回值face_neighbour[i]存的是 第i个face所有相邻face的索引
    std::vector<std::vector<int>> GetFaceNeighbour() {
        face_neighbour_.resize(mNumFaces_);

        // 先找边邻接关系
        edge_neighbour_ = GetEdgeNeighbour();
        for (auto m: edge_neighbour_) {
            // 因为一个边应该只能连接两个三角面，所以若不等于2，则有问题
            if (m.second.size() != 2) {
                ERROR_LOG("One edge just join two triangle faces !");
                return std::vector<std::vector<int>>();
            }

            // 获取边连接的两个三角面的索引
            int fu = m.second[0];
            int fv = m.second[1];

            edge_between_faces_[fu][fv] = m.first;
            edge_between_faces_[fv][fu] = m.first;

            // printf("fu: %d,\tfv: %d\n", fu, fv);

            // 这两个三角面互为邻居
            face_neighbour_[fu].push_back(fv);
            face_neighbour_[fv].push_back(fu);
        }

        // 返回邻接结果
        return face_neighbour_;
    }

    // 用于找到边邻接关系，map的first是某条边(因为用升序处理，所以以pair代替)，second是这条边所链接的两个三角面的索引
    std::map<std::pair<int, int>, std::vector<int>> GetEdgeNeighbour() {
        std::map<std::pair<int, int>, std::vector<int>> mmap;

        // 找到每个mesh中的face
        for (int j=0;j<mesh_->mNumFaces;j++) {
            aiFace face = mesh_->mFaces[j];

            // 如果不是三角面怎么办
            if (face.mNumIndices != 3) {
                // **** TODO
                // 警告或怎么样
            }

            // 获取每个face的三个点
            std::vector<int> vertices;
            for (int i=0;i<face.mNumIndices;i++) {
                vertices.push_back(face.mIndices[i]);
            }

            // 先对三个点按升序排列，保证pair的唯一性
            std::sort(vertices.begin(), vertices.end());

            // 先遍历前两对点，因为是升序
            int num = vertices.size() - 1;
            for (int i=0;i<num;i++) {
                // 边的点对
                std::pair<int, int> p = std::make_pair(vertices[i], vertices[i + 1]);
                // 如果该边还没有在map中
                if (mmap.find(p) == mmap.end()) {
                    // 建立vector
                    mmap[p] = std::vector<int>();
                }
                // 将该face的index加入vector
                mmap[p].push_back(j);
            }

            // 对于最后一对同理
            std::pair<int, int> p = std::make_pair(vertices[0], vertices[num]);
            // 如果该边还没有在map中
            if (mmap.find(p) == mmap.end()) {
                // 建立vector
                mmap[p] = std::vector<int>();
            }
            // 将该face的index加入vector
            mmap[p].push_back(j);
        }
        
        return mmap;
    }

    // 计算某个三角面片的面心坐标
    aiVector3D GetFaceCenter(int face_idx) {
        std::vector<aiVector3D> p;

        aiFace face = mesh_->mFaces[face_idx];
        for (int i=0;i<face.mNumIndices;i++) {
            p.push_back(mesh_->mVertices[face.mIndices[i]]);
            // printf("Vertex: (\t%f,\t%f,\t%f).\n", mesh->mVertices[face.mIndices[i]][0], mesh->mVertices[face.mIndices[i]][1], mesh->mVertices[face.mIndices[i]][2]);
        }

        if (p.size() != 3) {
            // 如果一个面不包含3个顶点
            // **** TODO
        }

        aiVector3D center = p[0];
        center += p[1];
        center += p[2];

        center /= 3;
        // printf("Center: (\t%f,\t%f,\t%f).\n", center[0], center[1], center[2]);
        return center;
    }

    // 计算某个三角面片的法线，对于Assimp来说是顺时针的三角面片
    //  v1 = p1 - p2, v2 = p0 - p1
    //  法线方向为 v1 × v2
    //  按照叉乘公式展开，再进行归一化
    aiVector3D GetFaceNormal(int face_idx) {
        std::vector<aiVector3D> p;

        aiFace face = mesh_->mFaces[face_idx];
        for (int i=0;i<face.mNumIndices;i++) {
            p.push_back(mesh_->mVertices[face.mIndices[i]]);
        }

        if (p.size() != 3) {
            // 如果一个面不包含3个顶点
            // **** TODO
        }

        aiVector3D v1 = p[0];
        aiVector3D v2 = p[1];

        // 利用 Assimp 提供的数据结构做减法
        v1 -= p[1];
        v2 -= p[2];

        aiVector3D normal = CrossProduct(v1, v2);
        // 归一化为单位向量
        normal.Normalize();

        return normal;
    } 

    // 判断二面角是否为凸
    bool isConvex(int face_idx_1, int face_idx_2) {
        aiVector3D c12 = centers_[face_idx_2] - centers_[face_idx_1];
        return GetInnerProduct(c12, normals_[face_idx_1]) < 0;
    }

    // 计算两个向量的叉积
    // a = (x1, y1, z1), b = (x2, y2, z2)
    // a × b = (y1*z2 - z1*y2, z1*x2 - x1*z2, x1*y2 - x2*y1)
    static aiVector3D CrossProduct(aiVector3D a, aiVector3D b) {
        double x = a[1] * b[2] - a[2] * b[1];
        double y = a[2] * b[0] - a[0] * b[2];
        double z = a[0] * b[1] - a[1] * b[0];

        return aiVector3D(x, y, z);
    }

    // 计算相邻三角面之间的距离，包括了测地距离和角距离
    double GetFacesDistance(int face_idx_1, int face_idx_2, double delta = 0.5) {
        // 如果还没有计算过距离，则计算距离
        if (avg_geod_dist_ == 0 || avg_angle_dist_ == 0) {
            GetAvgAngleDistance();
            GetAvgGeodesicDistance();
        }

        double geod_dist = GetGeodesicDistance(face_idx_1, face_idx_2);
        double ang_dist = GetAngleDistance(face_idx_1, face_idx_2);

        return delta * geod_dist / avg_geod_dist_ + (1 - delta) * ang_dist / avg_angle_dist_;
    }

    // 计算两相邻面片的测地距离，原理见 https://www.cnblogs.com/graphics/archive/2012/08/10/2627458.html
    double GetGeodesicDistance(int face_idx_1, int face_idx_2) {
        // 获取两个三角面片相接的边
        std::pair<int, int> edge = edge_between_faces_[face_idx_1][face_idx_2];
        if (edge.first == 0 && edge.second == 0) {
            ERROR_LOG("Cannot compute the geodesic distance between faces which is not neighbour.");
            return -1;
        }

        // 获取边的两个顶点并转化为Eigen的Vector
        aiVector3D v1 = mesh_->mVertices[edge.first];
        aiVector3D v2 = mesh_->mVertices[edge.second];
        aiVector3D c1 = centers_[face_idx_1];
        aiVector3D c2 = centers_[face_idx_2];

        Eigen::Vector3d v1_eigen(v1[0], v1[1], v1[2]);
        Eigen::Vector3d v2_eigen(v2[0], v2[1], v2[2]);
        Eigen::Vector3d c1_eigen(c1[0], c1[1], c1[2]);
        Eigen::Vector3d c2_eigen(c2[0], c2[1], c2[2]);

        Eigen::Vector3d axis = v2_eigen - v1_eigen;
        Eigen::Vector3d a = c1_eigen - v1_eigen;
        Eigen::Vector3d b = c2_eigen - v1_eigen;

        double len_axis = axis.norm();
        double len_a = a.norm();
        double len_b = b.norm();
        
        double angle_a = acos(a.dot(axis) / len_a / len_axis);
        double angle_b = acos(b.dot(axis) / len_b / len_axis);

        double len = len_a*len_a + len_b*len_b - 2*len_a*len_b*cos(angle_a + angle_b);

        return sqrt(len);
    }

    // 得到绕任意轴旋转的旋转矩阵
    // 其中旋转轴是 v2 - v1，旋转角度是 theta
    // 参考 https://www.cnblogs.com/graphics/archive/2012/08/10/2627458.html
    static Eigen::Matrix4d GetRotationMatrix(Eigen::Vector3d v1, Eigen::Vector3d v2, double theta) {
        // 旋转轴是 v2 - v1
        Eigen::Vector3d axis = v2 - v1;

        // 计算一下 sintheta
        double costheta = cos(theta);
        double sintheta = sin(theta);

        // 设定一些需要的数，方便后面使用
        double u = axis[0];
        double v = axis[1];
        double w = axis[2];

        double a = v1[0];
        double b = v1[1];
        double c = v1[2];

        double r11 = u*u + (v*v + w*w)*costheta;
        double r12 = u*v*(1 - costheta) - w*sintheta;
        double r13 = u*w*(1 - costheta) + v*costheta;
        double r14 = (a*(v*v + w*w) - u*(b*v + c*w))*(1 - costheta) + (b*w - c*v)*sintheta;
        double r21 = u*v*(1 - costheta) + w*sintheta;
        double r22 = v*v + (u*u + w*w)*costheta;
        double r23 = v*w*(1 - costheta) - u*sintheta;
        double r24 = (b*(u*u + w*w) - v*(a*u + c*w))*(1 - costheta) + (c*u - a*w)*sintheta;
        double r31 = u*w*(1 - costheta) - v*sintheta;
        double r32 = v*w*(1 - costheta) + u*sintheta;
        double r33 = w*w + (u*u + v*v)*costheta;
        double r34 = (c*(u*u + v*v) - w*(a*u + b*v))*(1 - costheta) + (a*v - b*u)*sintheta;
        double r41 = 0, r42 = 0, r43 = 0, r44 = 1;

        Eigen::Matrix4d rotation_matrix;
        rotation_matrix << r11, r12, r13, r14,
                            r21, r22, r23, r24,
                            r31, r32, r33, r34,
                            r41, r42, r43, r44;
                        
        return rotation_matrix;
    }

    // 计算两个向量的内积
    // 
    static double GetInnerProduct(aiVector3D a, aiVector3D b) {
        double inner_product = a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
        return inner_product;
    }

    // 计算两个坐标之间的欧式距离
    //
    static double GetEuclideanDistance(aiVector3D a, aiVector3D b) {
        a -= b;
        double norm2 = a[0]*a[0] + a[1]*a[1] + a[2]*a[2];
        return sqrt(norm2);
    }

    // 计算两个相邻面片之间的角度距离
    double GetAngleDistance(int face_idx_1, int face_idx_2, double eposilon = 0.2) {
        double eta = eposilon;
        if (!isConvex(face_idx_1, face_idx_2)) eta = 1;
        return eta * (1 - GetInnerProduct(normals_[face_idx_1], normals_[face_idx_2]));
    }

    // 获取所有相邻面片的平均角距离
    double GetAvgAngleDistance() {
        int num = 0;
        double angle_dist_sum = 0.0;
        for (auto m: edge_neighbour_) {
            int face_idx_1 = m.second[0];
            int face_idx_2 = m.second[1];

            angle_dist_sum += GetAngleDistance(face_idx_1, face_idx_2);
            num++;
        }

        return avg_angle_dist_ = angle_dist_sum / num;
    }

    // 获取所有相邻面片的平均测地距离
    double GetAvgGeodesicDistance() {
        int num = 0;
        double geod_dist_sum = 0.0;
        for (auto m: edge_neighbour_) {
            int face_idx_1 = m.second[0];
            int face_idx_2 = m.second[1];

            geod_dist_sum += GetGeodesicDistance(face_idx_1, face_idx_2);
            num++;
        }

        return avg_geod_dist_ = geod_dist_sum / num;
    }

    // 获得点到平面的投影点，计算公式参考 https://www.cnblogs.com/nobodyzhou/p/6145030.html
    // 平面由一个点和法向量描述
    // 
    // Input:
    //      point:      待投影点
    //      mesh:       网格，用于获取face以及vertices等
    //      centers:    各三角面片的面心坐标
    //      normals:    各三角面片的法线坐标
    //      face_idx:   投影平面的index
    // Output:
    //      返回计算得到的投影点坐标
    aiVector3D GetProjection(aiVector3D point, int face_idx) {
        // 将一些参数按照公式设置好
        aiFace face = mesh_->mFaces[face_idx];
        double x0 = centers_[face_idx][0];
        double y0 = centers_[face_idx][1];
        double z0 = centers_[face_idx][2];

        double a = normals_[face_idx][0];
        double b = normals_[face_idx][1];
        double c = normals_[face_idx][2];

        double x = point[0];
        double y = point[1];
        double z = point[2];

        double t = a*x0 + b*y0 + c*z0 - (a*x + b*y + c*z);
        t /= (a*a + b*b + c*c);

        return aiVector3D(x + a*t, y + b*t, z + c*t);
    }
};

#endif
