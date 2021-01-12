#include <iostream>
#include <string>
#include <sstream>

#include <assimp/Importer.hpp>      // 导入器在该头文件中定义
#include <assimp/scene.h>           // 读取到的模型数据都放在scene中
#include <assimp/postprocess.h>     // 该头文件中包含后处理的标志位定义

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <GL/freeglut_ext.h>

#include <GL/glut.h>
#include <GL/gl.h>

#include "utils.h"
#include "logging.h"
#include "graph.h"
#include "cluster.h"
#include "draw.h"
#include "cut.h"

Utils* utils;
FuzzClusterSolver* solver;
Graph* graph;
Drawer* drawer;
CutSolver* cut;

template<typename T>
T string2other(std::string s) {
    T res;
    std::stringstream ss;
    ss << s;
    ss >> res;
    return res;
}

int main(int argc, char* argv[]) {

    if (argc != 5) {
        Utils::PrintUsage();
        return 1;
    }

    std::string model_path = argv[1];
    int k = string2other<int>(argv[2]);
    double eposilon = string2other<double>(argv[3]);
    int draw_mode = string2other<int>(argv[4]);

    printf("k: %d\n", k);
    printf("eposilon: %f\n", eposilon);

    Assimp::Importer importer;

    // 使用导入器导入选定的模型文件 
    const aiScene* scene = importer.ReadFile(model_path,
        aiProcess_CalcTangentSpace|            //后处理标志，自动计算切线和副法线
        aiProcess_Triangulate|                //后处理标志，自动将四边形面转换为三角面
        aiProcess_JoinIdenticalVertices|    //后处理标志，自动合并相同的顶点
        aiProcess_SortByPType);                //后处理标志，将不同图元放置到不同的模型中去，图片类型可能是点、直线、三角形等
                                            //更多后处理标志可以参考Assimp的文档  
    if(!scene)   
    {
        ERROR_LOG("Model cannot be imported successfully. Please check path.");    
        return false;  
    }

    // 给几个常用的变量赋值
    if (!scene->mNumMeshes) {
        ERROR_LOG("No meshed in scene.");
        return 1;
    }
    if (scene->mNumMeshes > 1) {
        ERROR_LOG("This program just support one mesh.");
        return 1;
    }

    int mNumFaces = scene->mMeshes[0]->mNumFaces;
    int mNumVertices = scene->mMeshes[0]->mNumVertices;
    aiMesh* mesh = scene->mMeshes[0];

    utils = new Utils(mesh);
    solver = new FuzzClusterSolver();

     // 找到每个面片的所有邻居（3个）
    std::vector<std::vector<int>> face_neighbour =  utils->GetFaceNeighbour();

    graph = new Graph(utils);

    // 计算最短距离矩阵
    // graph->floyd();
    graph->Dijkstra();

    // 得到距离最远的两个面
    // std::pair<int, int> most_far_faces = graph->GetMostFarFaces();

    // 初始化solver
    solver->Init(graph, k, 20, eposilon);
    solver->Solve();

    // 最大流
    cut = new CutSolver(utils, graph, solver);
    cut->Solve();

    // 建立Drawer绘制模型
    drawer = new Drawer(utils, solver, cut, draw_mode);
    drawer->Show(argc, argv);

    // printf("max_flow: %f\n", max_flow);

    return 0;
}
