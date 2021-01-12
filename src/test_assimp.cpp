#include <iostream>
#include <cmath>

#include <assimp/Importer.hpp>      // 导入器在该头文件中定义
#include <assimp/scene.h>           // 读取到的模型数据都放在scene中
#include <assimp/postprocess.h>     // 该头文件中包含后处理的标志位定义

#include <eigen3/Eigen/Core>

#include "logging.h"
#include "utils.h"

using namespace std;

int main(int argc, char* argv[]) {

    string model_path = argv[1];

    // 定义一个导入器 
    Assimp::Importer importer;   
    
    // 使用导入器导入选定的模型文件 
    const aiScene* scene = importer.ReadFile( model_path,
        aiProcess_CalcTangentSpace|            //后处理标志，自动计算切线和副法线
        aiProcess_Triangulate|                //后处理标志，自动将四边形面转换为三角面
        aiProcess_JoinIdenticalVertices|    //后处理标志，自动合并相同的顶点
        aiProcess_SortByPType);                //后处理标志，将不同图元放置到不同的模型中去，图片类型可能是点、直线、三角形等
                                            //更多后处理标志可以参考Assimp的文档  
    if(!scene)   
    {
        //导入错误，获取错误信息并进行相应的处理
        //DoTheErrorLogging( importer.GetErrorString());     
        return false;  
    }

    aiVector3D v1(1, 1, 1);
    aiVector3D v2(-1, -1, -1);

    printf("Distance between v1 and v2: %f\n", Utils::GetEuclideanDistance(v1, v2)); 

    // 看一下mesh
    int num = 0;
    for (int i=0;i<scene->mNumMeshes;i++) {
        aiMesh* mesh = scene->mMeshes[i];
        for (int j=0;j<mesh->mNumVertices;j++) {
            aiVector3D v = mesh->mVertices[j];
            printf("Vertex %d: (\t%f,\t%f,\t%f).\n", j, v[0], v[1], v[2]);
        }
    }

    Eigen::Vector3d a(1, 1, 1);
    cout << "vector norm: " << a.dot(a) << endl;

    // cout << "total faces: " << num << endl;

    return 0;
}
