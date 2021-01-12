#ifndef H_DRAW_
#define H_DRAW_

#include <cstdlib>
#include <ctime>

#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/freeglut_ext.h>
#include <GL/freeglut_std.h>

#include <assimp/scene.h>

#include "logging.h"
#include "utils.h"
#include "graph.h"
#include "cluster.h"
#include "Angel.h"
#include "cut.h"

typedef Angel::vec4  color4;
typedef Angel::vec4  point4;

int NumVertices; //(6 faces)(2 triangles/face)(3 vertices/triangle)

point4* points;
color4* colors;

// Array of rotation angles (in degrees) for each coordinate axis
enum { Xaxis = 0, Yaxis = 1, Zaxis = 2, NumAxes = 3 };
int     Axis = Xaxis;
GLfloat Theta[NumAxes] = { 0.0, 0.0, 0.0 };
GLfloat zoom = 1.0;

// OpenGL initialization
void
init()
{
    // Create and initialize a buffer object
    GLuint buffer;
    glGenBuffers( 1, &buffer );
    glBindBuffer( GL_ARRAY_BUFFER, buffer );
    glBufferData( GL_ARRAY_BUFFER, (sizeof(point4) + sizeof(color4)) * NumVertices,
		  NULL, GL_STATIC_DRAW );
    glBufferSubData( GL_ARRAY_BUFFER, 0, sizeof(point4) * NumVertices, points );
    glBufferSubData( GL_ARRAY_BUFFER, sizeof(point4) * NumVertices, sizeof(color4) * NumVertices, colors );

    // Load shaders and use the resulting shader program
    GLuint program = InitShader( "vshader.glsl", "fshader.glsl" );
    glUseProgram( program );

    // set up vertex arrays
    GLuint vPosition = glGetAttribLocation( program, "vPosition" );
    glEnableVertexAttribArray( vPosition );
    glVertexAttribPointer( vPosition, 4, GL_FLOAT, GL_FALSE, 0,
			   BUFFER_OFFSET(0) );

    GLuint vColor = glGetAttribLocation( program, "vColor" ); 
    glEnableVertexAttribArray( vColor );
    glVertexAttribPointer( vColor, 4, GL_FLOAT, GL_FALSE, 0,
			   BUFFER_OFFSET(sizeof(point4) * NumVertices) );

    glEnable( GL_DEPTH_TEST );
    glClearColor( 1.0, 1.0, 1.0, 1.0 ); 
}

//----------------------------------------------------------------------------

void
display( void )
{
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

    mat4  transform = ( RotateX( Theta[Xaxis] ) *
			RotateY( Theta[Yaxis] ) *
			RotateZ( Theta[Zaxis] ) );

    point4  transformed_points[NumVertices];

    for ( int i = 0; i < NumVertices; ++i ) {
	    transformed_points[i] = transform * points[i];
        points[i] *= zoom; points[i].w = 1;
    }
    zoom = 1.0;

    glBufferSubData( GL_ARRAY_BUFFER, 0, sizeof(transformed_points),
		     transformed_points );

    
    glDrawArrays( GL_TRIANGLES, 0, NumVertices );
    //glDrawArrays( GL_LINES, 0, NumVertices );
    glutSwapBuffers();
}

//----------------------------------------------------------------------------

void
keyboard( unsigned char key, int x, int y )
{
    switch( key ) {
        case 033: // Escape Key
        case 'q': case 'Q':
            exit( EXIT_SUCCESS );
            break;
        case 'w': case 'W':
            zoom += 0.03;
            glutPostRedisplay();
            break;
        case 's': case 'S':
            zoom -= 0.03;
            glutPostRedisplay();
            break;
    }
}

//----------------------------------------------------------------------------

void
mouse( int button, int state, int x, int y )
{
    if ( state == GLUT_DOWN ) {
        switch( button ) {
            case GLUT_LEFT_BUTTON:    Axis = Xaxis;  break;
            case GLUT_MIDDLE_BUTTON:  Axis = Yaxis;  break;
            case GLUT_RIGHT_BUTTON:   Axis = Zaxis;  break;
        }
    }
}

//----------------------------------------------------------------------------

void
idle( void )
{
    Theta[Axis] += 0.5;

    if ( Theta[Axis] > 360.0 ) {
	Theta[Axis] -= 360.0;
    }
    
    glutPostRedisplay();
}

// Drawer 类
class Drawer {
public:
    Utils* utils_;
    CutSolver* cut_;
    FuzzClusterSolver* solver_;
    aiMesh* mesh_;

    std::vector<color4> rgbs_;

    int mNumFaces_;
    int k_;

    int mode_;      // 0: 绘制fuzz之前的样子
                    // 1: 绘制fuzz
                    // 2: 绘制最小割之后的样子(默认)
public:
    Drawer() {}

    Drawer(Utils* utils, FuzzClusterSolver* solver, CutSolver* cut, int mode = 2) {
        utils_ = utils;
        solver_ = solver;
        cut_ = cut;
        mNumFaces_ = utils_->mNumFaces_;
        mesh_ = utils_->mesh_;

        NumVertices = mNumFaces_ * 3;
        k_ = solver_->k_;
        mode_ = mode;

        // 随机生成 k 种颜色用于显示
        GenerateKColorsRnd();
        // GenerateKColorsFixed();

        if (!cut_ && mode_ == 2) {
            printf("Cut is NULL, change default mode to 1.\n");
            mode_ = 1;
        }

        GenerateVertexAndColor();
    }

    // k <= 8
    void GenerateKColorsFixed() {
        rgbs_.push_back(color4(0, 0, 0, 1));
        rgbs_.push_back(color4(0, 0, 1, 1));
        rgbs_.push_back(color4(0, 1, 0, 1));
        rgbs_.push_back(color4(0, 1, 1, 1));
        rgbs_.push_back(color4(1, 0, 0, 1));
        rgbs_.push_back(color4(1, 0, 1, 1));
        rgbs_.push_back(color4(1, 1, 0, 1));
        rgbs_.push_back(color4(1, 1, 1, 1));
    }

    void GenerateKColorsRnd() {
        std::srand((unsigned)time(NULL));
        // 随机生成 k 种颜色
        for (int i=0;i<k_;i++) {
            double r = rand() % 256;
            double g = rand() % 256;
            double b = rand() % 256;

            rgbs_.push_back(color4(r / 255.0, g / 255.0, b / 255.0, 1));
        }
    }

    // 生成顶点和颜色
    void GenerateVertexAndColor() {
        // 建立顶点数组和颜色数组
        points = new point4[NumVertices];
        colors = new color4[NumVertices];
        // i / 3 是面的索引
        for (int i=0;i<mNumFaces_;i++) {
            aiFace face = mesh_->mFaces[i];
            // 遍历三个顶点
            for (int j=0;j<face.mNumIndices;j++) {
                // 获取顶点坐标对应的索引
                int vertex_idx = i * 3 + j;

                aiVector3D pose = mesh_->mVertices[face.mIndices[j]];
                points[vertex_idx] = point4(pose[0], pose[1], pose[2], 1);
                colors[vertex_idx] = ComputeColor(face.mIndices[j]);
            }
        }
    }

    // 计算顶点颜色
    color4 ComputeColor(int v_idx) {
        // 获取该顶点连接的三角面的索引
        std::vector<int> inv_index = utils_->inv_index_[v_idx];
        if (inv_index.size() == 0) return color4(0, 0, 0, 0);

        color4 res_color(0, 0, 0, 0);
        if (mode_ == 0) {
            // printf("v_idx:\t%d", v_idx);
            // 遍历三个三角面，计算加权颜色
            int num = 0;
            for (auto face_idx: inv_index) {
                // printf("\tface_idx:\t%d", face_idx);
                color4 c(0, 0, 0, 0);
                for (int i=0;i<k_;i++) {
                    double p = solver_->probabilities_[face_idx][i];
                    c += (p * rgbs_[i]);
                }
                res_color += c;
                num++;
            }
            // printf("\n");

            res_color /= num;
        }
        else if (mode_ == 1) {
            int num[k_ + 1] = {0};
            for (auto face_idx: inv_index) {
                int label = solver_->face_zone_[face_idx];
                if (label < k_) num[label]++;   // label区域
                else num[k_]++;              // fuzz区域
            }

            int mmax = -INF;
            int imax = -1;
            for (int i=0;i<=k_;i++) {
                if (num[i] > mmax) {
                    mmax = num[i];
                    imax = i;
                }
            }

            if (imax == k_) {
                res_color = color4(1, 0, 0, 1);
            }
            else {
                res_color = rgbs_[imax];
            }

            // printf("r: %d\tb: %d\tg: %d\n", nums[2], nums[1], nums[0]);
            // res_color = color4(nums[2], nums[1], nums[0], 1);
        }
        else if (mode_ == 2) {
            int nums[k_] = {0};
            for (auto face_idx: inv_index) {
                int label = cut_->face_zone_[face_idx];
                nums[label]++;
            }

            int mmax = -INF;
            int imax = -1;
            for (int i=0;i<k_;i++) {
                if (nums[i] > mmax) {
                    mmax = nums[i];
                    imax = i;
                }
            }

            // printf("r: %d\tb: %d\tg: %d\n", nums[2], nums[1], nums[0]);
            res_color = rgbs_[imax];
        }

        return res_color;
    }

    // 绘制展示
    void Show( int argc, char **argv )
    {
        glutInit( &argc, argv );
        glutInitDisplayMode( GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH );
        glutInitWindowSize( 1024, 1024 );
        glutCreateWindow( "Color Cube" );

        glewExperimental = GL_TRUE;
        glewInit();

        init();

        glutDisplayFunc( display );
        glutKeyboardFunc( keyboard );
        glutMouseFunc( mouse );
        glutIdleFunc( idle );

        glutMainLoop();
    }

};

#endif