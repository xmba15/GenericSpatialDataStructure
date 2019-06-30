/**
 * @file    ExOctreeOpenGL.cpp
 *
 * @author  btran
 *
 * @date    2019-09-10
 *
 *
 * Copyright (c) organization
 *
 */

#include <iostream>

#include <GL/freeglut.h>

#include <spatial_partioning/Octree.hpp>
#include <spatial_partioning/PointCloud.hpp>
#include <spatial_partioning/utility/PCDInOut.hpp>

static algo::Octree<double, algo::geometry::PointCloud::PointType> octree;
static algo::geometry::PointCloud pointcloud;

void drawFunc(const std::array<double, 3>& center, const double radius);

void drawScene(void)
{
    glClear(GL_COLOR_BUFFER_BIT);

    glColor3f(0.0, 0.0, 1.0f);

    // glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    // glBegin(GL_TRIANGLE_STRIP);
    // glVertex3f(10.0, 90.0, 0.0);
    // glVertex3f(10.0, 10.0, 0.0);
    // glVertex3f(35.0, 75.0, 0.0);
    // glVertex3f(30.0, 20.0, 0.0);
    // glVertex3f(90.0, 90.0, 0.0);
    // glVertex3f(80.0, 40.0, 0.0);
    // glEnd();

    // octree.drawOctree(drawFunc);

    glBegin(GL_POINTS);
    for (const auto& point : pointcloud.points_) {
        // glVertex3dv(point.data());
        glVertex3d(point[0], point[1], point[2]);
    }
    glEnd();

    glBegin(GL_POINTS);
    glVertex3d(50, 50, 40);
    glEnd();

    glFlush();
}

void setup(void)
{
    glClearColor(1.0, 1.0, 1.0, 0.0);
}

void resize(int w, int h)
{
    glViewport(0, 0, w, h);

    // ② 視野領域の決定
    glMatrixMode(GL_PROJECTION);  // 射影行列を操作する
    glLoadIdentity();             // 行列を初期化
    gluPerspective(60.0, static_cast<double>(w) / h, 1.0, 100.0);

    // ③ 視点位置の決定
    glMatrixMode(GL_MODELVIEW);  // モデルビュー行列の設定
    glLoadIdentity();            // 行列を初期化
    gluLookAt(7.0, 5.0, 3.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
}

void keyInput(unsigned char key, int x, int y)
{
    switch (key) {
        case 27:
            exit(0);
            break;
        default:
            break;
    }
}

void draw(int argc, char** argv)
{
    glutInit(&argc, argv);

    glutInitWindowSize(800, 600);
    glutInitWindowPosition(100, 100);

    glutCreateWindow("draw octree");

    glutDisplayFunc(drawScene);
    glutReshapeFunc(resize);
    glutKeyboardFunc(keyInput);
}

int main(int argc, char* argv[])
{
#ifdef DATA_PATH
    std::stringstream ss;
    ss << DATA_PATH << "/easy_jet.pcd";

    algo::pcdio::ReadPointCloud(ss.str(), pointcloud, "auto", true, true, true);

    octree = algo::Octree<double, algo::geometry::PointCloud::PointType>(pointcloud.points_);

    draw(argc, argv);

    setup();
    glutMainLoop();
#endif  // DATA_PATH

    return 0;
}

void drawFunc(const std::array<double, 3>& center, const double radius)
{
    std::array<double, 3> vertices[] = {
        {center[0] + radius, center[1] - radius, center[2] - radius},
        {center[0] + radius, center[1] + radius, center[2] - radius},

        {center[0] - radius, center[1] + radius, center[2] - radius},
        {center[0] - radius, center[1] - radius, center[2] - radius},

        {center[0] + radius, center[1] - radius, center[2] + radius},
        {center[0] + radius, center[1] + radius, center[2] + radius},

        {center[0] - radius, center[1] - radius, center[2] + radius},
        {center[0] - radius, center[1] + radius, center[2] + radius},
    };

    std::array<int, 2> edges[] = {{0, 1}, {0, 3}, {0, 4}, {2, 1}, {2, 3}, {2, 7},
                                  {6, 3}, {6, 4}, {6, 7}, {5, 1}, {5, 4}, {5, 7}};

    glBegin(GL_LINES);
    {
        for (const auto& edge : edges) {
            for (int vertexIdx : edge) {
                // std::cout << vertices[vertexIdx][0] << " " << vertices[vertexIdx][1] << " " << vertices[vertexIdx][2]
                //           << std::endl;
                glVertex3dv(vertices[vertexIdx].data());
            }
        }
    }
    glEnd();
}
