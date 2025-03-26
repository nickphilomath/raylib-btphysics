#pragma once

#include <iostream>
#include <string>
#include <vector>

#include <raylib.h>
#include <raymath.h>
#include <btBulletDynamicsCommon.h>


class Application
{
public:
    Application();
    ~Application();

    void run();

private:
    btRigidBody*                createCube(float x, float y, float z, float size, float mass);
    void                        drawCube(btRigidBody* body);

    btDiscreteDynamicsWorld*    m_world = nullptr;
    std::vector<btRigidBody*>   m_rigidBodies;
    Camera3D                    m_camera = { 0 };

    btRigidBody*                m_ground = nullptr;
    Model                       m_cubeModel;
};