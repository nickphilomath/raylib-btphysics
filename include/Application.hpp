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
    void                        createDynamicsWorld();
    btBoxShape*                 createBoxShape(const btVector3& halfExtents);
    btRigidBody*                createRigidBody(float mass, const btTransform& startTransform, btCollisionShape* shape, const btVector4& color);

    btRigidBody*                createCube(float x, float y, float z, float size, float mass);
    void                        drawCube(btRigidBody* body);

    std::vector<btRigidBody*>   m_rigidBodies;
    Camera3D                    m_camera = { 0 };

    //keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*> m_collisionShapes;
	btBroadphaseInterface* m_broadphase;
	btCollisionDispatcher* m_dispatcher;
	btConstraintSolver* m_solver;
	btDefaultCollisionConfiguration* m_collisionConfiguration;
	btDiscreteDynamicsWorld* m_dynamicsWorld;

    btRigidBody*                m_ground = nullptr;
    Model                       m_groundModel;
    Vector3                     m_groundSize = { 80, 1, 80 };
    Vector3                     m_groundPos = { 0, 0, 0 };
    Model                       m_cubeModel;

    Texture2D                   m_texture_dark;
    Texture2D                   m_texture_orange;
};