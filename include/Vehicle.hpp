#pragma once


#include <raylib.h>
#include <btBulletDynamicsCommon.h>



class Vehicle
{
public:
    Vehicle(int id, Vector3 position, btDynamicsWorld* dynamicsWorld);
    ~Vehicle();

    int getId() const;
    int getX() const;
    int getY() const;
    int getSpeed() const;

    void Render();

private:
    btDynamicsWorld* m_dynamicsWorld;
    btVector3 m_size;
    btRigidBody* m_rigidBody;
};