#include "Vehicle.hpp"


Vehicle::Vehicle(int id, Vector3 position, btDynamicsWorld* dynamicsWorld)
    : m_dynamicsWorld(dynamicsWorld)
{
    m_size = btVector3(3, 4, 2);

    btVector3 halfExtents(m_size.x() / 2, m_size.y() / 2, m_size.z() / 2);
    btBoxShape* bodyShape = new btBoxShape(halfExtents);
    // m_collisionShapes.push_back(groundShape);
    btTransform vehicleTransform;
	vehicleTransform.setIdentity();
	vehicleTransform.setOrigin(btVector3(5, 5, 3));

    btScalar mass(1.f);
    btVector3 localInertia(0, 0, 0);
    bodyShape->calculateLocalInertia(mass, localInertia);

    btDefaultMotionState* vehicleMotionState = new btDefaultMotionState(vehicleTransform);
    btRigidBody::btRigidBodyConstructionInfo vehicleRigidBodyCI(mass, vehicleMotionState, bodyShape, localInertia);
    m_rigidBody = new btRigidBody(vehicleRigidBodyCI);
    m_rigidBody->setUserIndex(id);
    m_rigidBody->setUserPointer(this);

    m_dynamicsWorld->addRigidBody(m_rigidBody);
}

Vehicle::~Vehicle()
{
}

void Vehicle::Render()
{
}
