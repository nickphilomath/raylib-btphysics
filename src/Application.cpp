#include "Application.hpp"

Application::Application() {
    // Initialize Raylib
    const int screenWidth = 1366;
    const int screenHeight = 768;
    InitWindow(screenWidth, screenHeight, "Bullet Physics with Raylib");
    
    // Set up Bullet Physics world
    btDefaultCollisionConfiguration* collisionConfig = new btDefaultCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfig);
    btBroadphaseInterface* broadphase = new btDbvtBroadphase();
    btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();
    m_world = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfig);
    m_world->setGravity(btVector3(0, -9.81f, 0));

    // Create ground
    const Vector3 groundSize = { 20, 1, 20 };
    const Vector3 groundPos = { 0, 0, 0 };

    btCollisionShape* groundShape = new btBoxShape(btVector3(groundSize.x / 2, groundSize.y / 2, groundSize.z / 2));
    btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(groundPos.x, groundPos.y, groundPos.z)));
    
    btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0, groundMotionState, groundShape, btVector3(0, 0, 0));
    m_ground = new btRigidBody(groundRigidBodyCI);
    m_world->addRigidBody(m_ground);

    // Set up Camera
    m_camera.position = (Vector3){ 10.0f, 10.0f, 15.0f };
    m_camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };
    m_camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    m_camera.fovy = 45.0f;
    m_camera.projection = CAMERA_PERSPECTIVE;

    // Load Cube Model
    m_cubeModel = LoadModel("resources/models/cube.obj");
    // load textures
    m_texture_dark = LoadTexture("resources/textures/kenney_prototype-textures/PNG/Orange/texture_08.png");
    m_cubeModel.materials[0].maps[MATERIAL_MAP_ALBEDO].texture = m_texture_dark;
}

Application::~Application() {
    // Cleanup Bullet Physics
    for (btRigidBody* body : m_rigidBodies) {
        m_world->removeRigidBody(body);
        delete body->getMotionState();
        delete body->getCollisionShape();
        delete body;
    }
    m_world->removeRigidBody(m_ground);
    delete m_ground->getMotionState();
    delete m_ground->getCollisionShape();
    delete m_ground;
    
    delete m_world;
    // delete solver;
    // delete broadphase;
    // delete dispatcher;
    // delete collisionConfig;

    // Close Raylib
    CloseWindow();
}

btRigidBody *Application::createCube(float x, float y, float z, float size, float mass)
{
    btCollisionShape* cubeShape = new btBoxShape(btVector3(size / 2, size / 2, size / 2));
    btDefaultMotionState* cubeMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(x, y, z)));

    btVector3 cubeInertia(0, 0, 0);
    if (mass > 0) {
        cubeShape->calculateLocalInertia(mass, cubeInertia);
    }

    btRigidBody::btRigidBodyConstructionInfo cubeRigidBodyCI(mass, cubeMotionState, cubeShape, cubeInertia);
    btRigidBody* cubeRigidBody = new btRigidBody(cubeRigidBodyCI);

    m_world->addRigidBody(cubeRigidBody);
    m_rigidBodies.push_back(cubeRigidBody);
    return cubeRigidBody;
}

void Application::drawCube(btRigidBody *cube)
{
    btTransform transform;
    cube->getMotionState()->getWorldTransform(transform);
    btVector3 cubePos = transform.getOrigin();

    // Extract rotation as Euler angles
    btScalar roll, pitch, yaw;
    transform.getBasis().getEulerZYX(yaw, pitch, roll);
    Vector3 rotationEuler = { RAD2DEG * roll, RAD2DEG * pitch, RAD2DEG * yaw };

    // Draw Falling Cube using transform info
    // Apply transformations directly to the model's transform matrix
    Matrix translation = MatrixTranslate(cubePos.x(), cubePos.y(), cubePos.z());
    Matrix rotation = MatrixRotateXYZ((Vector3){ DEG2RAD * rotationEuler.x, DEG2RAD * rotationEuler.y, DEG2RAD * rotationEuler.z });
    m_cubeModel.transform = MatrixMultiply(rotation, translation);

    // std::cout << "cube position: " << cubePos.x() << " " << cubePos.y() << " " << cubePos.z() << std::endl;

    // Draw the transformed model
    DrawModel(m_cubeModel, {0, 0, 0}, 1.0f, WHITE);
    // DrawModelWires(m_cubeModel, {0, 0, 0}, 1.0f, RED);
}

void Application::run() {
    // SetTargetFPS(60);
    DisableCursor();

    int count = 0;

    // create cubes
    createCube(0, 10, 0, 1, 1);
    count++;
    bool spawnStopped = false;

    btCollisionShape* ballShape = new btSphereShape(3);
    btDefaultMotionState* ballMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 5, 0)));
    btRigidBody::btRigidBodyConstructionInfo ballRigidBodyCI(1, ballMotionState, ballShape, btVector3(0, 0, 0));
    btRigidBody* ballRigidBody = new btRigidBody(ballRigidBodyCI);
    m_world->addRigidBody(ballRigidBody);


    while (!WindowShouldClose()) {
        // Update
        UpdateCamera(&m_camera, CAMERA_FREE);

        Vector3 cameraPos = { m_camera.position.x, m_camera.position.y, m_camera.position.z };
        btTransform ballTransform;
        ballTransform.setIdentity();
        ballTransform.setOrigin(btVector3(cameraPos.x, cameraPos.y, cameraPos.z));
        ballRigidBody->setWorldTransform(ballTransform);
        ballRigidBody->getMotionState()->setWorldTransform(ballTransform);

        if (IsKeyPressed('R')) {spawnStopped = false; count = 0;}
        if (count < 300 && !spawnStopped) {
            createCube(0, 20, 0, 1, 1);
            count++;
        } else spawnStopped = true;

        m_world->stepSimulation(GetFrameTime(), 10);

        // Remove cubes that fall below -100
        for (auto it = m_rigidBodies.begin(); it != m_rigidBodies.end(); ) {
            btTransform transform;
            (*it)->getMotionState()->getWorldTransform(transform);
            btVector3 pos = transform.getOrigin();

            if (pos.y() < -100) {
                m_world->removeRigidBody(*it);
                delete (*it)->getMotionState();
                delete (*it)->getCollisionShape();
                delete *it;
                it = m_rigidBodies.erase(it);
                count--;
            } else {
                ++it;
            }
        }

        // Render
        BeginDrawing();
        ClearBackground(RAYWHITE);
        BeginMode3D(m_camera);
        DrawGrid(100, 1.0f);

        // Draw Ground
        btTransform groundTransform;
        m_ground->getMotionState()->getWorldTransform(groundTransform);
        btVector3 groundPos = groundTransform.getOrigin();
        DrawCube((Vector3){ groundPos.x(), groundPos.y(), groundPos.z() }, 20, 1, 20, BLUE);
        DrawCubeWires((Vector3){ groundPos.x(), groundPos.y(), groundPos.z() }, 20, 1, 20, MAROON);

        for (btRigidBody* body : m_rigidBodies) {
            drawCube(body);
        }

        EndMode3D();
        DrawText("Bullet Physics with Raylib", 10, 10, 20, BLACK);
        DrawFPS(10, 40);
        DrawText(TextFormat("Cubes: %d", count), 10, 70, 20, BLACK);
        EndDrawing();
    } 

}


