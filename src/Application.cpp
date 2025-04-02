#include "Application.hpp"

#define ARRAY_SIZE_Y 5
#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Z 5

Application::Application() {
    // Initialize Raylib
    const int screenWidth = 1366;
    const int screenHeight = 768;
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(screenWidth, screenHeight, "Bullet Physics with Raylib");

    // Set up Camera
    m_camera.position = (Vector3){ 10.0f, 10.0f, 15.0f };
    m_camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };
    m_camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    m_camera.fovy = 45.0f;
    m_camera.projection = CAMERA_PERSPECTIVE;

    // load textures
    m_texture_orange = LoadTexture("resources/textures/kenney_prototype-textures/PNG/Orange/texture_10.png");
    m_texture_dark   = LoadTexture("resources/textures/kenney_prototype-textures/PNG/Dark/texture_08.png");
    Texture2D texture_purple = LoadTexture("resources/textures/kenney_prototype-textures/PNG/Purple/texture_10.png");
    SetTextureWrap(m_texture_dark, TEXTURE_WRAP_REPEAT);
    
    // Set up Bullet Physics world
    createDynamicsWorld();

    // Create ground
    // btCollisionShape* groundShape = new btBoxShape(btVector3(m_groundSize.x / 2, m_groundSize.y / 2, m_groundSize.z / 2));
    // btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(m_groundPos.x, m_groundPos.y, m_groundPos.z)));
    
    // btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0, groundMotionState, groundShape, btVector3(0, 0, 0));
    // m_ground = new btRigidBody(groundRigidBodyCI);
    // m_dynamicsWorld->addRigidBody(m_ground);

    m_groundModel = LoadModelFromMesh(GenMeshCube(m_groundSize.x, m_groundSize.y, m_groundSize.z));
    m_groundModel.materials[0].maps[MATERIAL_MAP_ALBEDO].texture = m_texture_dark;

    btBoxShape* groundShape = createBoxShape(btVector3(m_groundSize.x / 2, m_groundSize.y / 2, m_groundSize.z / 2));
    m_collisionShapes.push_back(groundShape);

    btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, 0, 0));

    {
		btScalar mass(0.);
		createRigidBody(mass, groundTransform, groundShape, btVector4(0, 0, 1, 1));
	}

    {
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance

		btBoxShape* colShape = createBoxShape(btVector3(0.5, 0.5, 0.5));

		//btCollisionShape* colShape = new btSphereShape(btScalar(1.));
		m_collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		// btVector3 localInertia(0, 0, 0);
		// if (isDynamic)
		// 	colShape->calculateLocalInertia(mass, localInertia);

		for (int k = 0; k < ARRAY_SIZE_Y; k++)
		{
			for (int i = 0; i < ARRAY_SIZE_X; i++)
			{
				for (int j = 0; j < ARRAY_SIZE_Z; j++)
				{
					startTransform.setOrigin(btVector3(
						btScalar(1 * i),
						btScalar(10 + 1 * k),
						btScalar(1 * j)));

                    createRigidBody(mass, startTransform, colShape, btVector4(1, 0, 0, 1));
				}
			}
		}
	}


    // Load Cube Model
    // m_cubeModel = LoadModel("resources/models/cube.obj");
    m_cubeModel = LoadModelFromMesh(GenMeshCube(1.0f, 1.0f, 1.0f));
    m_cubeModel.materials[0].maps[MATERIAL_MAP_ALBEDO].texture = m_texture_orange;
}

Application::~Application() {
    // Cleanup Bullet Physics
    for (btRigidBody* body : m_rigidBodies) {
        m_dynamicsWorld->removeRigidBody(body);
        delete body->getMotionState();
        delete body->getCollisionShape();
        delete body;
    }
    
    delete m_dynamicsWorld;
    delete m_solver;
    delete m_broadphase;
    delete m_dispatcher;
    delete m_collisionConfiguration;

    // Close Raylib
    CloseWindow();
}

void Application::createDynamicsWorld()
{
    m_collisionConfiguration = new btDefaultCollisionConfiguration();
    m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
    m_broadphase = new btDbvtBroadphase();
    m_solver = new btSequentialImpulseConstraintSolver();
    m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
    m_dynamicsWorld->setGravity(btVector3(0, -9.81f, 0));
}

btBoxShape* Application::createBoxShape(const btVector3& halfExtents)
{
    btBoxShape* box = new btBoxShape(halfExtents);
    return box;
}

btRigidBody *Application::createRigidBody(float mass, const btTransform &startTransform, btCollisionShape *shape, const btVector4 &color = btVector4(1, 0, 0, 1))
{
    btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

    //rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool isDynamic = (mass != 0.f);

    btVector3 localInertia(0, 0, 0);
    if (isDynamic)
        shape->calculateLocalInertia(mass, localInertia);

    //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
    btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, shape, localInertia);
    btRigidBody* body = new btRigidBody(cInfo);

    body->setUserIndex(-1);
    m_dynamicsWorld->addRigidBody(body);
    return body;
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

    m_dynamicsWorld->addRigidBody(cubeRigidBody);
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
    DrawModelWires(m_cubeModel, {0, 0, 0}, 1.0f, RED);
}

void Application::run() {
    SetTargetFPS(60);
    DisableCursor();


    while (!WindowShouldClose()) {
        // Update
        UpdateCamera(&m_camera, CAMERA_FREE);

        if (IsKeyDown(KEY_LEFT_CONTROL)) EnableCursor();
        if (IsKeyReleased(KEY_LEFT_CONTROL)) DisableCursor();

        m_dynamicsWorld->stepSimulation(GetFrameTime(), 10);


        // Render
        BeginDrawing();
        ClearBackground(RAYWHITE);
        BeginMode3D(m_camera);
        DrawGrid(100, 1.0f);

        DrawModel(m_groundModel, {0, 0, 0}, 1.0f, WHITE);

        int numCollisionObjects = m_dynamicsWorld->getNumCollisionObjects();
        for (int i = 0; i < numCollisionObjects; i++) {
            btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
            btRigidBody* body = btRigidBody::upcast(obj);
            if (body && body->getMotionState()) {
                drawCube(body);
            }
        }

        EndMode3D();
        DrawText("Bullet Physics with Raylib", 10, 10, 20, BLACK);
        DrawFPS(10, 40);
        DrawText(TextFormat("Cubes: %d", 0), 10, 70, 20, BLACK);
        EndDrawing();
    } 

}


