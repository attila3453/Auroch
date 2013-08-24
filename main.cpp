#include <irrlicht/irrlicht.h>
#include <bullet/btBulletDynamicsCommon.h>
#include <vector>
#include <fstream>
#include <iostream>
#include <string.h>

using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;
using namespace std;

ifstream fi("settings.txt");

int W = 800, H = 600;
const float minwidth = .1, maxwidth = .9, minheight = .1, maxheight = .9;

ISceneManager* smgr;
IVideoDriver* driver;
IGUIEnvironment *guienv;
ICameraSceneNode* camera;

btDynamicsWorld *world;
btDispatcher *dispatcher;
btBroadphaseInterface *broadphase;
btConstraintSolver *solver;
btCollisionConfiguration *collisionconfig;

vector<btRigidBody*> bodies;
float rotspeed = .05, movspeed = .5;
float camradius = .5;
float zoomfactor = 2;
vector3df offset(0, 5, 5);

int cameraid = 0;
int distfromcamera = 25;

class Input : public IEventReceiver {
public:
    virtual bool OnEvent(const SEvent& event) {
        if (event.EventType == EET_KEY_INPUT_EVENT)
            KeyIsDown[event.KeyInput.Key] = event.KeyInput.PressedDown;

        if(event.EventType == EET_MOUSE_INPUT_EVENT) {
            mx = event.MouseInput.X;
            my = event.MouseInput.Y;

            leftmouse = event.MouseInput.isLeftPressed();
            mousewheel = event.MouseInput.Wheel;
        }

        return false;
    }

    virtual bool IsKeyDown(EKEY_CODE keyCode) const {
        return KeyIsDown[keyCode];
    }

    Input() {
        for (u32 i=0; i<KEY_KEY_CODES_COUNT; ++i)
            KeyIsDown[i] = false;
    }

    int mx, my;
    bool leftmouse;
    float mousewheel;

private:
    bool KeyIsDown[KEY_KEY_CODES_COUNT];
} receiver;

//The btBvhTriangleMeshShape is a static-triangle mesh shape,
//it can only be used for fixed/non-moving objects.
void CreateTriangleMesh(IMesh *mesh, btVector3 pos) {
    ISceneNode *Node = smgr->addOctreeSceneNode(mesh);
    Node->setMaterialFlag(EMF_LIGHTING, 0);

    IMeshBuffer *buffer;
    u16 *ids;
    int i, nrbuffers = mesh->getMeshBufferCount();

    btTriangleMesh *trimesh = new btTriangleMesh();

    for(i = 0; i < nrbuffers; i++) {
        buffer = mesh->getMeshBuffer(i);

        btVector3 v[3];
        ids = buffer->getIndices();

        S3DVertex *verts = (S3DVertex2TCoords *)buffer->getVertices();

        for(u32 i = 0; i < buffer->getIndexCount(); i += 3) {
            v[0] = btVector3(verts[ids[i]].Pos.X, verts[ids[i]].Pos.Y, verts[ids[i]].Pos.Z);
            v[1] = btVector3(verts[ids[i+1]].Pos.X, verts[ids[i+1]].Pos.Y, verts[ids[i+1]].Pos.Z);
            v[2] = btVector3(verts[ids[i+2]].Pos.X, verts[ids[i+2]].Pos.Y, verts[ids[i+2]].Pos.Z);
            trimesh->addTriangle(v[0], v[1], v[2]);
        }
    }

    delete ids;

    btTransform Transform;
    Transform.setIdentity();
    Transform.setOrigin(pos);

    btDefaultMotionState *motion = new btDefaultMotionState(Transform);
    btCollisionShape *trimeshshape = new btBvhTriangleMeshShape(trimesh, true);

    btVector3 LocalInertia;
    trimeshshape->calculateLocalInertia(0, LocalInertia);

    btRigidBody *body = new btRigidBody(0, motion, trimeshshape, LocalInertia);

    body->setUserPointer((void *)(Node));

    world->addCollisionObject(body);
    bodies.push_back(body);
}

void CreateBox(btVector3 pos, vector3df scale, btScalar mass, const char * texturepath) {
	ISceneNode *Node = smgr->addCubeSceneNode(1);
	Node->setScale(scale);
	Node->setMaterialFlag(EMF_LIGHTING, 0);

	if(texturepath)
        Node->setMaterialTexture(0, driver->getTexture(texturepath));

	btTransform Transform;
	Transform.setIdentity();
	Transform.setOrigin(pos);

	btDefaultMotionState *MotionState = new btDefaultMotionState(Transform);
	btVector3 HalfExtents(scale.X / 2., scale.Y / 2., scale.Z / 2.);
	btCollisionShape *Shape = new btBoxShape(HalfExtents);

	btVector3 LocalInertia;
	Shape->calculateLocalInertia(mass, LocalInertia);

	btRigidBody *RigidBody = new btRigidBody(mass, MotionState, Shape, LocalInertia);

	RigidBody->setUserPointer((void *)(Node));
	world->addRigidBody(RigidBody);
	bodies.push_back(RigidBody);
}

void CreateSphere(btVector3 pos, btScalar radius, btScalar mass, const char * texturepath) {
    ISceneNode *Node = smgr->addSphereSceneNode(radius);

    if(texturepath)
        Node->setMaterialTexture(0, driver->getTexture(texturepath));

    Node->setMaterialFlag(EMF_LIGHTING, 0);

    btTransform Transform;
    Transform.setIdentity();
    Transform.setOrigin(pos);

    btDefaultMotionState *MotionState = new btDefaultMotionState(Transform);
    btCollisionShape *Shape = new btSphereShape(radius);

    btVector3 LocalInertia;
    Shape->calculateLocalInertia(mass, LocalInertia);

    btRigidBody *RigidBody = new btRigidBody(mass, MotionState, Shape, LocalInertia);

    RigidBody->setUserPointer((void *)(Node));

    world->addRigidBody(RigidBody);
    bodies.push_back(RigidBody);
}

void UpdatePhysics(u32 TDeltaTime) {

    world->stepSimulation(TDeltaTime * 0.001f, 60);

    for(int i = 0; i < (signed)bodies.size(); i++) {
        ISceneNode *Node = (ISceneNode *)(bodies[i]->getUserPointer());

        btVector3 Point = bodies[i]->getCenterOfMassPosition();
        Node->setPosition(vector3df((f32)Point[0], (f32)Point[1], (f32)Point[2]));

        vector3df euler;
        btQuaternion bquat = bodies[i]->getOrientation();
        quaternion iquat(bquat.getX(), bquat.getY(), bquat.getZ(), bquat.getW());
        iquat.toEuler(euler);
        euler *= RADTODEG;
        Node->setRotation(euler);

    }
}

void updatecamera() {
        btVector3 ppos = bodies[cameraid]->getCenterOfMassPosition();
        vector3df pos = vector3df(ppos.getX(), ppos.getY(), ppos.getZ());
        offset = vector3df(0, distfromcamera, distfromcamera) * zoomfactor;
        vector3df rot = camera->getRotation();

        if(receiver.mx < W * minwidth) rot.Y -= rotspeed;
        if(receiver.mx > W * maxwidth) rot.Y += rotspeed;
        if(receiver.my < H * minheight) rot.X += rotspeed;
        if(receiver.my > H * maxheight) rot.X -= rotspeed;

        camera->setRotation(rot);

        matrix4 yawMat;
        yawMat.setRotationRadians(rot);
        yawMat.transformVect(offset);

        camera->setPosition(pos + offset);
        camera->setTarget(pos);
        offset.normalize();
        vector3df movedir = vector3df(-offset.X, 0, -offset.Z);
        vector3df strafevector(-movedir.Z, 0, movedir.X);
        vector3df speed(0, 0, 0);

        if(receiver.IsKeyDown(KEY_KEY_W)) speed += movedir;
        if(receiver.IsKeyDown(KEY_KEY_S)) speed -= movedir;
        if(receiver.IsKeyDown(KEY_KEY_A)) speed += strafevector;
        if(receiver.IsKeyDown(KEY_KEY_D)) speed -= strafevector;

        btVector3 vel0 = bodies[cameraid]->getLinearVelocity();
        bodies[cameraid]->setLinearVelocity(vel0 + btVector3(speed.X, speed.Y, speed.Z));
}

int main(int argc, char** argv) {
    int selectrenderer = 1;

    std::string line;
    while(getline(fi, line)) {
        cout<<line<<'\n';
        if(line.substr(0, 14) == "selectrenderer")
            sscanf(line.c_str(), "selectrenderer %d", &selectrenderer);
    }

    E_DRIVER_TYPE renderer = EDT_OPENGL;

    if(selectrenderer) {
        int nrend;
        printf("Select renderer (default OpenGL):\n1:OpenGL\n2:Direct3D\n3:Burning's software renderer\n");
        scanf("%d", &nrend);
        if(nrend == 1) renderer = EDT_OPENGL;
        if(nrend == 2) renderer = EDT_DIRECT3D9;
        if(nrend == 3) renderer = EDT_BURNINGSVIDEO;
    }

    dimension2d<unsigned int> windowsize = dimension2d<unsigned int>(W, H);

    IrrlichtDevice *device = createDevice(renderer, windowsize, 32, false, false, true, &receiver);

    guienv = device->getGUIEnvironment();
    IGUIFont *font = guienv->getFont("data/fontconsolas.xml");

    device->setWindowCaption(L"Auroch");

    driver = device->getVideoDriver();
    smgr = device->getSceneManager();

    //smgr->addLightSceneNode(0, vector3df(0,50,0), SColorf(0.3f,0.3f,0.3f), 1.0f, 1 );

    camera = smgr->addCameraSceneNode();

    device->setResizable(true);

    collisionconfig = new btDefaultCollisionConfiguration();
    dispatcher = new btCollisionDispatcher(collisionconfig);
    broadphase = new btDbvtBroadphase();
    solver = new btSequentialImpulseConstraintSolver();

    world = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionconfig);
    world->setGravity(btVector3(0, -25, 0));

    cameraid = 0;
    CreateBox(btVector3(0, 50, 0), vector3df(5, 15, 5), 5, "data/cow.jpg");

    CreateTriangleMesh(smgr->getMesh("data/testmap2.obj"), btVector3(0, 0, 0));

    for(int i = 0;i < 50;i++){
        CreateBox(btVector3(0, 50, 0), vector3df(5, 15, 5), 5, "data/cow.jpg");
        CreateSphere(btVector3(20, 60, 30), 5, 5, "data/cow.jpg");
    }

    while(device->run()) {
        if(receiver.IsKeyDown(KEY_ESCAPE)) {
            break;
        }

        if(receiver.IsKeyDown(KEY_KEY_R)) {
            camera->setPosition(vector3df(0, 15, 0));
            camera->setRotation(vector3df(0, 0, 0));
        }

        if(receiver.IsKeyDown(KEY_SPACE)) {
            btVector3 speed1 = bodies[cameraid]->getLinearVelocity();
            speed1 += btVector3(0, 2, 0);
            bodies[cameraid]->setLinearVelocity(speed1);
        }

        if(receiver.IsKeyDown(KEY_PLUS))
            zoomfactor-=.2;

        if(receiver.IsKeyDown(KEY_MINUS))
            zoomfactor+=.2;

        if(receiver.IsKeyDown(KEY_DOWN) and cameraid > 0)
            cameraid--;

        if(receiver.IsKeyDown(KEY_UP) and cameraid < (signed)bodies.size() - 1)
            cameraid++;

        rect<s32> viewport = driver->getViewPort();
        W = viewport.getWidth();
        H = viewport.getHeight();

        bodies[cameraid]->activate();

        UpdatePhysics(24);

        updatecamera();

        driver->beginScene(true, true, SColor(0,200,500,700));

        smgr->drawAll();
        stringw text = stringw("Bodies:") + stringw(bodies.size()).c_str() + "\n"+
                       stringw("Camera id:") + stringw(cameraid) + "\n";
        font->draw(text, rect<int>(0, 0, 50, 50), SColor(255, 0, 0, 255));

        guienv->drawAll();

        driver->endScene();
    }

    for(unsigned int i = 0; i < bodies.size(); i++) {
        ISceneNode *Node = (ISceneNode *)(bodies[i]->getUserPointer());
        Node->remove();
        world->removeCollisionObject(bodies[i]);
        delete bodies[i]->getCollisionShape();
        delete bodies[i]->getMotionState();
        delete bodies[i];
    }

    delete collisionconfig;
    delete dispatcher;
    delete broadphase;
    delete solver;
    delete world;

    return 0;
}

