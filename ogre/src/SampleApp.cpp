#include "ExampleApplication.h"

class TutorialApplication : public ExampleApplication
{
protected:
public:
    TutorialApplication()
    {
    }

    ~TutorialApplication()
    {
    }
protected:
    virtual void createCamera(void)
    {
        mCamera = mSceneMgr->createCamera("PlayerCam");
        mCamera->setPosition(Vector3(0, 10, 500));
        mCamera->lookAt(Vector3(0,0,0));
        mCamera->setNearClipDistance(5);
    }

    virtual void createViewports(void)
    {
        Viewport* vp = mWindow->addViewport(mCamera);
        vp->setBackgroundColour(ColourValue(0,0,0));
        // Alter the camera aspect ratio to match the viewport
        mCamera->setAspectRatio(Real(vp->getActualWidth()) / Real(vp->getActualHeight()));
    }

    void createScene(void)
    {
        mSceneMgr->setAmbientLight(ColourValue(0, 0, 0));
        //        mSceneMgr->setShadowTechnique(SHADOWTYPE_STENCIL_ADDITIVE);

        // MeshPtr bodymesh = MeshManager::getSingleton().load("tbird-body-mesh.mesh",
        //                                                     ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        // bodymesh->createManualLodLevel(2500, "tbird-body-mesh-lod2.mesh");
        // bodymesh->createManualLodLevel(5000, "tbird-body-mesh-lod3.mesh");

        // MeshPtr wheelmesh = MeshManager::getSingleton().load("tbird-wheel-mesh.mesh",
        //                                                      ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        // wheelmesh->createManualLodLevel(2500, "tbird-wheel-mesh-lod2.mesh");
        // wheelmesh->createManualLodLevel(5000, "tbird-wheel-mesh-lod3.mesh");

        // Entity* ent2 = mSceneMgr->createEntity("Car", bodymesh->getName() );
        // //        ent2->setCastShadows(true);

        // SceneNode *carnode = mSceneMgr->getRootSceneNode()->createChildSceneNode("Car", Vector3(0,  100+0.507+0.751*0.5, 0));
        // carnode->scale(100, 100, 100);
        // carnode->attachObject(ent2);

        // {
        //     Entity *wheel = mSceneMgr->createEntity("Wheel1", wheelmesh->getName());
        //     //            wheel->setCastShadows(true);

        //     SceneNode *wheelnode = carnode->createChildSceneNode("Wheel1", Vector3(0,  -.507, 0.808));
        //     wheelnode->yaw(Degree(180));
        //     wheelnode->attachObject(wheel);
        // }
        // {
        //     Entity *wheel = mSceneMgr->createEntity("Wheel2", wheelmesh->getName());
        //     //            wheel->setCastShadows(true);

        //     SceneNode *wheelnode = carnode->createChildSceneNode("Wheel2", Vector3(0,  -.507, -0.808));
        //     wheelnode->attachObject(wheel);
        // }
        // {
        //     Entity *wheel = mSceneMgr->createEntity("Wheel3", wheelmesh->getName());
        //     //            wheel->setCastShadows(true);

        //     SceneNode *wheelnode = carnode->createChildSceneNode("Wheel3", Vector3(2.916,  -.507, 0.808));
        //     wheelnode->yaw(Degree(180));
        //     wheelnode->attachObject(wheel);
        // }
        // {
        //     Entity *wheel = mSceneMgr->createEntity("Wheel4", wheelmesh->getName());
        //     //            wheel->setCastShadows(true);

        //     SceneNode *wheelnode = carnode->createChildSceneNode("Wheel4", Vector3(2.916,  -.507, -0.808));
        //     wheelnode->attachObject(wheel);
        // }

        mSceneMgr->setSkyBox(true, "Examples/SpaceSkyBox", 5000, false);

        Plane plane(Vector3::UNIT_Y, 0);
        MeshManager::getSingleton().createPlane("ground",
                                                ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, plane,
                                                5000,5000,20,20,true,1,5,5,Vector3::UNIT_Z);
        Entity *ent = mSceneMgr->createEntity("GroundEntity", "ground");
        mSceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(ent);
        ent->setMaterialName("lighting");
        ent->setCastShadows(false);

        Light *light = mSceneMgr->createLight("Light1");
        light->setType(Light::LT_POINT);
        light->setPosition(Vector3(0, 150, 250));

        light->setDiffuseColour(1.0, 1.0, 1.0);
        light->setSpecularColour(1.0, 1.0, 1.0);

        light = mSceneMgr->createLight("Light3");
        light->setType(Light::LT_DIRECTIONAL);
        light->setDiffuseColour(ColourValue(.25, .25, 0));
        light->setSpecularColour(ColourValue(.25, .25, 0));
        light->setDirection(Vector3( 0, -1, 1 ));

        light = mSceneMgr->createLight("Light2");
        light->setType(Light::LT_SPOTLIGHT);
        light->setDiffuseColour(0, 0, 1.0);
        light->setSpecularColour(0, 0, 1.0);

        light->setDirection(-1, -1, 0);
        light->setPosition(Vector3(300, 300, 0));

        light->setSpotlightRange(Degree(35), Degree(50));
    }
};

#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
#define WIN32_LEAN_AND_MEAN
#include "windows.h"

INT WINAPI WinMain( HINSTANCE hInst, HINSTANCE, LPSTR strCmdLine, INT )
#else
int main(int argc, char **argv)
#endif
{
    printf("OgreCompVer: %d\n", OGRE_COMP_VER);

    // Create application object
    TutorialApplication app;

    try {
        app.go();
    } catch( Exception& e ) {
#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
        MessageBox( NULL, e.what(), "An exception has occurred!", MB_OK | MB_ICONERROR | MB_TASKMODAL);
#else
        fprintf(stderr, "An exception has occurred: %s\n",
                e.what());
#endif
    }

    return 0;
}
