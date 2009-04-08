#include <Ogre.h>
#include <OIS/OIS.h>
#include <boost/foreach.hpp>
#include "ExampleFrameListener.h"

using namespace Ogre;

struct exit_listener : public FrameListener
{
    exit_listener(OIS::Keyboard *keyboard)
        : keyboard_(keyboard)
    {}

    bool frameStarted(const FrameEvent &evt)
    {
        keyboard_->capture();
        return !keyboard_->isKeyDown(OIS::KC_ESCAPE);
    }

    OIS::Keyboard *keyboard_;
};

struct hwm_viewer
{
    void go()
    {
        create_root();
        define_resources();
        setup_render_system();
        create_render_window();
        initialize_resource_groups();
        setup_scene();
        //        setup_input_system();
        //setup_CEGUI();
        create_frame_listener();
        start_render_loop();
    }

    ~hwm_viewer()
    {
        // input_manager_->destroyInputObject(keyboard_);
        // OIS::InputManager::destroyInputSystem(input_manager_);
        // delete renderer_;
        // delete system_;

        delete move_listener_;
        //    delete e_listener_;
        delete root_;
    }

    void create_root()
    {
        root_ = new Root();
    }

    void define_resources()
    {
        String sec_name, type_name, arch_name;
        ConfigFile cf;
        cf.load("resources.cfg");

        ConfigFile::SectionIterator seci = cf.getSectionIterator();
        while(seci.hasMoreElements())
        {
            sec_name = seci.peekNextKey();
            ConfigFile::SettingsMultiMap *settings = seci.getNext();
            ConfigFile::SettingsMultiMap::iterator i;
            for(i = settings->begin(); i != settings->end(); ++i)
            {
                type_name = i->first;
                arch_name = i->second;
                ResourceGroupManager::getSingleton().addResourceLocation(arch_name, type_name, sec_name);
            }
        }
    }

    void setup_render_system()
    {
        if(!root_->restoreConfig() && !root_->showConfigDialog())
            throw Exception(52, "User cancelled the config dialog!",
                            "hwm_viewer::setup_render_system()");

    }

    void create_render_window()
    {
        root_->initialise(true, "hwm viewer");
    }

    void initialize_resource_groups()
    {
        TextureManager::getSingleton().setDefaultNumMipmaps(5);
        ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
    }

    void setup_scene()
    {
        terrain_scene_manager_ = root_->createSceneManager("TerrainSceneManager", "GroundSceneManager");
        camera_ = terrain_scene_manager_->createCamera("Camera");

        camera_->setPosition(707, 2500, 528);
        camera_->lookAt(0,0,0);
        //        camera_->setOrientation(Quaternion(-0.3486, 0.0122, 0.9365, 0.0329));
        camera_->setNearClipDistance(1);
        camera_->setFarClipDistance(1000);

        Viewport *vp  = root_->getAutoCreatedWindow()->addViewport(camera_);

        terrain_scene_manager_->setAmbientLight(ColourValue(0.5, 0.5, 0.5));

        Light *l = terrain_scene_manager_->createLight("MainLight");
        l->setPosition(20, 80, 50);

        ColourValue bgcolor(0.93, 0.86, 0.76);
        terrain_scene_manager_->setFog(FOG_LINEAR, bgcolor, 0.001, 500, 1000);
        vp->setBackgroundColour(bgcolor);

        std::string terrain_cfg("my_terrain.cfg");
        terrain_scene_manager_->setWorldGeometry(terrain_cfg);

        if(root_->getRenderSystem()->getCapabilities()->hasCapability(RSC_INFINITE_FAR_PLANE))
            camera_->setFarClipDistance(0);

        Plane plane;
        plane.d = 5000;
        plane.normal = -Vector3::UNIT_Y;

        //root_->showDebugOverlay(true);
    }

    void setup_input_system()
    {
        size_t windowHnd = 0;
        std::ostringstream windowHndStr;
        OIS::ParamList pl;
        RenderWindow *win = root_->getAutoCreatedWindow();

        win->getCustomAttribute("WINDOW", &windowHnd);
        windowHndStr << windowHnd;
        pl.insert(std::make_pair(std::string("WINDOW"), windowHndStr.str()));
        input_manager_ = OIS::InputManager::createInputSystem(pl);

        try
        {
            keyboard_ = static_cast<OIS::Keyboard*>(input_manager_->createInputObject(OIS::OISKeyboard, false));
            //            mouse_ = static_cast<OIS::Mouse*>(input_manager_->createInputObject(OIS::OISMouse, false));
        }
        catch (const OIS::Exception &e)
        {
            throw Exception(42, e.eText, "hwm_viewer::setupInputSystem");
        }
    }
    // void setup_CEGUI()
    // {}

    void create_frame_listener()
    {
        //   e_listener_ = new exit_listener(keyboard_);
        //   root_->addFrameListener(e_listener_);
        move_listener_ = new ExampleFrameListener(root_->getAutoCreatedWindow(), camera_);
        root_->addFrameListener(move_listener_);
    }

    void start_render_loop()
    {
        root_->startRendering();
    }

    Root *root_;
    OIS::Keyboard *keyboard_;
    OIS::InputManager *input_manager_;
    // CEGUI::OgreCEGUIRenderer *renderer_;
    // CEGUI::System *system_;
    exit_listener *e_listener_;
    ExampleFrameListener *move_listener_;

    Camera *camera_;
    SceneManager *terrain_scene_manager_;
};

int main(int argc, char **argv)
{
    hwm_viewer hv;
    hv.go();

    return 0;
}
