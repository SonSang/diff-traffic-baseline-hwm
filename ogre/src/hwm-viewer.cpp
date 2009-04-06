#include <Ogre.h>
#include <OIS/OIS.h>
#include <boost/foreach.hpp>

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
        setup_input_system();
        //setup_CEGUI();
        create_frame_listener();
        start_render_loop();
    }

    ~hwm_viewer()
    {
        input_manager_->destroyInputObject(keyboard_);
        OIS::InputManager::destroyInputSystem(input_manager_);
        // delete renderer_;
        // delete system_;

        delete listener_;
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
        SceneManager *mgr = root_->createSceneManager(ST_GENERIC, "Default SceneManager");
        Camera       *cam = mgr->createCamera("Camera");
        Viewport     *vp  = root_->getAutoCreatedWindow()->addViewport(cam);
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
        listener_ = new exit_listener(keyboard_);
        root_->addFrameListener(listener_);
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
    exit_listener *listener_;
};

int main(int argc, char **argv)
{
    hwm_viewer hv;
    hv.go();

    return 0;
}
