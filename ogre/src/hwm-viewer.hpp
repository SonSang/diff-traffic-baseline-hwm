#ifndef _HWM_VIEWER_HPP_
#define _HWM_VIEWER_HPP_

#include <Ogre.h>
#include <OgreStringConverter.h>
#include <OgreException.h>

#define OIS_DYNAMIC_LIB
#include <OIS/OIS.h>

#include "Caelum.h"
#include "network.hpp"

struct anim_car;

struct car_model
{
    Ogre::MeshPtr bodymesh_;
    Ogre::MeshPtr wheelmesh_;
};

class hwm_frame_listener;

struct hwm_viewer
{
    hwm_viewer(network *net);

    void go();

    ~hwm_viewer();

    void create_root();

    void define_resources();

    void setup_render_system();

    void start_caelum();

    void create_render_window();

    void initialize_resource_groups();

    void initialize_network();

    void setup_scene();

    void create_car(const car_model &carmodel, const std::string &name, anim_car &ac);

    void create_lane_mesh(const lane &la, const std::string &name, float bb[6]);

    // void setup_CEGUI();

    void create_frame_listener();

    void start_render_loop();

    Ogre::Root *root_;
    OIS::Keyboard *keyboard_;
    OIS::InputManager *input_manager_;
    // CEGUI::OgreCEGUIRenderer *renderer_;
    // CEGUI::System *system_;
    hwm_frame_listener *move_listener_;

    Ogre::Camera *camera_;
    Ogre::SceneManager *scene_manager_;

    network *net_;

    std::vector<anim_car> lane_cars_;

    Caelum::CaelumSystem *caelum_system_;
};

#endif
