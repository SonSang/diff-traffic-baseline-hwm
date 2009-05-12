#ifndef _HWM_VIEWER_HPP_
#define _HWM_VIEWER_HPP_

#include <Ogre.h>
#include <OgreStringConverter.h>
#include <OgreException.h>

#define OIS_DYNAMIC_LIB
#include <OIS/OIS.h>

#include "Caelum.h"
#include "network.hpp"

struct hwm_viewer;

struct anim_car
{
    anim_car () : root_(0), cm_(0)
    {
        memset(wheels_, 0, sizeof(Ogre::SceneNode*)*4);
    }

    Ogre::SceneNode *root_;
    Ogre::SceneNode *wheels_[4];
    car_model       *cm_;
};

struct car_time_sample
{
    void from_pair(const car_time_sample &cts0,
                   const car_time_sample &cts1,
                   float                  f0,
                   float                  f1);

    void apply_to_car(anim_car &ac) const;

    int   id;
    float pos[3];
    float theta;
    float velocity;
    int   motion_state;
};

struct car_time_series
{
    typedef std::vector<car_time_sample> sample_vector;
    typedef std::pair<float,sample_vector > entry;

    int read(FILE *fp);

    void update_cars(float t, hwm_viewer *hv) const;

    std::vector<entry> samples;
};

struct ogre_car_model
{
    anim_car create_car(Ogre::SceneManager *sm, Ogre::SceneNode *base, const std::string &name) const;

    Ogre::MeshPtr bodymesh;
    Ogre::MeshPtr wheelmesh;

    std::vector<Ogre::MaterialPtr> materials;

    car_model *cm;
};

struct ogre_car_db : public car_db
{
    void load_meshes();

    std::map<std::string, ogre_car_model> odb;
};

class hwm_frame_listener;

struct hwm_viewer
{
    hwm_viewer(network *net, const char *anim_file, const char *carpath);

    void go();

    ~hwm_viewer();

    void create_root();

    void define_resources();

    void setup_render_system();

    void start_caelum();

    void create_render_window();

    void initialize_resource_groups();

    void initialize_network();

    void load_terrain(Ogre::SceneNode *sn);

    void setup_scene();

    anim_car& access_sim_car(int id);

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
    std::vector<anim_car> sim_cars_;
    Ogre::SceneNode *vehicle_node_;
    car_time_series cts_;

    Caelum::CaelumSystem *caelum_system_;

    ogre_car_db *o_cars_;
};

#endif
