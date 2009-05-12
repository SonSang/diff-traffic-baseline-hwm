#include "hwm-viewer.hpp"

#include <cmath>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <OgreProgressiveMesh.h>

using namespace Ogre;

class hwm_frame_listener: public FrameListener, public WindowEventListener
{
protected:
	virtual void UpdateStats(void)
	{
		static String curr_fps = "Current FPS: ";
		static String avg_fps = "Average FPS: ";
		static String best_fps = "Best FPS: ";
		static String worst_fps = "Worst FPS: ";
		static String tris = "Triangle Count: ";
		static String batches = "Batch Count: ";

		// update stats when necessary
		try {
			OverlayElement *gui_avg   = OverlayManager::getSingleton().getOverlayElement("Core/AverageFps");
			OverlayElement *gui_curr  = OverlayManager::getSingleton().getOverlayElement("Core/CurrFps");
			OverlayElement *gui_best  = OverlayManager::getSingleton().getOverlayElement("Core/BestFps");
			OverlayElement *gui_worst = OverlayManager::getSingleton().getOverlayElement("Core/WorstFps");

			const RenderTarget::FrameStats &stats = window_->getStatistics();
			gui_avg ->setCaption(   avg_fps + StringConverter::toString(stats.avgFPS));
			gui_curr->setCaption(  curr_fps + StringConverter::toString(stats.lastFPS));
			gui_best->setCaption(  best_fps + StringConverter::toString(stats.bestFPS)
				+" "+StringConverter::toString(stats.bestFrameTime)+" ms");
			gui_worst->setCaption(worst_fps + StringConverter::toString(stats.worstFPS)
				+" "+StringConverter::toString(stats.worstFrameTime)+" ms");

			OverlayElement *gui_tris = OverlayManager::getSingleton().getOverlayElement("Core/NumTris");
			gui_tris->setCaption(tris + StringConverter::toString(stats.triangleCount));

			OverlayElement *gui_batches = OverlayManager::getSingleton().getOverlayElement("Core/NumBatches");
			gui_batches->setCaption(batches + StringConverter::toString(stats.batchCount));

			OverlayElement *gui_dbg = OverlayManager::getSingleton().getOverlayElement("Core/DebugText");
			gui_dbg->setCaption(debug_text_);
		}
		catch(...) { /* ignore */ }
	}

public:
	// Constructor takes a RenderWindow because it uses that to determine input context
	hwm_frame_listener(hwm_viewer *hwm_v, RenderWindow *win, Camera *cam, bool buffered_keys = false, bool buffered_mouse = false,
                       bool buffered_joy = false) :
		camera_(cam), translate_vector_(Vector3::ZERO), current_speed_(0), window_(win), stats_on_(true), num_screen_shots_(0),
		move_scale_(0.0f), rot_scale_(0.0f), time_until_next_toggle_(0), filtering_(TFO_BILINEAR),
		aniso_(1), scene_detail_index_(0), move_speed_(500), rotate_speed_(30), debug_overlay_(0),
		input_manager_(0), mouse_(0), keyboard_(0), joy_(0), t_(0), hwm_v_(hwm_v)
	{

		debug_overlay_ = OverlayManager::getSingleton().getByName("Core/DebugOverlay");

		LogManager::getSingletonPtr()->logMessage("*** Initializing OIS ***");
		OIS::ParamList pl;
		size_t window_hnd = 0;
		std::ostringstream window_hnd_str;

		win->getCustomAttribute("WINDOW", &window_hnd);
		window_hnd_str << window_hnd;
		pl.insert(std::make_pair(std::string("WINDOW"), window_hnd_str.str()));

		input_manager_ = OIS::InputManager::createInputSystem( pl );

		//Create all devices (We only catch joystick exceptions here, as, most people have Key/Mouse)
		keyboard_ = static_cast<OIS::Keyboard*>(input_manager_->createInputObject( OIS::OISKeyboard, buffered_keys ));
		mouse_ = static_cast<OIS::Mouse*>(input_manager_->createInputObject( OIS::OISMouse, buffered_mouse ));
		try {
			joy_ = static_cast<OIS::JoyStick*>(input_manager_->createInputObject( OIS::OISJoyStick, buffered_joy ));
		}
		catch(...) {
			joy_ = 0;
		}

		//Set initial mouse clipping size
		windowResized(window_);

		showDebugOverlay(true);

		//Register as a Window listener
		WindowEventUtilities::addWindowEventListener(window_, this);
	}

	//Adjust mouse clipping area
	virtual void windowResized(RenderWindow *rw)
	{
		unsigned int width, height, depth;
		int left, top;
		rw->getMetrics(width, height, depth, left, top);

		const OIS::MouseState &ms = mouse_->getMouseState();
		ms.width = width;
		ms.height = height;
	}

	//Unattach OIS before window shutdown (very important under Linux)
	virtual void windowClosed(RenderWindow *rw)
	{
		//Only close for window that created OIS (the main window in these demos)
		if( rw == window_ )
		{
			if( input_manager_ )
			{
				input_manager_->destroyInputObject( mouse_ );
				input_manager_->destroyInputObject( keyboard_ );
				input_manager_->destroyInputObject( joy_ );

				OIS::InputManager::destroyInputSystem(input_manager_);
				input_manager_ = 0;
			}
		}
	}

	virtual ~hwm_frame_listener()
	{
		//Remove ourself as a Window listener
		WindowEventUtilities::removeWindowEventListener(window_, this);
		windowClosed(window_);
	}

	virtual bool processUnbufferedKeyInput(const FrameEvent &evt)
	{

        if(keyboard_->isKeyDown(OIS::KC_C))
        {
            MaterialPtr material = MaterialManager::getSingleton().getByName("Test/ColourTest");
            material->getTechnique(0)->getPass(0)->setAmbient(drand48(), drand48(), drand48());
        }

		if(keyboard_->isKeyDown(OIS::KC_A))
			translate_vector_.x = -move_scale_;	// Move camera left

		if(keyboard_->isKeyDown(OIS::KC_D))
			translate_vector_.x = move_scale_;	// Move camera RIGHT

		if(keyboard_->isKeyDown(OIS::KC_UP) || keyboard_->isKeyDown(OIS::KC_W) )
			translate_vector_.z = -move_scale_;	// Move camera forward

		if(keyboard_->isKeyDown(OIS::KC_DOWN) || keyboard_->isKeyDown(OIS::KC_S) )
			translate_vector_.z = move_scale_;	// Move camera backward

		if(keyboard_->isKeyDown(OIS::KC_PGUP))
			translate_vector_.y = move_scale_;	// Move camera up

		if(keyboard_->isKeyDown(OIS::KC_PGDOWN))
			translate_vector_.y = -move_scale_;	// Move camera down

		if(keyboard_->isKeyDown(OIS::KC_RIGHT))
			camera_->yaw(-rot_scale_);

		if(keyboard_->isKeyDown(OIS::KC_LEFT))
			camera_->yaw(rot_scale_);

        bool update_cars = false;
		if(keyboard_->isKeyDown(OIS::KC_Y))
        {
            update_cars = true;
            t_ += evt.timeSinceLastFrame;
        }

		if(keyboard_->isKeyDown(OIS::KC_H))
        {
            update_cars = true;
            t_ -= evt.timeSinceLastFrame;
            if(t_ < 0.0f)
                t_ = 0.0f;
        }

        if(update_cars)
        {
            hwm_v_->cts_.update_cars(t_, hwm_v_);
        }

		if( keyboard_->isKeyDown(OIS::KC_ESCAPE) || keyboard_->isKeyDown(OIS::KC_Q) )
			return false;

       	if( keyboard_->isKeyDown(OIS::KC_F) && time_until_next_toggle_ <= 0 )
		{
			stats_on_ = !stats_on_;
			showDebugOverlay(stats_on_);
			time_until_next_toggle_ = 1;
		}

		if( keyboard_->isKeyDown(OIS::KC_T) && time_until_next_toggle_ <= 0 )
		{
			switch(filtering_)
			{
			case TFO_BILINEAR:
				filtering_ = TFO_TRILINEAR;
				aniso_ = 1;
				break;
			case TFO_TRILINEAR:
				filtering_ = TFO_ANISOTROPIC;
				aniso_ = 8;
				break;
			case TFO_ANISOTROPIC:
				filtering_ = TFO_BILINEAR;
				aniso_ = 1;
				break;
			default: break;
			}
			MaterialManager::getSingleton().setDefaultTextureFiltering(filtering_);
			MaterialManager::getSingleton().setDefaultAnisotropy(aniso_);

			showDebugOverlay(stats_on_);
			time_until_next_toggle_ = 1;
		}

		if(keyboard_->isKeyDown(OIS::KC_SYSRQ) && time_until_next_toggle_ <= 0)
		{
			std::stringstream ss;
			ss << "screenshot_" << ++num_screen_shots_ << ".png";
			window_->writeContentsToFile(ss.str());
			time_until_next_toggle_ = 0.5;
			debug_text_ = "Saved: " + ss.str();
		}

		if(keyboard_->isKeyDown(OIS::KC_R) && time_until_next_toggle_ <=0)
		{
			scene_detail_index_ = (scene_detail_index_+1)%3 ;
			switch(scene_detail_index_) {
				case 0 : camera_->setPolygonMode(PM_SOLID); break;
				case 1 : camera_->setPolygonMode(PM_WIREFRAME); break;
				case 2 : camera_->setPolygonMode(PM_POINTS); break;
			}
			time_until_next_toggle_ = 0.5;
		}

		static bool displayCameraDetails = false;
		if(keyboard_->isKeyDown(OIS::KC_P) && time_until_next_toggle_ <= 0)
		{
			displayCameraDetails = !displayCameraDetails;
			time_until_next_toggle_ = 0.5;
			if (!displayCameraDetails)
				debug_text_ = "";
		}

		// Print camera details
		if(displayCameraDetails)
			debug_text_ = "P: " + StringConverter::toString(camera_->getDerivedPosition()) +
                " " + "O: " + StringConverter::toString(camera_->getDerivedOrientation());

		// Return true to continue rendering
		return true;
	}

	virtual bool processUnbufferedMouseInput(const FrameEvent &evt)
	{

		// Rotation factors, may not be used if the second mouse button is pressed
		// 2nd mouse button - slide, otherwise rotate
		const OIS::MouseState &ms = mouse_->getMouseState();
		if( ms.buttonDown( OIS::MB_Right ) )
		{
			translate_vector_.x += ms.X.rel * 0.13;
			translate_vector_.y -= ms.Y.rel * 0.13;
		}
		else
		{
			rot_x_ = Degree(-ms.X.rel * 0.13);
			rot_y_ = Degree(-ms.Y.rel * 0.13);
		}

		return true;
	}

	virtual void moveCamera()
	{
		// Make all the changes to the camera
		// Note that YAW direction is around a fixed axis (freelook style) rather than a natural YAW
		//(e.g. airplane)
		camera_->yaw(rot_x_);
		camera_->pitch(rot_y_);
		camera_->moveRelative(translate_vector_);
	}

	virtual void showDebugOverlay(bool show)
	{
		if (debug_overlay_)
		{
			if (show)
				debug_overlay_->show();
			else
				debug_overlay_->hide();
		}
	}

	// Override frameRenderingQueued event to process that (don't care about frameEnded)
	bool frameRenderingQueued(const FrameEvent &evt)
	{

		if(window_->isClosed())	return false;

		speed_limit_ = move_scale_ * evt.timeSinceLastFrame;

		//Need to capture/update each device
		keyboard_->capture();
		mouse_->capture();
		if( joy_ ) joy_->capture();

		bool buffJ = (joy_) ? joy_->buffered() : true;

    	Ogre::Vector3 lastMotion = translate_vector_;

		//Check if one of the devices is not buffered
		if( !mouse_->buffered() || !keyboard_->buffered() || !buffJ )
		{
			// one of the input modes is immediate, so setup what is needed for immediate movement
			if (time_until_next_toggle_ >= 0)
				time_until_next_toggle_ -= evt.timeSinceLastFrame;

			// Move about 100 units per second
			move_scale_ = move_speed_ * evt.timeSinceLastFrame;
			// Take about 10 seconds for full rotation
			rot_scale_ = rotate_speed_ * evt.timeSinceLastFrame;

			rot_x_ = 0;
			rot_y_ = 0;
			translate_vector_ = Ogre::Vector3::ZERO;

		}

		//Check to see which device is not buffered, and handle it
		if( !keyboard_->buffered() )
			if( processUnbufferedKeyInput(evt) == false )
				return false;
		if( !mouse_->buffered() )
			if( processUnbufferedMouseInput(evt) == false )
				return false;

		// ramp up / ramp down speed
    	if (translate_vector_ == Ogre::Vector3::ZERO)
		{
			// decay (one third speed)
			current_speed_ -= evt.timeSinceLastFrame * 0.3;
			translate_vector_ = lastMotion;
		}
		else
		{
			// ramp up
			current_speed_ += evt.timeSinceLastFrame;

		}
		// Limit motion speed
		if (current_speed_ > 1.0)
			current_speed_ = 1.0;
		if (current_speed_ < 0.0)
			current_speed_ = 0.0;

		translate_vector_ *= current_speed_;


		if( !mouse_->buffered() || !keyboard_->buffered() || !buffJ )
			moveCamera();

		return true;
	}

	bool frameEnded(const FrameEvent &evt)
	{
		UpdateStats();
		return true;
	}

protected:
	Camera  *camera_;

	Vector3 translate_vector_;
	Real current_speed_;
	RenderWindow *window_;
	bool stats_on_;

	String debug_text_;

	unsigned int num_screen_shots_;
	float move_scale_;
	float speed_limit_;
	Degree rot_scale_;
	// just to stop toggles flipping too fast
	Real time_until_next_toggle_;
	Radian rot_x_, rot_y_;
	TextureFilterOptions filtering_;
	int aniso_;

	int scene_detail_index_;
	Real move_speed_;
	Degree rotate_speed_;
	Overlay *debug_overlay_;
	//OIS Input devices
	OIS::InputManager *input_manager_;
	OIS::Mouse    *mouse_;
	OIS::Keyboard *keyboard_;
	OIS::JoyStick *joy_;

    float t_;
    hwm_viewer *hwm_v_;
};

hwm_viewer::hwm_viewer(network *net, const char *anim_file, const char *carpath) : net_(net), caelum_system_(0)
{
    o_cars_ = new ogre_car_db();
    o_cars_->add_dir(carpath);
    FILE *fp = fopen(anim_file, "r");
    assert(fp);
    cts_.read(fp);
    fclose(fp);
}

void hwm_viewer::go()
{
    create_root();
    define_resources();
    setup_render_system();
    create_render_window();
    initialize_resource_groups();
    initialize_network();
    setup_scene();
    start_caelum();
    //setup_CEGUI();
    create_frame_listener();
    start_render_loop();
}

hwm_viewer::~hwm_viewer()
{
    // delete renderer_;
    // delete system_;
    delete o_cars_;

    delete move_listener_;
    if(caelum_system_)
    {
        caelum_system_->shutdown(false);
        caelum_system_ = 0;
    }
    delete root_;
}

void hwm_viewer::create_root()
{
    root_ = new Root();
}

void hwm_viewer::define_resources()
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

void hwm_viewer::setup_render_system()
{
    if(!root_->restoreConfig() && !root_->showConfigDialog())
        throw Exception(52, "User cancelled the config dialog!",
                        "hwm_viewer::setup_render_system()");

}

void hwm_viewer::start_caelum()
{
    caelum_system_ = new Caelum::CaelumSystem(root_, scene_manager_, Caelum::CaelumSystem::CAELUM_COMPONENTS_DEFAULT);
    root_->addFrameListener(caelum_system_);
    root_->getAutoCreatedWindow()->getViewport(0)->getTarget()->addListener(caelum_system_);
    // Set time acceleration.
    //    caelum_system_->getUniversalClock ()->setTimeScale (512);
    caelum_system_->setManageSceneFog(false);
}

void hwm_viewer::create_render_window()
{
    root_->initialise(true, "hwm viewer");
}

void hwm_viewer::initialize_resource_groups()
{
    TextureManager::getSingleton().setDefaultNumMipmaps(5);
    ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
}

void hwm_viewer::initialize_network()
{
}

static const char *statesville_mesh[] = {
    "01_-_Default",
    "02_-_Default",
    "03_-_Defauldfgt",
    "03_-_Default",
    "04_-_Defaultdf",
    "04_-_Default",
    "05_-_Defaultghkj",
    "05_-_Defaultghkjs",
    "05_-_Default",
    "07_-_Default",
    "08_-_Defadfultghj",
    "08_-_Defaultghdfj",
    "08_-_Defaultghj",
    "08_-_Default",
    "09_-_Default",
    "13_-_Default",
    "19_-_Defaultcv",
    "19_-_Default",
    "xcdfhdd",
    "xcdfhd",
    "xcdfh",
    "xc"
};

void hwm_viewer::load_terrain(SceneNode *sn)
{
    for(size_t i = 0; i < sizeof(statesville_mesh)/sizeof(statesville_mesh[0]); ++i)
    {
        const char *str = statesville_mesh[i];
        Entity *ent = scene_manager_->createEntity(boost::str(boost::format("%1%-entity") % str), boost::str(boost::format("%1%.mesh") % str));
        ent->setMaterialName(str);
        SceneNode *node = sn->createChildSceneNode();
        node->attachObject(ent);
    }
}

void hwm_viewer::setup_scene()
{
    scene_manager_ = root_->createSceneManager(ST_GENERIC, "SceneManager");
    camera_ = scene_manager_->createCamera("Camera");

    camera_->setPosition(-20, 100, 0);
    camera_->lookAt(0,0,0);
    camera_->setNearClipDistance(10);
    camera_->setFarClipDistance(10000);

    root_->getAutoCreatedWindow()->addViewport(camera_);

    //    scene_manager_->setShadowTechnique(SHADOWTYPE_STENCIL_ADDITIVE);
    //        scene_manager_->setShadowTextureSize(1 << 11);
    //        scene_manager_->setAmbientLight(ColourValue(0.5, 0.5, 0.5));

    Light *l = scene_manager_->createLight("MainLight");
    l->setType(Light::LT_POINT);
    l->setPosition(0, 800, 0);
    l->setAttenuation(2000, 1.0, 0.0, 0.0);
    l->setDiffuseColour(1.0, 1.0, 1.0);
    l->setSpecularColour(1.0, 1.0, 1.0);

    // ColourValue bgcolor(0.93, 0.86, 0.76);
    // scene_manager_->setFog(FOG_LINEAR, bgcolor, 0.001, 500, 2500);
    // vp->setBackgroundColour(bgcolor);

    // SceneNode *network_node = scene_manager_->getRootSceneNode()->createChildSceneNode();
    // //    StaticGeometry *net_sg = scene_manager_->createStaticGeometry("RoadNetwork");

    float network_node_scale = 10.0f;

    // network_node->translate(0,0,0);
    // network_node->scale(network_node_scale, network_node_scale, network_node_scale);
    // network_node->pitch(Degree(-90));

    // float static_bb[6] = {FLT_MAX, FLT_MAX, FLT_MAX,
    //                       -FLT_MAX, -FLT_MAX, -FLT_MAX};

    // int count = 0;
    // foreach(const lane &la, net_->lanes)
    // {
    //     float bb[6] = {FLT_MAX, FLT_MAX, FLT_MAX,
    //                    -FLT_MAX, -FLT_MAX, -FLT_MAX};

    //     std::string meshname = boost::str(boost::format("lane-%1%") %  count);
    //     create_lane_mesh(la,  meshname, bb);

    //     for(int i = 0; i < 3; ++i)
    //     {
    //         if(bb[i] < static_bb[i])
    //             static_bb[i] = bb[i];

    //         if(bb[i+3] > static_bb[i+3])
    //             static_bb[i+3] = bb[i+3];
    //     }

    //     Entity *lane = scene_manager_->createEntity(boost::str(boost::format("lane-%1%-entity") % count), meshname);
    //     lane->setMaterialName("Test/RoadSurface");
    //     SceneNode *lane_node = network_node->createChildSceneNode();
    //     lane_node->attachObject(lane);
    //     ++count;
    // }

    // for(int i = 0; i < 6; ++i)
    //     static_bb[i] *= network_node_scale;

    // float y_low  = static_bb[1];
    // float y_high = static_bb[4];
    // static_bb[1] = -static_bb[5];
    // static_bb[4] = -static_bb[2];
    // static_bb[5] = y_high;
    // static_bb[2] = y_low;

    // network_node->translate(-(static_bb[0]+static_bb[3])*0.5,
    //                         -(static_bb[1]+static_bb[4])*0.5,
    //                         -(static_bb[2]+static_bb[5])*0.5);

    // Vector3 dims(std::max(static_bb[3]-static_bb[0], 1.0f),
    //              std::max(static_bb[4]-static_bb[1], 1.0f),
    //              std::max(static_bb[5]-static_bb[2], 1.0f));

    // net_sg->addSceneNode(network_node);
    // scene_manager_->destroySceneNode(network_node);

    // net_sg->setRegionDimensions(dims);
    // net_sg->setOrigin(Vector3(static_bb[0], static_bb[1], static_bb[2]));
    // net_sg->build();

    // Plane plane;
    // plane.normal = Vector3::UNIT_Y;
    // plane.d = 1;
    // MeshManager::getSingleton().createPlane("Myplane",
    //                                         ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, plane,
    //                                         150000,150000,20,20,true,1,10,10,Vector3::UNIT_Z);
    // Entity *ground = scene_manager_->createEntity( "plane", "Myplane" );

    // ground->setMaterialName("Examples/GrassFloor");

    SceneNode *ground_node = scene_manager_->getRootSceneNode()->createChildSceneNode();
    ground_node->translate(1852.340698, 0.000000, -257.709320);
    ground_node->scale(network_node_scale, network_node_scale, network_node_scale);
    //    ground_node->attachObject(ground);

    vehicle_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

    o_cars_->load_meshes();

    load_terrain(ground_node);

    // count = 0;
    // foreach(const lane &la, net_->lanes)
    // {
    //     lane_cars_.push_back(o_cars_->odb.begin()->second.create_car(scene_manager_, vehicle_node_, boost::str(boost::format("Car-%1%") % count)));
    //     ++count;
    // }
}

anim_car& hwm_viewer::access_sim_car(int id)
{
    if(id >= static_cast<int>(sim_cars_.size()))
        sim_cars_.resize(id+1);
    assert(id >= 0 && id <= static_cast<int>(sim_cars_.size()));
    if(sim_cars_[id].root_ == 0)
        sim_cars_[id] = o_cars_->odb.begin()->second.create_car(scene_manager_, vehicle_node_, boost::str(boost::format("Car-%1%") % id));

    return sim_cars_[id];
}

void hwm_viewer::create_lane_mesh(const lane &la, const std::string &name, float bb[6])
{
    LogManager::getSingleton().logMessage(boost::str(boost::format("Creating lane mesh %s") % name));

    /// Create the mesh via the MeshManager
    Ogre::MeshPtr msh = MeshManager::getSingleton().createManual(name, "General");

    /// Create one submesh
    SubMesh *sub = msh->createSubMesh();

    std::vector<point> verts;
    std::vector<quad>  quads;

    const road_membership *rom = &(la.road_memberships.base_data);
    int p = -1;
    while(1)
    {
        float offsets[2] = {rom->lane_position-LANE_WIDTH*0.5,
                            rom->lane_position+LANE_WIDTH*0.5};

        rom->parent_road.dp->rep.lane_mesh(verts, quads, rom->interval, rom->lane_position, offsets);

        ++p;
        if(p >= static_cast<int>(la.road_memberships.entries.size()))
            break;
        rom = &(la.road_memberships.entries[p].data);
    }

    /// Define the vertices (each consisting of 2 groups of 3 floats)
    float *vert_data = (float*)malloc(verts.size()*2*3*sizeof (float));
    for(size_t i = 0; i < verts.size(); ++i)
    {
        vert_data[3*(2*i + 0) + 0] = verts[i].x;
        if(verts[i].x < bb[0])
            bb[0] = verts[i].x;
        else if(verts[i].x > bb[3])
            bb[3] = verts[i].x;
        vert_data[3*(2*i + 0) + 1] = verts[i].y;
        if(verts[i].y < bb[1])
            bb[1] = verts[i].y;
        else if(verts[i].y > bb[4])
            bb[4] = verts[i].y;
        vert_data[3*(2*i + 0) + 2] = verts[i].z;
        if(verts[i].z < bb[2])
            bb[2] = verts[i].z;
        else if(verts[i].z > bb[5])
            bb[5] = verts[i].z;
        vert_data[3*(2*i + 1) + 0] = 0.0f;
        vert_data[3*(2*i + 1) + 1] = 1.0f;
        vert_data[3*(2*i + 1) + 2] = 0.0f;
    }

    /// Create vertex data structure for vertices shared between submeshes
    msh->sharedVertexData = new VertexData();
    msh->sharedVertexData->vertexCount = verts.size();

    /// Create declaration (memory format) of vertex data
    VertexDeclaration *decl = msh->sharedVertexData->vertexDeclaration;
    size_t offset = 0;
    // 1st buffer
    decl->addElement(0, offset, VET_FLOAT3, VES_POSITION);
    offset += VertexElement::getTypeSize(VET_FLOAT3);
    decl->addElement(0, offset, VET_FLOAT3, VES_NORMAL);
    offset += VertexElement::getTypeSize(VET_FLOAT3);
    /// Allocate vertex buffer of the requested number of vertices (vertexCount)
    /// and bytes per vertex (offset)
    HardwareVertexBufferSharedPtr vbuf =
        HardwareBufferManager::getSingleton().createVertexBuffer(offset,
                                                                 msh->sharedVertexData->vertexCount,
                                                                 HardwareBuffer::HBU_STATIC_WRITE_ONLY);
    /// Upload the vertex data to the card
    vbuf->writeData(0, vbuf->getSizeInBytes(), vert_data, true);

    /// Set vertex buffer binding so buffer 0 is bound to our vertex buffer
    VertexBufferBinding *bind = msh->sharedVertexData->vertexBufferBinding;
    bind->setBinding(0, vbuf);

    free(vert_data);

    unsigned short *faces = (unsigned short*)malloc(quads.size()*3*2*sizeof (unsigned short));
    for(size_t i = 0; i < quads.size(); ++i)
    {
        faces[3*(2*i + 0) + 0] = quads[i].v[0];
        faces[3*(2*i + 0) + 1] = quads[i].v[2];
        faces[3*(2*i + 0) + 2] = quads[i].v[1];

        faces[3*(2*i + 1) + 0] = quads[i].v[2];
        faces[3*(2*i + 1) + 1] = quads[i].v[0];
        faces[3*(2*i + 1) + 2] = quads[i].v[3];
    }

    /// Allocate index buffer of the requested number of vertices (ibufCount)
    HardwareIndexBufferSharedPtr ibuf = HardwareBufferManager::getSingleton().
        createIndexBuffer(
                          HardwareIndexBuffer::IT_16BIT,
                          quads.size()*3*2,
                          HardwareBuffer::HBU_STATIC_WRITE_ONLY);

    /// Upload the index data to the card
    ibuf->writeData(0, ibuf->getSizeInBytes(), faces, true);

    free(faces);

    /// Set parameters of the submesh
    sub->useSharedVertices = true;
    sub->indexData->indexBuffer = ibuf;
    sub->indexData->indexCount = quads.size()*3*2;
    sub->indexData->indexStart = 0;

    /// Set bounding information (for culling)
    msh->_setBounds(AxisAlignedBox(bb[0], bb[1], bb[2], bb[3], bb[4], bb[5]));
    msh->_setBoundingSphereRadius(0.5f*std::max(std::max(bb[3]-bb[0], bb[4]-bb[1]), bb[5]-bb[2]));

    /// Notify Mesh object that it has been loaded
    msh->load();

    LogManager::getSingleton().logMessage(boost::str(boost::format("Done creating lane mesh %s") % name));
}

// void hwm_viewer::setup_CEGUI()
// {}

void hwm_viewer::create_frame_listener()
{
    move_listener_ = new hwm_frame_listener(this, root_->getAutoCreatedWindow(), camera_);
    root_->addFrameListener(move_listener_);
}

void hwm_viewer::start_render_loop()
{
    root_->startRendering();
}


int main(int argc, char **argv)
{
    network *net = new network;

    if(argc < 4)
    {
        fprintf(stderr, "Usage: %s <network-file> <car-anim-file> <car-db dir>\n", argv[0]);
        exit(1);
    }

    if(!net->load_from_xml(argv[1]))
    {
        fprintf(stderr, "Couldn't load network %s\n", argv[1]);
        exit(1);
    }

    hwm_viewer hv(net, argv[2], argv[3]);
    hv.go();

    delete net;

    return 0;
}

anim_car ogre_car_model::create_car(SceneManager *sm, SceneNode *base, const std::string &name) const
{
    const float scale = 10.0f;

    anim_car ac;

    ac.cm_ = cm;

    std::string body_name(boost::str(boost::format("%1%-body") % name));

    Entity* ent2 = sm->createEntity(body_name, bodymesh->getName() );
    //    ent2->setCastShadows(true);

    ac.root_ = base->createChildSceneNode(body_name, Vector3(0, scale + cm->z_offset + cm->wheel_diameter*0.5, 0));
    ac.root_->scale(scale, scale, scale);
    ac.root_->attachObject(ent2);

    for(int w = 0; w < 4; ++w)
    {
        std::string wheel_name(boost::str(boost::format("%1%-wheel-%2%") % name % w));
        Entity *wheel = sm->createEntity(wheel_name, wheelmesh->getName());
        //        wheel->setCastShadows(true);

        ac.wheels_[w] = ac.root_->createChildSceneNode(wheel_name, Vector3(cm->wheel_points[w][0], cm->wheel_points[w][2], cm->wheel_points[w][1]));

        if(w < 2)
            ac.wheels_[w]->yaw(Degree(180));

        ac.wheels_[w]->attachObject(wheel);
    }

    return ac;
}


static void look_for_lod(MeshPtr mesh, std::string &mesh_root, std::vector<float> &lodlevels)
{
    int count = 2;
    foreach(float lod, lodlevels)
    {
        std::string lod_name = boost::str(boost::format("%1%-lod%2%.mesh") %  mesh_root % count);

        LogManager::getSingleton().logMessage(boost::str(boost::format("Looking for lod %1% (filename %2%)") % count % lod_name));

        MeshPtr lodm;
        try
        {
            mesh->createManualLodLevel(lod, lod_name);
        }
        catch(FileNotFoundException &fnfe)
        {
            LogManager::getSingleton().logMessage(boost::str(boost::format("Couldn't find %1%; stopping lod search") % lod_name));
            return;
        }

        LogManager::getSingleton().logMessage(boost::str(boost::format("Successfully found %1%") % lod_name));
        ++count;
    }
}

void ogre_car_db::load_meshes()
{
    for(std::map<std::string, car_model>::iterator current = db.begin();
        current != db.end();
        ++current)
    {
        std::string body_mesh_name = boost::str(boost::format("%1%.mesh") %  current->second.body_mesh);

        std::string wheel_mesh_name = boost::str(boost::format("%1%.mesh") %  current->second.wheel_mesh);

        ogre_car_model ocm;
        try
        {
            LogManager::getSingleton().logMessage(boost::str(boost::format("Looking for bodymesh (%1%) for car %2%") % body_mesh_name % current->first));
            ocm.bodymesh = MeshManager::getSingleton().load(body_mesh_name,
                                                            ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        }
        catch(FileNotFoundException &fnfe)
        {
            LogManager::getSingleton().logMessage(boost::str(boost::format("Couldn't find bodymesh (%1%) for car %2%; skipping this model") % body_mesh_name % current->first));
            return;
        }
        LogManager::getSingleton().logMessage(boost::str(boost::format("Found bodymesh (%1%) for car %2%") % body_mesh_name % current->first));
        std::vector<float> lodlevels;
        for(int i = 0; i < 4; ++i)
        {
            lodlevels.push_back(200*(i+1));
        }
        //ocm.bodymesh->generateLodLevels(lodlevels, ProgressiveMesh::VRQ_PROPORTIONAL, 0.75);
        look_for_lod(ocm.bodymesh, current->second.body_mesh, lodlevels);

        try
        {
            LogManager::getSingleton().logMessage(boost::str(boost::format("Looking for wheelmesh (%1%) for car %2%") % wheel_mesh_name % current->first));
            ocm.wheelmesh = MeshManager::getSingleton().load(wheel_mesh_name,
                                                             ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        }
        catch(FileNotFoundException &fnfe)
        {
            LogManager::getSingleton().logMessage(boost::str(boost::format("Couldn't find wheelmesh (%1%) for car %2%; skipping this model") % wheel_mesh_name % current->first));
            return;
        }
        LogManager::getSingleton().logMessage(boost::str(boost::format("Found wheelmesh (%1%) for car %2%") % wheel_mesh_name % current->first));
        look_for_lod(ocm.wheelmesh, current->second.wheel_mesh, lodlevels);
        //ocm.wheelmesh->generateLodLevels(lodlevels, ProgressiveMesh::VRQ_PROPORTIONAL, 0.75);
        ocm.cm = &(current->second);
        odb[current->first] = ocm;
    }
};

#define GETS_BUFFER_SIZE 1024

void car_time_sample::from_pair(const car_time_sample &cts0,
                                const car_time_sample &cts1,
                                float                  f0,
                                float                  f1)
{
    assert(cts0.id == cts1.id);
    id           = cts0.id;
    for(int i = 0; i < 3; ++i)
        pos[i]   = cts0.pos[i]*f0 + cts1.pos[i]*f1;
    theta        = cts0.theta*f0 + cts1.theta*f1;
    velocity     = cts0.velocity*f0 + cts1.velocity*f1;
    motion_state = cts0.motion_state; //< Perhaps this could be something else
}

void car_time_sample::apply_to_car(anim_car &ac) const
{
    ac.root_->setVisible(true, true);

    ac.root_->setOrientation(std::cos(theta*0.5f), 0.0, -std::sin(theta*0.5f), 0.0);
    ac.root_->setPosition(10.0*pos[0], 10.0*(pos[2]+ac.cm_->z_offset + ac.cm_->wheel_diameter*0.5), -10.0*pos[1]);
    // should update wheels...
}

struct car_time_sample_cmp
{
    bool operator()(const car_time_sample &l, const car_time_sample &r) const
    {
        return l.id < r.id;
    }
};

int car_time_series::read(FILE *fp)
{
    int count = 0;
    char gets_buff[GETS_BUFFER_SIZE];
    while(!feof(fp))
    {
        char *res = fgets(gets_buff, GETS_BUFFER_SIZE-1, fp);
        if(res != gets_buff)
            continue;

        int ncars;
        float new_time;

        int nitems = sscanf(res, "%f %d", &new_time, &ncars);
        assert(nitems == 2);

        assert(ncars > 0);
        float last_time = samples.empty() ? -FLT_MAX : samples.back().first;
        assert(last_time < new_time);

        samples.push_back(entry(new_time, sample_vector(ncars)));

        sample_vector &current = samples.back().second;

        for(int i = 0; i < ncars; ++i)
        {
            car_time_sample &cts = current[i];

            char *res = fgets(gets_buff, GETS_BUFFER_SIZE-1, fp);
            if(res != gets_buff)
            {
                samples.pop_back();
                return count;
            }

            float n[2];

            int nitems = sscanf(res, "%d %f %f %f %f %f %f %d",
                                &(cts.id),
                                cts.pos,
                                cts.pos+1,
                                cts.pos+2,
                                n,
                                n+1,
                                &(cts.velocity),
                                &(cts.motion_state));
            assert(nitems == 8);
            assert(cts.id >=0);

            cts.theta = std::atan2(-n[1], n[0]);

            if(feof(fp))
            {
                samples.pop_back();
                return count;
            }
        }

        std::sort(current.begin(), current.end(), car_time_sample_cmp());
        ++count;
    }

    return count;
}

struct entry_cmp
{
    bool operator()(const car_time_series::entry &l, const car_time_series::entry &r) const
    {
        return l.first < r.first;
    }
};

void car_time_series::update_cars(float t, hwm_viewer *hv) const
{
    foreach(anim_car &ac, hv->sim_cars_)
    {
        if(ac.root_)
            ac.root_->setVisible(false, true);
    }

    const entry ent(t, sample_vector());
    std::vector<entry>::const_iterator res(std::lower_bound(samples.begin(), samples.end(), ent, entry_cmp()));

    if(res == samples.begin())
    {
        foreach(const car_time_sample &cts, res->second)
            cts.apply_to_car(hv->access_sim_car(cts.id));
    }
    else if(res == samples.end())
    {
        --res;
        foreach(const car_time_sample &cts, res->second)
            cts.apply_to_car(hv->access_sim_car(cts.id));
    }
    else
    {
        float t1 = res->first;
        const sample_vector &sv1(res->second);
        --res;
        float t0 = res->first;
        const sample_vector &sv0(res->second);

        float fac1 = (t - t0)/(t1-t0);
        float fac0 = 1.0f - fac1;

        foreach(const car_time_sample &cts, sv0)
        {

            std::vector<car_time_sample>::const_iterator res(std::lower_bound(sv1.begin(), sv1.end(), cts, car_time_sample_cmp()));
            if(res == sv1.end() || res->id != cts.id)
            {
                // possibly fade 'em out?
                cts.apply_to_car(hv->access_sim_car(cts.id));
                // apply to car
            }
            else
            {
                // construct avg
                car_time_sample avg;
                avg.from_pair(cts, *res, fac0, fac1);

                // apply to car
                avg.apply_to_car(hv->access_sim_car(avg.id));
            }
        }
    }

}

