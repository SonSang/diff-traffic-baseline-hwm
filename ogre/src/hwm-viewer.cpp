#include <Ogre.h>
#include <OgreStringConverter.h>
#include <OgreException.h>

//Use this define to signify OIS will be used as a DLL
//(so that dll import/export macros are in effect)
#define OIS_DYNAMIC_LIB
#include <OIS/OIS.h>

#include <boost/foreach.hpp>

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

			const RenderTarget::FrameStats& stats = window_->getStatistics();
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
	hwm_frame_listener(RenderWindow *win, Camera *cam, bool buffered_keys = false, bool buffered_mouse = false,
                       bool buffered_joy = false) :
		camera_(cam), translate_vector_(Vector3::ZERO), current_speed_(0), window_(win), stats_on_(true), num_screen_shots_(0),
		move_scale_(0.0f), rot_scale_(0.0f), time_until_next_toggle_(0), filtering_(TFO_BILINEAR),
		aniso_(1), scene_detail_index_(0), move_speed_(500), rotate_speed_(30), debug_overlay_(0),
		input_manager_(0), mouse_(0), keyboard_(0), joy_(0)
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

	virtual bool processUnbufferedKeyInput(const FrameEvent& evt)
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

	virtual bool processUnbufferedMouseInput(const FrameEvent& evt)
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
	bool frameRenderingQueued(const FrameEvent& evt)
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

	bool frameEnded(const FrameEvent& evt)
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
        //setup_CEGUI();
        create_frame_listener();
        start_render_loop();
    }

    ~hwm_viewer()
    {
        // delete renderer_;
        // delete system_;

        delete move_listener_;
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
        scene_manager_ = root_->createSceneManager(ST_GENERIC, "SceneManager");
        camera_ = scene_manager_->createCamera("Camera");

        camera_->setPosition(20, 0, 0);
        camera_->lookAt(0,0,0);
        camera_->setNearClipDistance(1);
        camera_->setFarClipDistance(1000);

        Viewport *vp  = root_->getAutoCreatedWindow()->addViewport(camera_);

        scene_manager_->setAmbientLight(ColourValue(0.5, 0.5, 0.5));

        Light *l = scene_manager_->createLight("MainLight");
        l->setPosition(20, 80, 50);

        // ColourValue bgcolor(0.93, 0.86, 0.76);
        // scene_manager_->setFog(FOG_LINEAR, bgcolor, 0.001, 500, 2500);
        // vp->setBackgroundColour(bgcolor);

        Plane plane;
        plane.d = 5000;
        plane.normal = -Vector3::UNIT_Y;

        createColourCube();

        // MaterialPtr material = MaterialManager::getSingleton().create(
        //                                                               "Test/ColourTest", ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        // material->getTechnique(0)->getPass(0)->setVertexColourTracking(TVC_AMBIENT);


        SceneNode *node = scene_manager_->getRootSceneNode()->createChildSceneNode();
        Entity *cube = scene_manager_->createEntity( "cc", "ColourCube" );
        cube->setMaterialName("Test/ColourTest");
        node->attachObject( cube );
        node->translate(0,0,0);
    }

void createColourCube()
    {
	/// Create the mesh via the MeshManager
	Ogre::MeshPtr msh = MeshManager::getSingleton().createManual("ColourCube", "General");

	/// Create one submesh
	SubMesh* sub = msh->createSubMesh();

	const float sqrt13 = 0.577350269f; /* sqrt(1/3) */

	/// Define the vertices (8 vertices, each consisting of 2 groups of 3 floats
	const size_t nVertices = 8;
	const size_t vbufCount = 3*2*nVertices;
	float vertices[vbufCount] = {
			-100.0,100.0,-100.0,        //0 position
			-sqrt13,sqrt13,-sqrt13,     //0 normal
			100.0,100.0,-100.0,         //1 position
			sqrt13,sqrt13,-sqrt13,      //1 normal
			100.0,-100.0,-100.0,        //2 position
			sqrt13,-sqrt13,-sqrt13,     //2 normal
			-100.0,-100.0,-100.0,       //3 position
			-sqrt13,-sqrt13,-sqrt13,    //3 normal
			-100.0,100.0,100.0,         //4 position
			-sqrt13,sqrt13,sqrt13,      //4 normal
			100.0,100.0,100.0,          //5 position
			sqrt13,sqrt13,sqrt13,       //5 normal
			100.0,-100.0,100.0,         //6 position
			sqrt13,-sqrt13,sqrt13,      //6 normal
			-100.0,-100.0,100.0,        //7 position
			-sqrt13,-sqrt13,sqrt13,     //7 normal
	};

	RenderSystem* rs = Root::getSingleton().getRenderSystem();
	RGBA colours[nVertices];
	RGBA *pColour = colours;
	// Use render system to convert colour value since colour packing varies
	rs->convertColourValue(ColourValue(1.0,0.0,0.0), pColour++); //0 colour
	rs->convertColourValue(ColourValue(1.0,1.0,0.0), pColour++); //1 colour
	rs->convertColourValue(ColourValue(0.0,1.0,0.0), pColour++); //2 colour
	rs->convertColourValue(ColourValue(0.0,0.0,0.0), pColour++); //3 colour
	rs->convertColourValue(ColourValue(1.0,0.0,1.0), pColour++); //4 colour
	rs->convertColourValue(ColourValue(1.0,1.0,1.0), pColour++); //5 colour
	rs->convertColourValue(ColourValue(0.0,1.0,1.0), pColour++); //6 colour
	rs->convertColourValue(ColourValue(0.0,0.0,1.0), pColour++); //7 colour

	/// Define 12 triangles (two triangles per cube face)
	/// The values in this table refer to vertices in the above table
	const size_t ibufCount = 36;
	unsigned short faces[ibufCount] = {
			0,2,3,
			0,1,2,
			1,6,2,
			1,5,6,
			4,6,5,
			4,7,6,
			0,7,4,
			0,3,7,
			0,5,1,
			0,4,5,
			2,7,3,
			2,6,7
	};

	/// Create vertex data structure for 8 vertices shared between submeshes
	msh->sharedVertexData = new VertexData();
	msh->sharedVertexData->vertexCount = nVertices;

	/// Create declaration (memory format) of vertex data
	VertexDeclaration* decl = msh->sharedVertexData->vertexDeclaration;
	size_t offset = 0;
	// 1st buffer
	decl->addElement(0, offset, VET_FLOAT3, VES_POSITION);
	offset += VertexElement::getTypeSize(VET_FLOAT3);
	decl->addElement(0, offset, VET_FLOAT3, VES_NORMAL);
	offset += VertexElement::getTypeSize(VET_FLOAT3);
	/// Allocate vertex buffer of the requested number of vertices (vertexCount)
	/// and bytes per vertex (offset)
	HardwareVertexBufferSharedPtr vbuf =
		HardwareBufferManager::getSingleton().createVertexBuffer(
		offset, msh->sharedVertexData->vertexCount, HardwareBuffer::HBU_STATIC_WRITE_ONLY);
	/// Upload the vertex data to the card
	vbuf->writeData(0, vbuf->getSizeInBytes(), vertices, true);

	/// Set vertex buffer binding so buffer 0 is bound to our vertex buffer
	VertexBufferBinding* bind = msh->sharedVertexData->vertexBufferBinding;
	bind->setBinding(0, vbuf);

	// 2nd buffer
	offset = 0;
	decl->addElement(1, offset, VET_COLOUR, VES_DIFFUSE);
	offset += VertexElement::getTypeSize(VET_COLOUR);
	/// Allocate vertex buffer of the requested number of vertices (vertexCount)
	/// and bytes per vertex (offset)
	vbuf = HardwareBufferManager::getSingleton().createVertexBuffer(
		offset, msh->sharedVertexData->vertexCount, HardwareBuffer::HBU_STATIC_WRITE_ONLY);
	/// Upload the vertex data to the card
	vbuf->writeData(0, vbuf->getSizeInBytes(), colours, true);

	/// Set vertex buffer binding so buffer 1 is bound to our colour buffer
	bind->setBinding(1, vbuf);

	/// Allocate index buffer of the requested number of vertices (ibufCount)
	HardwareIndexBufferSharedPtr ibuf = HardwareBufferManager::getSingleton().
		createIndexBuffer(
		HardwareIndexBuffer::IT_16BIT,
		ibufCount,
		HardwareBuffer::HBU_STATIC_WRITE_ONLY);

	/// Upload the index data to the card
	ibuf->writeData(0, ibuf->getSizeInBytes(), faces, true);

	/// Set parameters of the submesh
	sub->useSharedVertices = true;
	sub->indexData->indexBuffer = ibuf;
	sub->indexData->indexCount = ibufCount;
	sub->indexData->indexStart = 0;

	/// Set bounding information (for culling)
	msh->_setBounds(AxisAlignedBox(-100,-100,-100,100,100,100));
	msh->_setBoundingSphereRadius(Math::Sqrt(3*100*100));

	/// Notify Mesh object that it has been loaded
	msh->load();
    }

    // void setup_CEGUI()
    // {}

    void create_frame_listener()
    {
        move_listener_ = new hwm_frame_listener(root_->getAutoCreatedWindow(), camera_);
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
    hwm_frame_listener *move_listener_;

    Camera *camera_;
    SceneManager *scene_manager_;
};

int main(int argc, char **argv)
{
    hwm_viewer hv;
    hv.go();

    return 0;
}
