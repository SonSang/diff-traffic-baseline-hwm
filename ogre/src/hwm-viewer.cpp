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
	virtual void update_stats(void)
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

			const RenderTarget::FrameStats& stats = mWindow->getStatistics();
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
			gui_dbg->setCaption(mDebugText);
		}
		catch(...) { /* ignore */ }
	}

public:
	// Constructor takes a RenderWindow because it uses that to determine input context
	hwm_frame_listener(RenderWindow* win, Camera* cam, bool bufferedKeys = false, bool bufferedMouse = false,
			     bool bufferedJoy = false ) :
		mCamera(cam), mTranslateVector(Vector3::ZERO), mCurrentSpeed(0), mWindow(win), mStatsOn(true), mNumScreenShots(0),
		mMoveScale(0.0f), mRotScale(0.0f), mTimeUntilNextToggle(0), mFiltering(TFO_BILINEAR),
		mAniso(1), mSceneDetailIndex(0), mMoveSpeed(500), mRotateSpeed(30), mDebugOverlay(0),
		mInputManager(0), mMouse(0), mKeyboard(0), mJoy(0)
	{

		mDebugOverlay = OverlayManager::getSingleton().getByName("Core/DebugOverlay");

		LogManager::getSingletonPtr()->logMessage("*** Initializing OIS ***");
		OIS::ParamList pl;
		size_t windowHnd = 0;
		std::ostringstream windowHndStr;

		win->getCustomAttribute("WINDOW", &windowHnd);
		windowHndStr << windowHnd;
		pl.insert(std::make_pair(std::string("WINDOW"), windowHndStr.str()));

		mInputManager = OIS::InputManager::createInputSystem( pl );

		//Create all devices (We only catch joystick exceptions here, as, most people have Key/Mouse)
		mKeyboard = static_cast<OIS::Keyboard*>(mInputManager->createInputObject( OIS::OISKeyboard, bufferedKeys ));
		mMouse = static_cast<OIS::Mouse*>(mInputManager->createInputObject( OIS::OISMouse, bufferedMouse ));
		try {
			mJoy = static_cast<OIS::JoyStick*>(mInputManager->createInputObject( OIS::OISJoyStick, bufferedJoy ));
		}
		catch(...) {
			mJoy = 0;
		}

		//Set initial mouse clipping size
		windowResized(mWindow);

		showDebugOverlay(true);

		//Register as a Window listener
		WindowEventUtilities::addWindowEventListener(mWindow, this);
	}

	//Adjust mouse clipping area
	virtual void windowResized(RenderWindow* rw)
	{
		unsigned int width, height, depth;
		int left, top;
		rw->getMetrics(width, height, depth, left, top);

		const OIS::MouseState &ms = mMouse->getMouseState();
		ms.width = width;
		ms.height = height;
	}

	//Unattach OIS before window shutdown (very important under Linux)
	virtual void windowClosed(RenderWindow* rw)
	{
		//Only close for window that created OIS (the main window in these demos)
		if( rw == mWindow )
		{
			if( mInputManager )
			{
				mInputManager->destroyInputObject( mMouse );
				mInputManager->destroyInputObject( mKeyboard );
				mInputManager->destroyInputObject( mJoy );

				OIS::InputManager::destroyInputSystem(mInputManager);
				mInputManager = 0;
			}
		}
	}

	virtual ~hwm_frame_listener()
	{
		//Remove ourself as a Window listener
		WindowEventUtilities::removeWindowEventListener(mWindow, this);
		windowClosed(mWindow);
	}

	virtual bool processUnbufferedKeyInput(const FrameEvent& evt)
	{

        if(mKeyboard->isKeyDown(OIS::KC_C))
        {
            MaterialPtr material = MaterialManager::getSingleton().getByName("Test/ColourTest");
            material->getTechnique(0)->getPass(0)->setAmbient(drand48(), drand48(), drand48());
        }

		if(mKeyboard->isKeyDown(OIS::KC_A))
			mTranslateVector.x = -mMoveScale;	// Move camera left

		if(mKeyboard->isKeyDown(OIS::KC_D))
			mTranslateVector.x = mMoveScale;	// Move camera RIGHT

		if(mKeyboard->isKeyDown(OIS::KC_UP) || mKeyboard->isKeyDown(OIS::KC_W) )
			mTranslateVector.z = -mMoveScale;	// Move camera forward

		if(mKeyboard->isKeyDown(OIS::KC_DOWN) || mKeyboard->isKeyDown(OIS::KC_S) )
			mTranslateVector.z = mMoveScale;	// Move camera backward

		if(mKeyboard->isKeyDown(OIS::KC_PGUP))
			mTranslateVector.y = mMoveScale;	// Move camera up

		if(mKeyboard->isKeyDown(OIS::KC_PGDOWN))
			mTranslateVector.y = -mMoveScale;	// Move camera down

		if(mKeyboard->isKeyDown(OIS::KC_RIGHT))
			mCamera->yaw(-mRotScale);

		if(mKeyboard->isKeyDown(OIS::KC_LEFT))
			mCamera->yaw(mRotScale);

		if( mKeyboard->isKeyDown(OIS::KC_ESCAPE) || mKeyboard->isKeyDown(OIS::KC_Q) )
			return false;

       	if( mKeyboard->isKeyDown(OIS::KC_F) && mTimeUntilNextToggle <= 0 )
		{
			mStatsOn = !mStatsOn;
			showDebugOverlay(mStatsOn);
			mTimeUntilNextToggle = 1;
		}

		if( mKeyboard->isKeyDown(OIS::KC_T) && mTimeUntilNextToggle <= 0 )
		{
			switch(mFiltering)
			{
			case TFO_BILINEAR:
				mFiltering = TFO_TRILINEAR;
				mAniso = 1;
				break;
			case TFO_TRILINEAR:
				mFiltering = TFO_ANISOTROPIC;
				mAniso = 8;
				break;
			case TFO_ANISOTROPIC:
				mFiltering = TFO_BILINEAR;
				mAniso = 1;
				break;
			default: break;
			}
			MaterialManager::getSingleton().setDefaultTextureFiltering(mFiltering);
			MaterialManager::getSingleton().setDefaultAnisotropy(mAniso);

			showDebugOverlay(mStatsOn);
			mTimeUntilNextToggle = 1;
		}

		if(mKeyboard->isKeyDown(OIS::KC_SYSRQ) && mTimeUntilNextToggle <= 0)
		{
			std::stringstream ss;
			ss << "screenshot_" << ++mNumScreenShots << ".png";
			mWindow->writeContentsToFile(ss.str());
			mTimeUntilNextToggle = 0.5;
			mDebugText = "Saved: " + ss.str();
		}

		if(mKeyboard->isKeyDown(OIS::KC_R) && mTimeUntilNextToggle <=0)
		{
			mSceneDetailIndex = (mSceneDetailIndex+1)%3 ;
			switch(mSceneDetailIndex) {
				case 0 : mCamera->setPolygonMode(PM_SOLID); break;
				case 1 : mCamera->setPolygonMode(PM_WIREFRAME); break;
				case 2 : mCamera->setPolygonMode(PM_POINTS); break;
			}
			mTimeUntilNextToggle = 0.5;
		}

		static bool displayCameraDetails = false;
		if(mKeyboard->isKeyDown(OIS::KC_P) && mTimeUntilNextToggle <= 0)
		{
			displayCameraDetails = !displayCameraDetails;
			mTimeUntilNextToggle = 0.5;
			if (!displayCameraDetails)
				mDebugText = "";
		}

		// Print camera details
		if(displayCameraDetails)
			mDebugText = "P: " + StringConverter::toString(mCamera->getDerivedPosition()) +
						 " " + "O: " + StringConverter::toString(mCamera->getDerivedOrientation());

		// Return true to continue rendering
		return true;
	}

	virtual bool processUnbufferedMouseInput(const FrameEvent& evt)
	{

		// Rotation factors, may not be used if the second mouse button is pressed
		// 2nd mouse button - slide, otherwise rotate
		const OIS::MouseState &ms = mMouse->getMouseState();
		if( ms.buttonDown( OIS::MB_Right ) )
		{
			mTranslateVector.x += ms.X.rel * 0.13;
			mTranslateVector.y -= ms.Y.rel * 0.13;
		}
		else
		{
			mRotX = Degree(-ms.X.rel * 0.13);
			mRotY = Degree(-ms.Y.rel * 0.13);
		}

		return true;
	}

	virtual void moveCamera()
	{
		// Make all the changes to the camera
		// Note that YAW direction is around a fixed axis (freelook style) rather than a natural YAW
		//(e.g. airplane)
		mCamera->yaw(mRotX);
		mCamera->pitch(mRotY);
		mCamera->moveRelative(mTranslateVector);
	}

	virtual void showDebugOverlay(bool show)
	{
		if (mDebugOverlay)
		{
			if (show)
				mDebugOverlay->show();
			else
				mDebugOverlay->hide();
		}
	}

	// Override frameRenderingQueued event to process that (don't care about frameEnded)
	bool frameRenderingQueued(const FrameEvent& evt)
	{

		if(mWindow->isClosed())	return false;

		mSpeedLimit = mMoveScale * evt.timeSinceLastFrame;

		//Need to capture/update each device
		mKeyboard->capture();
		mMouse->capture();
		if( mJoy ) mJoy->capture();

		bool buffJ = (mJoy) ? mJoy->buffered() : true;

    	Ogre::Vector3 lastMotion = mTranslateVector;

		//Check if one of the devices is not buffered
		if( !mMouse->buffered() || !mKeyboard->buffered() || !buffJ )
		{
			// one of the input modes is immediate, so setup what is needed for immediate movement
			if (mTimeUntilNextToggle >= 0)
				mTimeUntilNextToggle -= evt.timeSinceLastFrame;

			// Move about 100 units per second
			mMoveScale = mMoveSpeed * evt.timeSinceLastFrame;
			// Take about 10 seconds for full rotation
			mRotScale = mRotateSpeed * evt.timeSinceLastFrame;

			mRotX = 0;
			mRotY = 0;
			mTranslateVector = Ogre::Vector3::ZERO;

		}

		//Check to see which device is not buffered, and handle it
		if( !mKeyboard->buffered() )
			if( processUnbufferedKeyInput(evt) == false )
				return false;
		if( !mMouse->buffered() )
			if( processUnbufferedMouseInput(evt) == false )
				return false;

		// ramp up / ramp down speed
    	if (mTranslateVector == Ogre::Vector3::ZERO)
		{
			// decay (one third speed)
			mCurrentSpeed -= evt.timeSinceLastFrame * 0.3;
			mTranslateVector = lastMotion;
		}
		else
		{
			// ramp up
			mCurrentSpeed += evt.timeSinceLastFrame;

		}
		// Limit motion speed
		if (mCurrentSpeed > 1.0)
			mCurrentSpeed = 1.0;
		if (mCurrentSpeed < 0.0)
			mCurrentSpeed = 0.0;

		mTranslateVector *= mCurrentSpeed;


		if( !mMouse->buffered() || !mKeyboard->buffered() || !buffJ )
			moveCamera();

		return true;
	}

	bool frameEnded(const FrameEvent& evt)
	{
		update_stats();
		return true;
	}

protected:
	Camera* mCamera;

	Vector3 mTranslateVector;
	Real mCurrentSpeed;
	RenderWindow* mWindow;
	bool mStatsOn;

	String mDebugText;

	unsigned int mNumScreenShots;
	float mMoveScale;
	float mSpeedLimit;
	Degree mRotScale;
	// just to stop toggles flipping too fast
	Real mTimeUntilNextToggle ;
	Radian mRotX, mRotY;
	TextureFilterOptions mFiltering;
	int mAniso;

	int mSceneDetailIndex ;
	Real mMoveSpeed;
	Degree mRotateSpeed;
	Overlay* mDebugOverlay;

	//OIS Input devices
	OIS::InputManager* mInputManager;
	OIS::Mouse*    mMouse;
	OIS::Keyboard* mKeyboard;
	OIS::JoyStick* mJoy;
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
