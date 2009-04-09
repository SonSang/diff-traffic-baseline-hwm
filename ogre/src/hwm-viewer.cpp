#include <Ogre.h>
#include <OIS/OIS.h>
#include <boost/foreach.hpp>
#include "ExampleFrameListener.h"

using namespace Ogre;

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

        ColourValue bgcolor(0.93, 0.86, 0.76);
        scene_manager_->setFog(FOG_LINEAR, bgcolor, 0.001, 500, 2500);
        vp->setBackgroundColour(bgcolor);

        Plane plane;
        plane.d = 5000;
        plane.normal = -Vector3::UNIT_Y;

        createColourCube();

        MaterialPtr material = MaterialManager::getSingleton().create(
                                                                      "Test/ColourTest", ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        material->getTechnique(0)->getPass(0)->setVertexColourTracking(TVC_AMBIENT);


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
    ExampleFrameListener *move_listener_;

    Camera *camera_;
    SceneManager *scene_manager_;
};

int main(int argc, char **argv)
{
    hwm_viewer hv;
    hv.go();

    return 0;
}
