#include <GL/glew.h>
#include <FL/Fl.H>
#include <FL/Fl_Gl_Window.H>
#include <FL/Fl_Menu_Button.H>
#include <FL/gl.h>
#include <FL/glu.h>
#include "FL/glut.h"
#include "arcball.hpp"
#include "libroad/hwm_network.hpp"
#include "libroad/hwm_draw.hpp"
#include "libhybrid/hybrid-sim.hpp"

static inline void blackbody(float *rgb, const float val)
{
   if(val <= 0.0) // clamp low to black
       rgb[0] = rgb[1] = rgb[2] = 0.0f;
   else if(val >= 1.0f) // and high to white
       rgb[0] = rgb[1] = rgb[2] = 1.0f;
   else if(val < 1.0f/3.0f) // go to [1, 0, 0] over [0, 1/3)
   {
       rgb[0] = val*3.0f;
       rgb[1] = 0.0f;
       rgb[2] = 0.0f;
   }
   else if(val < 2.0f/3.0f)  // go to [1, 1, 0] over [1/3, 2/3)
   {
       rgb[0] = 1.0f;
       rgb[1] = (val-1.0f/3.0f)*3.0f;
       rgb[2] = 0.0f;
   }
   else // go to [1, 1, 1] over [2/3, 1.0)
   {
       rgb[0] = 1.0f;
       rgb[1] = 1.0f;
       rgb[2] = (val-2.0f/3.0f)*3.0f;
   }
   return;
}

static const float CAR_LENGTH = 4.5f;
//* This is the position of the car's axle from the FRONT bumper of the car
static const float CAR_REAR_AXLE = 3.5f;

class fltkview : public Fl_Gl_Window
{
public:
    fltkview(int x, int y, int w, int h, const char *l) : Fl_Gl_Window(x, y, w, h, l),
                                                          zoom(2.0),
                                                          car_pos(0.0f),
                                                          glew_state(GLEW_OK+1),
                                                          light_position(50.0, 100.0, 50.0, 1.0),
                                                          tex_(0),
                                                          sim(0),
                                                          drawfield(RHO)
    {
        lastmouse[0] = 0.0f;
        lastmouse[1] = 0.0f;

        this->resizable(this);
    }

    void setup_light()
    {
        static const GLfloat amb_light_rgba[] = { 0.1, 0.1, 0.1, 1.0 };
        static const GLfloat diff_light_rgba[] = { 0.7, 0.7, 0.7, 1.0 };
        static const GLfloat spec_light_rgba[] = { 1.0, 1.0, 1.0, 1.0 };
        static const GLfloat spec_material[] = { 1.0, 1.0, 1.0, 1.0 };
        static const GLfloat material[] = { 1.0, 1.0, 1.0, 1.0 };
        static const GLfloat shininess = 100.0;

        glEnable(GL_LIGHTING);
        glEnable(GL_LIGHT0);
        glEnable(GL_COLOR_MATERIAL);
        glPushMatrix();
        glLoadIdentity();
        glLightfv(GL_LIGHT0, GL_POSITION, light_position.data());
        glPopMatrix();
        glLightfv(GL_LIGHT0, GL_AMBIENT, amb_light_rgba );
        glLightfv(GL_LIGHT0, GL_DIFFUSE, diff_light_rgba );
        glLightfv(GL_LIGHT0, GL_SPECULAR, spec_light_rgba );
        glMaterialfv( GL_FRONT, GL_AMBIENT, material );
        glMaterialfv( GL_FRONT, GL_DIFFUSE, material );
        glMaterialfv( GL_FRONT, GL_SPECULAR, spec_material );
        glMaterialfv( GL_FRONT, GL_SHININESS, &shininess);
    }

    void init_glew()
    {
        glew_state = glewInit();
        if (GLEW_OK != glew_state)
        {
            /* Problem: glewInit failed, something is seriously wrong. */
            std::cerr << "Error: " << glewGetErrorString(glew_state)  << std::endl;
        }
        std::cerr << "Status: Using GLEW " << glewGetString(GLEW_VERSION) << std::endl;
    }

    void init_textures()
    {
        if(!glIsTexture(tex_))
        {
            glGenTextures(1, &tex_);
            glEnable(GL_TEXTURE_2D);
            glBindTexture (GL_TEXTURE_2D, tex_);
            glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
            glDisable(GL_TEXTURE_2D);
        }
    }

    void draw()
    {
        if (!valid())
        {
            glViewport(0, 0, w(), h());
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            gluPerspective(45.0f, (GLfloat)w()/(GLfloat)h(), 10.0f, 5000.0f);

            glMatrixMode(GL_MODELVIEW);
            glClearColor(0.0, 0.0, 0.0, 0.0);

            glEnable(GL_DEPTH_TEST);

            glHint(GL_LINE_SMOOTH_HINT,    GL_NICEST);
            glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);

            glEnable(GL_LINE_SMOOTH);

            if(GLEW_OK != glew_state)
                init_glew();

            if(!car_drawer.initialized())
                car_drawer.initialize(0.6*sim->hnet->lane_width,
                                      CAR_LENGTH,
                                      1.5f,
                                      CAR_REAR_AXLE);

            if(!network_drawer.initialized())
                network_drawer.initialize(sim->hnet, 0.4f);

            init_textures();

            setup_light();

            if(sim)
            {
                bb[0] = vec3f(FLT_MAX);
                bb[1] = vec3f(-FLT_MAX);
                sim->hnet->bounding_box(bb[0], bb[1]);
            }
        }

        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

        glLoadIdentity();

        glTranslatef(0.0f, 0.0f, -std::pow(2.0f, zoom));

        nav.get_rotation();
        glMultMatrixd(nav.world_mat());

        glLightfv(GL_LIGHT0, GL_POSITION, light_position.data());

        if(sim)
        {
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            glDisable(GL_LIGHTING);
            glColor3f(0.1, 0.2, 0.1);
            glPushMatrix();
            glTranslatef(0.0, 0.0, bb[0][2]-0.05f);
            glBegin(GL_QUADS);
            glVertex2f(bb[0][0], bb[0][1]);
            glVertex2f(bb[1][0], bb[0][1]);
            glVertex2f(bb[1][0], bb[1][1]);
            glVertex2f(bb[0][0], bb[1][1]);
            glEnd();
            glPopMatrix();

            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            glDisable(GL_LIGHTING);

            glColor3f(1.0, 1.0, 1.0);
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            glEnable(GL_LIGHTING);

            glEnable(GL_TEXTURE_2D);
            std::vector<vec4f> colors;
            BOOST_FOREACH(hybrid::lane *l, sim->macro_lanes)
            {
                if(!l->parent->active)
                    continue;

                glBindTexture (GL_TEXTURE_2D, tex_);

                colors.resize(l->N);
                for(size_t i = 0; i < l->N; ++i)
                {
                    float val;
                    if(drawfield == RHO)
                        val = l->q[i].rho();
                    else
                        val = arz<float>::eq::u(l->q[i].rho(),
                                                l->q[i].y(),
                                                l->parent->speedlimit,
                                                sim->gamma)/l->parent->speedlimit;

                    blackbody(colors[i].data(), val);
                    colors[i][3] = 1.0f;
                }

                glTexImage2D (GL_TEXTURE_2D,
                              0,
                              GL_RGBA,
                              l->N,
                              1,
                              0,
                              GL_RGBA,
                              GL_FLOAT,
                              colors[0].data());

                network_drawer.draw_lane_solid(l->parent->id);
            }
            glDisable(GL_TEXTURE_2D);

            BOOST_FOREACH(hybrid::lane *l, sim->micro_lanes)
            {
                if(!l->parent->active)
                    continue;

                glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                glDisable(GL_LIGHTING);
                glColor3f(0.9, 0.9, 1.0);
                network_drawer.draw_lane_wire(l->parent->id);

                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
                glEnable(GL_LIGHTING);

                glColor3f(1.0, 0.0, 0.0);
                BOOST_FOREACH(const hybrid::car &c, l->current_cars())
                {
                    assert(c.position >= 0);
                    assert(c.position < 1.0);
                    mat4x4f trans(l->parent->point_frame(c.position));
                    mat4x4f ttrans(tvmet::trans(trans));

                    glPushMatrix();
                    glMultMatrixf(ttrans.data());
                    car_drawer.draw();
                    glPopMatrix();
                }
            }

            glColor3f(0.5, 0.5, 1.0);
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            glDisable(GL_LIGHTING);
            network_drawer.draw_intersections_wire();

            glEnable(GL_BLEND);
            glDisable(GL_LIGHTING);
        }

        glFlush();
        glFinish();
    }

    int handle(int event)
    {
        switch(event)
        {
        case FL_PUSH:
            {
                int x = Fl::event_x();
                int y = Fl::event_y();
                float fx =   2.0f*x/(w()-1) - 1.0f;
                float fy = -(2.0f*y/(h()-1) - 1.0f);
                if(Fl::event_button() == FL_LEFT_MOUSE)
                {
                    nav.get_click(fx, fy);
                }
                else if(Fl::event_button() == FL_RIGHT_MOUSE)
                {
                }
                else if(Fl::event_button() == FL_MIDDLE_MOUSE)
                {
                }
                lastmouse[0] = fx;
                lastmouse[1] = fy;
                redraw();
            }
            take_focus();
            return 1;
        case FL_DRAG:
            {
                int x = Fl::event_x();
                int y = Fl::event_y();
                float fx =  2.0f*x/(w()-1)-1.0f;
                float fy = -(2.0f*y/(h()-1)-1.0f);
                if(Fl::event_button() == FL_LEFT_MOUSE)
                {
                    nav.get_click(fx, fy, 1.0f, true);
                }
                else if(Fl::event_button() == FL_RIGHT_MOUSE)
                {
                    float scale = std::pow(2.0f, zoom-1.0f);

                    double update[3] = {
                        (fx-lastmouse[0])*scale,
                        (fy-lastmouse[1])*scale,
                        0.0f
                    };

                    nav.translate(update);
                }
                else if(Fl::event_button() == FL_MIDDLE_MOUSE)
                {
                    float scale = std::pow(1.5f, zoom-1.0f);
                    zoom += scale*(fy-lastmouse[1]);
                    if(zoom > 17.0f)
                        zoom = 17.0f;
                    else if(zoom < FLT_MIN)
                        zoom = FLT_MIN;

                }
                lastmouse[0] = fx;
                lastmouse[1] = fy;
                redraw();
            }
            take_focus();
            return 1;
        case FL_KEYBOARD:
            switch(Fl::event_key())
            {
            case 'n':
                if(sim)
                {
                    BOOST_FOREACH(hwm::intersection_pair &i, sim->hnet->intersections)
                    {
                        i.second.advance_state();
                    }
                }
                break;
            case ' ':
                if(sim)
                {
                    sim->update(0.033);
                    sim->macro_step(0.033);
                }
                break;
            case 'p':
                drawfield = RHO;
                break;
            case 'u':
                drawfield = U;
                break;
            case 't':
                car_pos += 0.02f;
                if(car_pos > 1.0f)
                    car_pos = 1.0f;
                break;
            case 'g':
                car_pos -= 0.02f;
                if(car_pos < 0.0f)
                    car_pos = 0.0f;
                break;
            default:
                break;
            }
            redraw();
            return 1;
        case FL_MOUSEWHEEL:
            take_focus();
            return 1;
        default:
            // pass other events to the base class...
            return Fl_Gl_Window::handle(event);
        }
    }

    arcball nav;
    float zoom;
    float lastmouse[2];

    float             car_pos;
    hwm::car_draw     car_drawer;
    hwm::network_draw network_drawer;

    vec3f bb[2];

    GLuint            glew_state;
    vec4f             light_position;
    GLuint            tex_;
    hybrid::simulator *sim;
    typedef enum {RHO, U} draw_type;
    draw_type         drawfield;
};

int main(int argc, char *argv[])
{
    hwm::network net(hwm::load_xml_network(argv[1], vec3f(1.0, 1.0, 15.0f)));
    net.build_intersections();
    net.build_fictitious_lanes();
    net.auto_scale_memberships();
    net.center();
    std::cerr << "HWM net loaded successfully" << std::endl;

    if(net.check())
        std::cerr << "HWM net checks out" << std::endl;
    else
    {
        std::cerr << "HWM net doesn't check out" << std::endl;
        exit(1);
    }

    hybrid::simulator s(&net);
    s.micro_initialize(0.73,
                       1.67,
                       33,
                       4,
                       5,
                       1.0);
    s.macro_initialize(0.5, 10.0f, 0.0f);

    BOOST_FOREACH(hybrid::lane &l, s.lanes)
    {
        s.micro_lanes.push_back(&l);
        s.macro_lanes.push_back(&l);
    }

    static const int cars_per_lane = 3;
    BOOST_FOREACH(hybrid::lane *l, s.micro_lanes)
    {
        if(!l->parent->active)
            continue;

        double p = -s.rear_bumper_offset()*l->inv_length;
        for (int i = 0; i < cars_per_lane; i++)
        {
            //TODO Just creating some cars here...
            hybrid::car tmp;
            tmp.position = p;
            tmp.velocity = 33;
            l->current_cars().push_back(tmp);
            //Cars need a minimal distance spacing
            p += (15.0 * l->inv_length);
            if(p + s.front_bumper_offset()*l->inv_length >= 1.0)
                break;
        }
    }

    s.settle(0.033);

    s.convert_cars();
    fltkview mv(0, 0, 500, 500, "fltk View");

    mv.sim = &s;

    mv.take_focus();
    Fl::visual(FL_DOUBLE|FL_DEPTH);

    mv.show(1, argv);
    return Fl::run();
}
