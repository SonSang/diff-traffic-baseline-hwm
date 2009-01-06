#include <FL/Fl.H>
#include <FL/Fl_Gl_Window.H>
#include <FL/Fl_Menu_Button.H>
#include <FL/gl.h>
#include <FL/glu.h>
#include "FL/glut.h"
#include <cfloat>
#include "arcball.hpp"
#include "network.hpp"

struct road_mesh
{
    std::vector<point> vrts;
    std::vector<quad>  faces;

    void draw() const
    {
        glBegin(GL_QUADS);
        foreach(const quad & q, faces)
            for(int i = 0; i < 4; ++i)
                glVertex2fv(&(vrts[q.v[i]].x));
        glEnd();
    }
};

void line_rep::draw() const
{
    glBegin(GL_LINE_STRIP);
    foreach(const point & p, points)
        glVertex2fv(&(p.x));
    glEnd();
}

network *net;
float t;
std::vector<road_mesh> rm;

class fltkview : public Fl_Gl_Window
{
public:
    fltkview(int x, int y, int w, int h, const char *l) : Fl_Gl_Window(x, y, w, h, l), zoom(2.0)
    {
        lastmouse[0] = 0.0f;
        lastmouse[1] = 0.0f;
    }

    void draw()
    {
        if (!valid())
        {
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            gluPerspective(45.0f, (GLfloat)w()/(GLfloat)h(), 0.1f, 100.0f);

            glMatrixMode(GL_MODELVIEW);
            glClearColor(0.0, 0.0, 0.0, 0.0);

            glEnable(GL_DEPTH_TEST);
        }

        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

        glLoadIdentity();

        glTranslatef(0.0f, 0.0f, -std::pow(2.0, zoom));

        nav.get_rotation();
        glMultMatrixd(nav.world_mat());

        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

        glColor3f(1.0f, 1.0f, 1.0f);
        foreach(const road_mesh & i, rm)
            i.draw();

        glColor3f(1.0f, 0.0f, 1.0f);
        point p, n;

        foreach(const lane &la, net->lanes)
        {
            float x = t;
            const road_membership *rom = &(la.road_memberships.get_rescale(x));

            rom->parent_road.dp->rep.locate_vec(&p, &n, x*(rom->interval[1]-rom->interval[0])+rom->interval[0], rom->lane_position);
            glPushMatrix();
            glTranslatef(p.x, p.y, 0.0f);
            float mat[16] =
                { n.x,   n.y, 0.0f, 0.0f,
                  n.y,  -n.x, 0.0f, 0.0f,
                 0.0f,  0.0f, 1.0f, 0.0f,
                 0.0f,  0.0f, 0.0f, 1.0f};
            glMultMatrixf(mat);
            glRotatef(90.0f, 1.0f, 0.0f, 0.0f);
            glutWireTeapot(0.05f);
            glPopMatrix();
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
                if(Fl::event_button() == FL_LEFT_MOUSE)
                {
                    float fx =   2.0f*x/(w()-1) - 1.0f;
                    float fy = -(2.0f*y/(h()-1) - 1.0f);
                    nav.get_click(fx, fy);
                }
                else if(Fl::event_button() == FL_RIGHT_MOUSE)
                {
                    float fx =   2.0f*x/(w()-1) - 1.0f;
                    float fy = -(2.0f*y/(h()-1) - 1.0f);

                    lastmouse[0] = fx;
                    lastmouse[1] = fy;
                }
                else if(Fl::event_button() == FL_MIDDLE_MOUSE)
                {
                    float fx =   2.0f*x/(w()-1) - 1.0f;
                    float fy = -(2.0f*y/(h()-1) - 1.0f);

                    lastmouse[0] = fx;
                    lastmouse[1] = fy;
                }
                redraw();
            }
            take_focus();
            return 1;
        case FL_DRAG:
            {
                int x = Fl::event_x();
                int y = Fl::event_y();
                if(Fl::event_button() == FL_LEFT_MOUSE)
                {
                    float fx =  2.0f*x/(w()-1)-1.0f;
                    float fy = -(2.0f*y/(h()-1)-1.0f);
                    nav.get_click(fx, fy, 1.0f, true);
                }
                else if(Fl::event_button() == FL_RIGHT_MOUSE)
                {
                    float fx =   2.0f*x/(w()-1) - 1.0f;
                    float fy = -(2.0f*y/(h()-1) - 1.0f);
                    float scale = std::pow(2.0f, zoom-1.0f);

                    double update[3] = {
                        (fx-lastmouse[0])*scale,
                        (fy-lastmouse[1])*scale,
                        0.0f
                    };

                    nav.translate(update);

                    lastmouse[0] = fx;
                    lastmouse[1] = fy;
                }
                else if(Fl::event_button() == FL_MIDDLE_MOUSE)
                {
                    float fx =   2.0f*x/(w()-1) - 1.0f;
                    float fy = -(2.0f*y/(h()-1) - 1.0f);
                    float scale = std::pow(2.0f, zoom-1.0f);
                    zoom += scale*(fy-lastmouse[1]);
                    if(zoom > 10.0f)
                        zoom = 10.0f;
                    else if(zoom < FLT_MIN)
                        zoom = FLT_MIN;

                    lastmouse[0] = fx;
                    lastmouse[1] = fy;
                }
                redraw();
            }
            take_focus();
            return 1;
        case FL_KEYBOARD:
            {
                switch(Fl::event_key())
                {
                case 'z':
                    t-= 0.01;
                    if(t < 0.0f)
                        t = 0.0;
                    break;
                case 'a':
                    t+= 0.01;
                    if(t > 1.0f)
                        t = 1.0;
                    break;
                default:
                    return Fl_Gl_Window::handle(event);
                }
            }
            redraw();
            return 1;
        case FL_MOUSEWHEEL:
            {
            }
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
};

#define LANE_WIDTH 0.1

int main(int argc, char * argv[])
{
    net = new network;

    if(!net->load_from_xml(argv[1]))
    {
        fprintf(stderr, "Couldn't load %s\n", argv[1]);
        exit(1);
    }

    float rng[2] = {0.0f, 1.0f};
    float offsets[2];
    t = 0.0f;

    foreach(const lane &la, net->lanes)
    {
        const road_membership *rom = &(la.road_memberships.base_data);
        int p = -1;
        while(1)
        {
            float offsets[2] = {rom->lane_position-LANE_WIDTH*0.5,
                                rom->lane_position+LANE_WIDTH*0.5};

            rm.push_back(road_mesh());
            rom->parent_road.dp->rep.lane_mesh(rm.back().vrts, rm.back().faces, rom->interval, rom->lane_position, offsets);

            ++p;
            if(p >= static_cast<int>(la.road_memberships.entries.size()))
                break;
            rom = &(la.road_memberships.entries[p].data);
        }
    }

    fltkview mv(0, 0, 500, 500, "fltk View");

    mv.take_focus();
    Fl::visual(FL_DOUBLE|FL_DEPTH);

    mv.show(1, argv);
    return Fl::run();
}
