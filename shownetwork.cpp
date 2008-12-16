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
        for(int i = 0; i < static_cast<int>(faces.size()); ++i)
        {
            glVertex2fv(&(vrts[faces[i].v[0]].x));
            glVertex2fv(&(vrts[faces[i].v[1]].x));
            glVertex2fv(&(vrts[faces[i].v[2]].x));
            glVertex2fv(&(vrts[faces[i].v[3]].x));
        }
        glEnd();
    }
};

road_mesh rm;

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

        rm.draw();

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
                    nav.get_click(fx, fy, 1.0, true);
                }
                else if(Fl::event_button() == FL_RIGHT_MOUSE)
                {
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

int main(int argc, char * argv[])
{
    line_rep lr;
    lr.points.push_back(point(0.0f, 0.0f));
    lr.points.push_back(point(1.0f, 0.5f));
    lr.points.push_back(point(2.0f, -0.5f));
    lr.points.push_back(point(3.0f, 0.0f));

    float rng[2] = {0.5f, 0.75f};
    float offsets[2] = {0.05f, 0.25f};

    lr.calc_rep();
    lr.lane_mesh(rm.vrts, rm.faces, rng, 0.15f, offsets);

    fltkview mv(0, 0, 500, 500, "fltk View");

    mv.take_focus();
    Fl::visual(FL_DOUBLE|FL_DEPTH);

    mv.show(1, argv);
    return Fl::run();
}
