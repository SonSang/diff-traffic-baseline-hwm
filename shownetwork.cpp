#include <FL/Fl.H>
#include <FL/Fl_Gl_Window.H>
#include <FL/Fl_Menu_Button.H>
#include <FL/gl.h>
#include <FL/glu.h>
#include "FL/glut.h"
#include <cfloat>
#include "arcball.hpp"
#include "network.hpp"

#define LANE_WIDTH 0.1

void blackbody(float val, float rgb[3])
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

int line_rep::draw_data(float offset, const float range[2], float &leftover, int incount, float h, const q *data, float speedlimit, float gamma_c, unsigned int n) const
{
    bool backwards;
    float r[2];
    if(range[0] < range[1])
    {
        r[0] = range[0];
        r[1] = range[1];
        backwards = false;
    }
    else
    {
        r[0] = range[1];
        r[1] = range[0];
        backwards = true;
    }

    int start = find_segment(r[0], offset);

    float t0 = r[0]*offset_length(offset);
    float t1 = r[1]*offset_length(offset);

    float ncells = (t1-t0)/h;

    int data_end = std::floor(ncells);
    float next_leftover;
    if(backwards)
    {
        leftover = h - (ncells - data_end)*h - leftover;
        next_leftover = h - leftover;
    }

    float last_cmitre = start == 0 ? 0 : cmitres[start-1];

    float segstart = t0 - (clengths[start] + offset*(cmitres[start]+last_cmitre));
    float mitre = cmitres[start] - last_cmitre;
    point p0(segstart*normals[start].x + offset*(-mitre*normals[start].x - normals[start].y) + points[start].x,
             segstart*normals[start].y + offset*(-mitre*normals[start].y + normals[start].x) + points[start].y);
    segstart = t0;

    int segment = start;
    int count = 0;

    bool alldone = false;
    while(!alldone && count*h + t0 < t1)
    {
        if(segment > start)
        {
            segstart = clengths[segment] + offset*(cmitres[segment]+cmitres[segment-1]);
            mitre = cmitres[segment]-cmitres[segment-1];
            p0.x = points[segment].x + offset*(-mitre*normals[segment].x - normals[segment].y);
            p0.y = points[segment].y + offset*(-mitre*normals[segment].y + normals[segment].x);
        }

        glPushMatrix();
        glTranslatef(p0.x, p0.y, 0.0f);
        float mat[16] =
            { normals[segment].x,  normals[segment].y, 0.0f, 0.0f,
              normals[segment].y, -normals[segment].x, 0.0f, 0.0f,
              0.0f,  0.0f, 1.0f, 0.0f,
              0.0f,  0.0f, 0.0f, 1.0f};
        glMultMatrixf(mat);
        glScalef(1.0f, LANE_WIDTH*0.4f, 1.0f);

        bool done = false;
        while(!done)
        {
            float s = count*h - leftover;
            float e = s + h;
            int drawcount = backwards ? (data_end - count + incount) : (count+incount);
            float rho = data[drawcount].rho;
            float u   = to_u(rho, data[drawcount].y, speedlimit, gamma_c);
            if(e + t0 >= t1)
            {
                if(backwards)
                    leftover = next_leftover;
                else
                    leftover = e - (t1 - t0);
                e = t1 - t0;

                done = true;
                alldone = true;
            }
            else if(segment < static_cast<int>(clengths.size())-2 && e > clengths[segment+1] + offset*(cmitres[segment+1] + cmitres[segment]) - t0)
            {
                e = clengths[segment+1] + offset*(cmitres[segment+1] + cmitres[segment]) - t0;
                done = true;
            }
           else
                ++count;

            s -= (segstart - t0);
            e -= (segstart - t0);

            if(s < 0.0f)
                s = 0.0f;

            float rgb[3];
            blackbody(u, rgb);

            glColor3fv(rgb);
            glBegin(GL_QUAD_STRIP);
            glVertex3f(s, -1.0f, 0);
            glVertex3f(e, -1.0f, 0);
            glVertex3f(s, -1.0f, rho);
            glVertex3f(e, -1.0f, rho);
            glVertex3f(s,  1.0f, rho);
            glVertex3f(e,  1.0f, rho);
            glVertex3f(s,  1.0f, 0);
            glVertex3f(e,  1.0f, 0);
            glEnd();
        }
        glPopMatrix();

        ++segment;
    }

    return count+incount;
}

void lane::draw_data(float gamma_c) const
{
    int count = 0;
    float lenused = 0.0f;

    const road_membership *rom = &(road_memberships.base_data);
    int p = -1;
    while(1)
    {
        float offsets[2] = {rom->lane_position-LANE_WIDTH*0.5,
                            rom->lane_position+LANE_WIDTH*0.5};

        count += rom->parent_road.dp->rep.draw_data(rom->lane_position, rom->interval, lenused, count, h, data, speedlimit, gamma_c, ncells);

        ++p;
        if(p >= static_cast<int>(road_memberships.entries.size()))
            break;
        rom = &(road_memberships.entries[p].data);
    }
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
            if(rom->interval[0] > rom->interval[1])
            {
                n.x *= -1.0f;
                n.y *= -1.0f;
            }

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

        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        foreach(const lane &la, net->lanes)
            la.draw_data(net->gamma_c);

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
                case ' ':
                    {
                        float dt =  net->sim_step();
                        printf("dt = %f\n", dt);
                    }
                    break;
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

int main(int argc, char * argv[])
{
    net = new network;

    if(!net->load_from_xml(argv[1]))
    {
        fprintf(stderr, "Couldn't load %s\n", argv[1]);
        exit(1);
    }

    net->prepare(0.1);

    foreach(lane &la, net->lanes)
    {
        for(unsigned int i = 0; i < la.ncells; ++i)
        {
            la.data[i].rho = i < (la.ncells>>1) ? 0.45 : 0.4;
            la.data[i].y = to_y(la.data[i].rho, 0.1, la.speedlimit, net->gamma_c);
        }
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
