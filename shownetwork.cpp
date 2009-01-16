#include <FL/Fl.H>
#include <FL/Fl_Gl_Window.H>
#include <FL/Fl_Menu_Button.H>
#include <FL/gl.h>
#include <FL/glu.h>
#include "FL/glut.h"
#include <cfloat>
#include "arcball.hpp"
#include "network.hpp"

#define LANE_WIDTH 2.5

static float zooms[10] = { 13.0f,
                           2.0f,
                           4.0f,
                           6.0f,
                           8.0f,
                           10.0f,
                           11.0f,
                           11.5f,
                           12.0f,
                           12.5f};

static GLuint car_list;

void init_draw_car()
{
    static const float verts[][3] = {{-CAR_LENGTH, -0.5f*LANE_WIDTH, -0.5f},  //0
                                     {       0.0f,-0.25f*LANE_WIDTH, -0.5f},  //1
                                     {       0.0f, 0.25f*LANE_WIDTH, -0.5f},  //2
                                     {-CAR_LENGTH,  0.5f*LANE_WIDTH, -0.5f},  //3

                                     {-CAR_LENGTH, -0.5f*LANE_WIDTH, 1.0f},  //4
                                     {       0.0f,-0.25f*LANE_WIDTH, 1.0f},  //5
                                     {       0.0f, 0.25f*LANE_WIDTH, 1.0f},  //6
                                     {-CAR_LENGTH,  0.5f*LANE_WIDTH, 1.0f}}; //7

    static const int faces[6][4] = {{ 0, 1, 2, 3}, // top
                                    { 4, 5, 6, 7}, // bottom
                                    { 0, 4, 5, 1}, // left side
                                    { 0, 3, 7, 4}, // back
                                    { 2, 6, 7, 3}, // right side
                                    { 1, 2, 6, 5}};// front


    car_list = glGenLists(1);
    glPushMatrix();
    glNewList(car_list, GL_COMPILE);
    glBegin(GL_QUADS);
    for(int i = 0; i < 6; ++i)
        for(int j = 0; j < 4; ++j)
            glVertex3fv(&(verts[faces[i][j]][0]));
    glEnd();

    glPopMatrix();
    glEndList();
}

void draw_car()
{
    glCallList(car_list);
}

void intersect_lines(point &res,
                     const point &o0, const point &n0,
                     const point &o1, const point &n1)
{
    float a[2] = { n0.y,  n1.y};
    float b[2] = {-n0.x, -n1.x};
    float c[2] = {o0.x*n0.y - o0.y*n0.x,
                  o1.x*n1.y - o1.y*n1.x};
    if(std::abs(n0.y) < 1e-6)
    {
        if(std::abs(n1.y) < 1e-6)
        {
            res.x = 0.5f*(o0.x + o1.x);
            res.y = 0.5f*(o0.y + o1.y);
            return;
        }
        std::swap(a[0], a[1]);
        std::swap(b[0], b[1]);
        std::swap(c[0], c[1]);
    }

    float inva0 = 1.0f/a[0];
    float denom = b[1] - a[1]*inva0*b[0];
    if(std::abs(denom) < 1e-6)
    {
        res.x = 0.5f*(o0.x + o1.x);
        res.y = 0.5f*(o0.y + o1.y);
        return;
    }

    res.y = (c[1] - a[1]*inva0 * c[0])/denom;
    res.x = (c[0] - b[0]*res.y)*inva0;
}

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
    float next_leftover = 0.0f;
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
                    leftover = h - (e - (t1 - t0));
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
            blackbody(u/speedlimit, rgb);

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

    return count;
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

void lane::draw_carticles() const
{
    foreach(const carticle &car, carticles[0])
    {
        float mat[16];
        get_matrix(car.x, mat);

        glColor3f(0.0f, 1.0f, 1.0f);
        glPushMatrix();
        mat[14] = 0.5f;
        glMultMatrixf(mat);
        draw_car();
        glPopMatrix();
    }
}

void intersection::draw() const
{
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glColor3f(0.0f, 1.0f, 0.0f);
    glBegin(GL_LINE_LOOP);
    foreach(const point& pt, shape)
        glVertex2fv(&(pt.x));
    glEnd();

    const state &st = states[current_state];
    for(size_t i = 0; i < st.in_states.size(); ++i)
    {
        if(st.in_states[i] < 0)
            continue;

        const lane *in_la = incoming[i].dp;
        point in_vec;
        point in_pt;
        in_la->get_point_and_normal(1.0, in_pt, in_vec);

        const lane *out_la = outgoing[st.in_states[i]].dp;
        point out_vec;
        point out_pt;
        out_la->get_point_and_normal(0.0, out_pt, out_vec);

        point middle;
        intersect_lines(middle,
                        in_pt, in_vec,
                        out_pt, out_vec);
        glBegin(GL_LINE_STRIP);
        glVertex2fv(&(in_pt.x));
        glVertex2fv(&(middle.x));
        glVertex2fv(&(out_pt.x));
        glEnd();

        float len = std::min(std::sqrt((middle.x-in_pt.x)*(middle.x-in_pt.x) + (middle.y-in_pt.y)*(middle.y-in_pt.y)),
                             std::sqrt((middle.x-out_pt.x)*(middle.x-out_pt.x) + (middle.y-out_pt.y)*(middle.y-out_pt.y)));
        point o(0.5f*(in_pt.x + middle.x),0.5f*(in_pt.y + middle.y));

        glBegin(GL_LINE_STRIP);
        glVertex2f(o.x - len*0.1*(in_vec.y + in_vec.x), o.y + len*0.1*(in_vec.x - in_vec.y));
        glVertex2f(o.x, o.y);
        glVertex2f(o.x + len*0.1*(in_vec.y - in_vec.x), o.y - len*0.1*(in_vec.x + in_vec.y));
        glEnd();

        o.x = 0.5f*(out_pt.x + middle.x);
        o.y = 0.5f*(out_pt.y + middle.y);
        glBegin(GL_LINE_STRIP);
        glVertex2f(o.x - len*0.1*(out_vec.y + out_vec.x), o.y + len*0.1*(out_vec.x - out_vec.y));
        glVertex2f(o.x, o.y);
        glVertex2f(o.x + len*0.1*(out_vec.y - out_vec.x), o.y - len*0.1*(out_vec.x + out_vec.y));
        glEnd();
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
        this->resizable(this);
    }

    void draw()
    {
        if (!valid())
        {
            glViewport(0, 0, w(), h());
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            gluPerspective(45.0f, (GLfloat)w()/(GLfloat)h(), 5.0f, 100000.0f);

            glMatrixMode(GL_MODELVIEW);
            glClearColor(0.0, 0.0, 0.0, 0.0);

            glEnable(GL_DEPTH_TEST);

            if(!glIsList(car_list))
                init_draw_car();
        }

        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

        glLoadIdentity();

        glTranslatef(0.0f, 0.0f, -std::pow(2.0f, zoom));

        nav.get_rotation();
        glMultMatrixd(nav.world_mat());

        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

        glTranslatef(-(net->bb[0] + net->bb[1])*0.5f,
                     -(net->bb[2] + net->bb[3])*0.5f,
                     0.0f);
        glColor3f(1.0f, 1.0f, 1.0f);
        foreach(const road_mesh & i, rm)
            i.draw();

        point p, n;

        foreach(const lane &la, net->lanes)
        {
            float mat[16];
            la.get_matrix(t, mat);
            p.x = mat[12];
            p.y = mat[13];

            glColor3f(1.0f, 0.0f, 1.0f);
            glPushMatrix();
            mat[14] = 0.5f;
            glMultMatrixf(mat);
            draw_car();
            glPopMatrix();

            glColor3f(0.0, 1.0, 0.0);
            float x = t;
            lane *left_lane = la.left_adjacency(x);

            if(left_lane)
            {
                point lp;
                left_lane->get_point(x, lp);

                glBegin(GL_LINES);
                glVertex2fv(&(p.x));
                glVertex2fv(&(lp.x));
                glEnd();
            }
            x = t;
            lane *right_lane = la.right_adjacency(x);

            if(right_lane)
            {
                point rp;
                right_lane->get_point(x, rp);

                glBegin(GL_LINES);
                glVertex2fv(&(p.x));
                glVertex2fv(&(rp.x));
                glEnd();
            }
        }

        // glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        // foreach(const lane &la, net->lanes)
        //     la.draw_data(net->gamma_c);

        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        foreach(const lane &la, net->lanes)
            la.draw_carticles();

        glPushMatrix();
        glTranslatef(0.0f, 0.0f, 0.01f);
        foreach(const intersection &is, net->intersections)
            is.draw();
        glPopMatrix();

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
                    float scale = std::pow(1.5f, zoom-1.0f);
                    zoom += scale*(fy-lastmouse[1]);
                    if(zoom > 17.0f)
                        zoom = 17.0f;
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
                case '0' ... '9':
                    zoom = zooms[Fl::event_key()-'0'];
                    break;
                case ' ':
                    {
                        float dt =  net->sim_step();
                        printf("dt = %f\n", dt);
                    }
                    break;
                case 's':
                    {
                        float scale = std::pow(1.5f, zoom-1.0f);
                        if(std::isfinite(scale))
                            zoom += scale*0.2f;
                        if(!std::isfinite(scale) ||zoom > 17.0f)
                            zoom = 17.0f;
                        else if(zoom < FLT_MIN)
                            zoom = FLT_MIN;
                    }
                    break;
                case 'n':
                    {
                        float scale = std::pow(1.5f, zoom-1.0f);
                        if(std::isfinite(scale))
                            zoom += -scale*0.02f;
                        if(!std::isfinite(scale) || zoom > 17.0f)
                            zoom = 17.0f;
                        else if(zoom < FLT_MIN)
                            zoom = FLT_MIN;
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
    net->calc_bounding_box();
    net->prepare(2.5);

    foreach(lane &la, net->lanes)
    {
        for(unsigned int i = 0; i < la.ncells; ++i)
        {
            la.data[i].rho = i < (la.ncells>>1) ? 0.45 : 0.4;
            la.data[i].y = to_y(la.data[i].rho, 4.5, la.speedlimit, net->gamma_c);
        }
        for(int j = 0; j < 1; ++j)
            la.carticles[0].push_back(j+0.9);
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
    const road_mesh *rom = &(rm[0]);
    net->bb[0] = net->bb[2] = FLT_MAX;
    net->bb[1] = net->bb[3] = -FLT_MAX;

    foreach(const point &pt, rom->vrts)
    {
        if(pt.x < net->bb[0])
            net->bb[0] = pt.x;
        else if(pt.x > net->bb[1])
            net->bb[1] = pt.x;
        if(pt.y < net->bb[2])
            net->bb[2] = pt.y;
        else if(pt.y > net->bb[3])
            net->bb[3] = pt.y;
    }

    fltkview mv(0, 0, 500, 500, "fltk View");

    mv.take_focus();
    Fl::visual(FL_DOUBLE|FL_DEPTH);

    mv.show(1, argv);
    return Fl::run();
}
