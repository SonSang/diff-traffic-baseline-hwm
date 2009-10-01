#include <FL/Fl.H>
#include <FL/Fl_Gl_Window.H>
#include <FL/Fl_Menu_Button.H>
#include <FL/gl.h>
#include <FL/glu.h>
#include "FL/glut.h"
#include <cfloat>
#include "arcball.hpp"
#include "network.hpp"
#include <fstream>
#include <iostream>
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>

typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
typedef boost::tokenizer<boost::char_separator<char>,std::istream_iterator<char> > line_tokenizer;

struct car_record
{
    car_record()
    {}

    car_record(const std::string &s)
    {
        boost::char_separator<char> sep(" ");
        tokenizer                   tokens(s, sep);
        tokenizer::iterator         tok = tokens.begin();

        id = boost::lexical_cast<int>(*tok);
        x  = boost::lexical_cast<float>(*++tok);
        y  = boost::lexical_cast<float>(*++tok);
        z  = boost::lexical_cast<float>(*++tok);
        nx = boost::lexical_cast<float>(*++tok);
        ny = boost::lexical_cast<float>(*++tok);
        ++tok;
        ++tok;
    }

    int id;
    float x;
    float y;
    float z;

    float nx;
    float ny;
};

typedef std::vector<car_record> car_vec;

typedef std::map<float,car_vec> car_series;

car_series series;

car_series load_cars(const char *filename)
{
    car_series result;

    std::ifstream ins(filename);
    std::istream &is = ins;

    boost::char_separator<char> line_sep("\n", "", boost::keep_empty_tokens);
    is.unsetf(std::ios::skipws);

    line_tokenizer line_tokens(std::istream_iterator<char>(is), std::istream_iterator<char>(), line_sep);
    line_tokenizer::const_iterator linetok = line_tokens.begin();

    car_vec current_time;
    float time;
    int ncars;
    bool expect_header = true;
    while(linetok != line_tokens.end() && *linetok != "")
    {
        if(expect_header)
        {
            boost::char_separator<char> sep(" ");
            tokenizer                   tokens(*linetok, sep);
            tokenizer::iterator         tok = tokens.begin();

            time  = boost::lexical_cast<float>(*tok);
            ncars = boost::lexical_cast<int>(*++tok);
            if(ncars <= 0)
                expect_header = true;
            else
            {
                expect_header = false;
                current_time.resize(ncars);
            }
        }
        else
        {
            current_time[--ncars] = car_record(*linetok);
            if(ncars <= 0)
            {
                result.insert(std::make_pair(time, current_time));
                current_time.clear();
                expect_header = true;
            }
        }
        ++linetok;
    }

    return result;
}

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

static GLuint init_draw_car()
{
    static const float verts[][3] = {{-(CAR_LENGTH-CAR_REAR_AXLE), -0.3f*LANE_WIDTH, 0.0f},  //0
                                     {              CAR_REAR_AXLE,-0.15f*LANE_WIDTH, 0.0f},  //1
                                     {              CAR_REAR_AXLE, 0.15f*LANE_WIDTH, 0.0f},  //2
                                     {-(CAR_LENGTH-CAR_REAR_AXLE),  0.3f*LANE_WIDTH, 0.0f},  //3

                                     {-(CAR_LENGTH-CAR_REAR_AXLE), -0.3f*LANE_WIDTH, 1.5f},  //4
                                     {              CAR_REAR_AXLE,-0.15f*LANE_WIDTH, 1.3f},  //5
                                     {              CAR_REAR_AXLE, 0.15f*LANE_WIDTH, 1.3f},  //6
                                     {-(CAR_LENGTH-CAR_REAR_AXLE),  0.3f*LANE_WIDTH, 1.5f}}; //7

    static const int faces[6][4] = {{ 0, 1, 2, 3}, // top
                                    { 4, 5, 6, 7}, // bottom
                                    { 0, 4, 5, 1}, // left side
                                    { 0, 3, 7, 4}, // back
                                    { 2, 6, 7, 3}, // right side
                                    { 1, 2, 6, 5}};// front


    GLuint car_list = glGenLists(1);
    glPushMatrix();
    glNewList(car_list, GL_COMPILE);
    glBegin(GL_QUADS);
    for(int i = 0; i < 6; ++i)
        for(int j = 0; j < 4; ++j)
            glVertex3fv(&(verts[faces[i][j]][0]));
    glEnd();
    glEndList();
    glPopMatrix();

    return car_list;
}

static void draw_car()
{
    static GLuint car_list = 0;

    if(!car_list)
        car_list = init_draw_car();
    glCallList(car_list);
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

network *net;
float t;
std::vector<road_mesh> rm;

class fltkview : public Fl_Gl_Window
{
public:
    fltkview(int x, int y, int w, int h, const char *l) : Fl_Gl_Window(x, y, w, h, l), zoom(2.0),
                                                          draw_intersections(true),
                                                          draw_lanes(true),
                                                          draw_carticles(true)
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
        }

        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

        glLoadIdentity();

        glTranslatef(0.0f, 0.0f, -std::pow(2.0f, zoom));

        nav.get_rotation();
        glMultMatrixd(nav.world_mat());

        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

        if(draw_carticles && !series.empty())
        {
            car_series::iterator time_state = series.upper_bound(t);
            if(time_state == series.end())
                --time_state;
            const car_vec &vec = time_state->second;
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

            foreach(const car_record &rec, vec)
            {
                glPushMatrix();
                glTranslatef(rec.x, rec.y, rec.z);
                float theta = atan2(rec.ny, rec.nx);
                glRotatef(theta*180.0/M_PI, 0.0, 0.0, 1.0);
                draw_car();
                glPopMatrix();
            }
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
                case 'l':
                    draw_lanes = !draw_lanes;
                    printf("draw lanes = %d\n", draw_lanes);
                    break;
                case 'i':
                    draw_intersections = !draw_intersections;
                    printf("draw intersections = %d\n", draw_intersections);
                    break;
                case 'c':
                    draw_carticles = !draw_carticles;
                    printf("draw carticles = %d\n", draw_carticles);
                    break;
                case '0' ... '9':
                    zoom = zooms[Fl::event_key()-'0'];
                    break;
                case 'a':
                    t += 1.0f;
                    printf("t = %f\n", t);
                    break;
                case 'z':
                    t -= 1.0f;
                    printf("t = %f\n", t);
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
    bool draw_intersections;
    bool draw_lanes;
    bool draw_carticles;
};

int main(int argc, char * argv[])
{
    printf("%s version: %s\n", argv[0], hwm_version_string());

    if(argc < 2)
    {
        fprintf(stderr, "Usage: %s <output file> [network]\n", argv[0]);
        exit(1);
    }

    if(argc == 3)
    {
        net = new network;

        if(!net->load_from_xml(argv[1]))
        {
            fprintf(stderr, "Warning: couldn't load %s\n", argv[1]);
            delete net;
            net = 0;
        }

        net->calc_bounding_box();
        point pt;
        pt.x = -(net->bb[0]+net->bb[1])*0.5f;
        pt.y = -(net->bb[2]+net->bb[3])*0.5f;
        pt.z = 0.0f;

        printf("origin: %f %f %f\n", pt.x, pt.y, pt.z);
        net->translate(pt);

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
    }
    else
        net = 0;

    t = 0.0f;

    series = load_cars(argv[1]);
    assert(!series.empty());

    fltkview mv(0, 0, 500, 500, "fltk View");

    mv.take_focus();
    Fl::visual(FL_DOUBLE|FL_DEPTH);

    mv.show(1, argv);
    return Fl::run();
}
