#include <Magick++.h>
#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glext.h>
#include <FL/Fl.H>
#include <FL/Fl_Gl_Window.H>
#include <FL/Fl_Menu_Button.H>
#undef None // Stupid X11/X.h defines this!
#include <boost/regex.hpp>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <png.h>
#include "libroad/hwm_network.hpp"
#include "libroad/geometric.hpp"
#include "libhybrid/hybrid-sim.hpp"
#include "libroad/hwm_draw.hpp"
#include "libhybrid/timer.hpp"
#include "big-image-tile.hpp"
#include "night-render.hpp"
#include "car-animation.hpp"

struct fltkview;
#include "helper-window.hpp"

static const float FRAME_RATE                   = 1.0/24.0;
static const float EXPIRE_TIME                  = 2.0f;
static const float BRAKING_THRESHOLD            = -1.0f;
static const float ROAD_SURFACE_COLOR[3]        = {    237/255.0,     234/255.0,     186/255.0};
static const float ROAD_LINE_COLOR[3]           = {    135/255.0,     103/255.0,      61/255.0};
static const float REGION_BOX_BORDER_COLOR[4]   = {    246/255.0,     255/255.0,       0/255.0, 1.0};
static const float REGION_BOX_INTERNAL_COLOR[4] = {    246/255.0,     255/255.0,       0/255.0, 0.15};
static const float ROADBLOCK_FLASH_COLOR[4]     = {    246/255.0,     227/255.0,       0/255.0, 1.0};
static const float ROAD_LINE_SCALE              = 300.0;
static const float ROADBLOCK_SCALE              = 0.5;
static const float ROADBLOCK_FLASH_SCALE        = 15.0;
static const float ROADBLOCK_FLASH_OFFSET       = 0.2;
static const float ROADBLOCK_FLASH_INTERVAL     = 0.4;
static const char  RESOURCE_ROOT_ENV_NAME[]     = "TRAFFIC_2D_RESOURCE_ROOT";
static const char  ARROW_TEX[]                  = "arrow.png";
static const char  ROADBLOCK_TEX[]              = "roadblock.png";
static const char  ROADBLOCK_FLASH_TEX[]        = "roadblock-flash.png";
static char       *RESOURCE_ROOT                = 0;

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

   for(int i = 0; i < 3; ++i)
       rgb[i] = std::min(1.0f, rgb[i]*1.5f);

   return;
}

static const float car_colors [][4] = {{0.843137254902,0.0745098039216,0.0745098039216},
                                       {0.450980392157,0.788235294118,1.0},
                                       {0.0,0.447058823529,0.0941176470588},
                                       {1.0,0.945098039216,0.0941176470588},
                                       {0.98431372549,0.901960784314,0.450980392157},
                                       {0.0,0.239215686275,0.533333333333},
                                       {0.811764705882,0.811764705882,0.811764705882},
                                       {0.18431372549,0.18431372549,0.18431372549},
                                       {0.992156862745,0.992156862745,0.992156862745},
                                       {0.286274509804,0.376470588235,0.76862745098},
                                       {0.858823529412,0.792156862745,0.41568627451},
                                       {0.474509803922,0.407843137255,0.243137254902},
                                       {0.345098039216,0.345098039216,0.345098039216},
                                       {0.450980392157,0.0705882352941,0.607843137255},
                                       {0.117647058824,0.388235294118,0.16862745098},
                                       {0.698039215686,0.0,0.0823529411765},
                                       {0.980392156863,0.952941176471,0.921568627451},
                                       {0.0,0.376470588235,0.101960784314},
                                       {0.121568627451,0.16862745098,0.819607843137},
                                       {0.18431372549,0.18431372549,0.18431372549},
                                       {0.498039215686,0.435294117647,0.254901960784},
                                       {0.929411764706,0.929411764706,0.929411764706},
                                       {0.827450980392,0.737254901961,0.658823529412},
                                       {0.0745098039216,0.247058823529,0.866666666667},
                                       {0.0,0.396078431373,0.145098039216},
                                       {0.486274509804,0.21568627451,0.725490196078},
                                       {0.364705882353,0.639215686275,1.0},};

static size_t n_car_colors = sizeof(car_colors)/sizeof(car_colors[0]);

static GLint biggest_texture()
{
    GLint biggest_width = 64;
    GLint width = 1;
    while ( width ) /* use a better condition to prevent possible endless loop */
    {
        glTexImage2D(GL_PROXY_TEXTURE_2D,
                     0,                /* mip map level */
                     GL_RGBA,          /* internal format */
                     biggest_width,     /* width of image */
                     biggest_width,    /* height of image */
                     0,                /* texture border */
                     GL_RGBA,          /* pixel data format, */
                     GL_UNSIGNED_BYTE, /* pixel data type */
                     NULL              /* null pointer because this a proxy texture */
                     );

        /* the queried width will NOT be 0, if the texture format is supported */
        glGetTexLevelParameteriv(GL_PROXY_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &width);

        ++biggest_width;
    }
    return biggest_width;
}

typedef enum { LEFT, CENTER_X, RIGHT }  text_alignment_x;
typedef enum { TOP, CENTER_Y, BOTTOM }  text_alignment_y;

static void put_text(cairo_t * cr, const std::string &str, float x, float y, const text_alignment_x align_x, const text_alignment_y align_y)
{
    cairo_save (cr);

    cairo_font_extents_t fe;
    cairo_text_extents_t te;

    cairo_font_extents (cr, &fe);
    cairo_text_extents (cr, str.c_str(), &te);

    switch(align_x)
    {
    case LEFT:
        x += -te.x_bearing;
        break;
    case CENTER_X:
        x += -te.width/2-te.x_bearing;
        break;
    case RIGHT:
        x += - (te.x_bearing + te.width);
        break;
    default:
        assert(0);
    };

    switch(align_y)
    {
    case TOP:
        y +=  - te.y_bearing;
        break;
    case CENTER_Y:
        y += te.height/2;
        break;
    case BOTTOM:
        y +=  0.0;
        break;
    default:
        assert(0);
    };

    cairo_move_to(cr, x, y);

    cairo_show_text(cr, str.c_str());

    cairo_restore(cr);
}

static const char *fshader =
"uniform sampler2D full_tex, body_tex;                                    \n"
"uniform float     opacity;                                               \n"
"                                                                         \n"
"void main()                                                              \n"
"{                                                                        \n"
"vec4               color   = texture2D(full_tex, gl_TexCoord[0].st);     \n"
"color.a                   *= opacity;                                    \n"
"float              mask    = texture2D(body_tex, gl_TexCoord[0].st).r;   \n"
"gl_FragColor               = mix(color, gl_Color*color, mask);           \n"
"}                                                                        \n";

struct car_draw_info
{
    car     *thecar;
    bool     braking;
    mat4x4f  frame;
};

struct tex_car_draw
{
    tex_car_draw() : full_tex(0), body_tex(0), car_list(0)
    {
    }

    tex_car_draw(const float        car_width_,
                 const float        car_length_,
                 const float        car_height_,
                 const float        car_rear_axle_,
                 const std::string &str)
    {
        initialize(car_width_, car_length_, car_height_, car_rear_axle_, str);
    }

    bool initialized() const
    {
        return glIsTexture(full_tex) && glIsTexture(body_tex);
    }

    void initialize(const float        car_width_,
                    const float        car_length_,
                    const float        car_height_,
                    const float        car_rear_axle_,
                    const std::string &str)
    {
        assert(bf::is_directory(str));

        bf::path full(bf::path(str) / "full.png");
        bf::path body(bf::path(str) / "body-stencil.png");

        car_width     = car_width_;
        car_length    = car_length_;
        car_height    = car_height_;
        car_rear_axle = car_rear_axle_;

        Magick::Image full_im(full.string());
        const vec2i dim((int)full_im.columns(), (int)full_im.rows());
        if(dim[0] > dim[1])
           extents = vec2f((float)1.0, static_cast<float>(dim[1])/dim[0]);
        else
            extents = vec2f(static_cast<float>(dim[0])/dim[1], 1.0);

        unsigned char *pix = new unsigned char[dim[0]*dim[1]*4];

        glGenTextures(1, &full_tex);
        glBindTexture (GL_TEXTURE_2D, full_tex);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

        full_im.write(0, 0, dim[0], dim[1], "RGBA", Magick::CharPixel, pix);
        gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGBA, dim[0], dim[1],
                          GL_RGBA, GL_UNSIGNED_BYTE, pix);

        Magick::Image body_im(body.string());
        glGenTextures(1, &body_tex);
        glBindTexture (GL_TEXTURE_2D, body_tex);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

        body_im.write(0, 0, dim[0], dim[1], "R", Magick::CharPixel, pix);
        gluBuild2DMipmaps(GL_TEXTURE_2D, GL_LUMINANCE4, dim[0], dim[1],
                          GL_LUMINANCE, GL_UNSIGNED_BYTE, pix);

        delete[] pix;

        GLuint shader = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(shader, 1, &fshader, 0);
        glCompileShader(shader);

        printShaderInfoLog(shader);

        fprogram = glCreateProgram();
        glAttachShader(fprogram, shader);
        glLinkProgram(fprogram);
        glError();

        full_uniform_location    = glGetUniformLocation(fprogram, "full_tex");
        body_uniform_location    = glGetUniformLocation(fprogram, "body_tex");
        opacity_uniform_location = glGetUniformLocation(fprogram, "opacity");

        car_list = glGenLists(1);
        glNewList(car_list, GL_COMPILE);
        draw_full_car();
        glEndList();
    }

    void draw_start() const
    {
        glUseProgram(fprogram);

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, full_tex);
        glUniform1i(full_uniform_location, 0);

        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, body_tex);
        glUniform1i(body_uniform_location, 1);
    }

    void draw_car_list(float opacity=1.0) const
    {
        glUniform1f(opacity_uniform_location, opacity);
        glCallList(car_list);
    }

    void draw_full_car() const
    {
        glPushMatrix();
        glTranslatef(-car_length+car_rear_axle, 0, 0);
        glScalef(car_length, car_length/2, 1);
        glBegin(GL_QUADS);
        glTexCoord2f(0, 0);
        glVertex2f  (0,-extents[1]);
        glTexCoord2f(1 ,0);
        glVertex2f  (1,-extents[1]);
        glTexCoord2f(1 ,1);
        glVertex2f  (1, extents[1]);
        glTexCoord2f(0, 1);
        glVertex2f  (0, extents[1]);
        glEnd();
        glPopMatrix();
    }

    void draw_end() const
    {
        glUseProgram(0);
        glActiveTexture(GL_TEXTURE0);
    }

    float  car_width;
    float  car_length;
    float  car_height;
    float  car_rear_axle;
    vec2f  extents;
    GLuint full_tex;
    GLuint body_tex;
    GLuint fprogram;
    GLuint car_list;

    GLint full_uniform_location;
    GLint body_uniform_location;
    GLint opacity_uniform_location;

    std::tr1::unordered_map<size_t, car_draw_info> members;
};

static const char *pointVertexShader =
    "void main()                                                            \n"
    "{                                                                      \n"
    "    gl_PointSize = 10*gl_Point.size;                                  \n"
    "    gl_TexCoord[0] = gl_MultiTexCoord0;                                \n"
    "    gl_Position = ftransform();                                        \n"
    "    gl_FrontColor = gl_Color;                                          \n"
    "}                                                                      \n";

static const char *pointPixelShader =
    "uniform sampler2D splatTexture;                                        \n"
    "void main()                                                            \n"
    "{                                                                      \n"
    "    vec4 color =  gl_Color * texture2D(splatTexture, gl_TexCoord[0].st); \n"
    "    gl_FragColor =                                                     \n"
    "         color;\n"
    "}                                                                      \n";


static unsigned char* point_tex(int N)
{
    unsigned char *B = new unsigned char[4*N*N];

    const float inc = 2.0f/(N-1);

    float y = -1.0f;
    for (int j=0; j < N; j++)
    {
        float x = -1.0f;
        for (int i=0; i < N; i++)
        {
            unsigned char v =  (x*x + y*y) > 1.0f ? 0 : 255;
            int idx = (j*N + i)*4;
            B[idx+3] = B[idx+2] = B[idx+1] = B[idx] = v;
            x += inc;
        }
        y += inc;
    }
    return B;
}

struct view_path
{
    view_path(float psize) : active_point(-1),
                             program(0), texture(0), point_size(psize), obey_scale(false), obey_position(true)
    {}

    void clear()
    {
        extracted.clear();
        path         = arc_road();
        active_point = -1;
        scales.clear();
    }

    int pick_point(const vec3f &point, float dist)
    {
        float min_d = dist;
        int   pick  = -1;
        for(size_t i = 0; i < path.points_.size(); ++i)
        {
            const float current_distance = distance(point, path.points_[i]);
            if(current_distance < min_d)
            {
                min_d = current_distance;
                pick = i;
            }
        }
        return active_point = pick;
    }

    int add_point(const vec3f &point)
    {
        path.points_.push_back(point);
        const size_t new_size = path.points_.size();

        if(path.points_.size() > 2)
        {
            arc_road new_path;

            new_path.initialize_from_polyline(1.0, path.points_, false);
            path = new_path;
            update_extracted(0.01);
        }
        if(path.points_.size() != new_size)
            return -1;
        else
            return static_cast<int>(path.points_.size())-1;
    }

    void update_active(const vec3f &point)
    {
        if(active_point < 0 || active_point >= static_cast<int>(path.points_.size()))
            return;
        path.points_[active_point] = point;
        const size_t size = path.points_.size();

        if(path.points_.size() > 2)
        {
            arc_road new_path;
            new_path.initialize_from_polyline(1.0, path.points_, false);
            path = new_path;
            update_extracted(0.01);
        }

        if(path.points_.size() != size)
            active_point = -1;
    }

    void set_active_point(int p)
    {
        if(p >=0 && p < static_cast<int>(path.points_.size()))
            active_point = p;
        else
            active_point = -1;
    }

    void update_extracted(float resolution)
    {
        extracted.clear();
        path.extract_center(extracted, vec2f((float)0.0, (float)1.0), 0.0, resolution);
    }

    bool get_scale(float &scale, float t)
    {
        if(scales.empty())
            return false;
        else
        {
            float local;
            partition01<float>::const_iterator pt(scales.find_rescale(t, local));
            if(boost::next(pt) == scales.end())
                scale = pt->second;
            else
                scale = pt->second * (1-local)  + boost::next(pt)->second * local;
            return true;
        }
    }

    void insert_scale_keyframe(float t, float scale)
    {
        scales.insert(t, scale);
    }

    void initialize()
    {
        if(!glIsProgram(program))
        {
            GLuint vertex_shader = glCreateShader(GL_VERTEX_SHADER);
            printShaderInfoLog(vertex_shader);
            GLuint pixel_shader  = glCreateShader(GL_FRAGMENT_SHADER);
            printShaderInfoLog(pixel_shader);

            glShaderSource(vertex_shader, 1, &pointVertexShader, 0);
            glShaderSource(pixel_shader,  1,  &pointPixelShader, 0);

            glCompileShader(vertex_shader);
            glCompileShader(pixel_shader);

            program = glCreateProgram();

            glAttachShader(program, vertex_shader);
            glAttachShader(program, pixel_shader);

            glLinkProgram(program);
        }

        if(!glIsTexture(texture))
        {
            const int      resolution = 32;
            unsigned char *data       = point_tex(resolution);

            glGenTextures(1, &texture);
            glBindTexture(GL_TEXTURE_2D, texture);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, resolution, resolution, 0,
                         GL_RGBA, GL_UNSIGNED_BYTE, data);

            delete[] data;
        }
        glEnable(GL_POINT_SPRITE);
        glTexEnvi(GL_POINT_SPRITE, GL_COORD_REPLACE, GL_TRUE);
        glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
    }

    void draw(float scale) const
    {
        glColor4f(1.0, 1.0, 0.0, 1.0);
        glDisable(GL_TEXTURE_2D);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glLineWidth(1.0);
        glBegin(GL_LINE_STRIP);
        for(int i = 0; i < active_point; ++i)
        {
            glVertex3fv(path.points_[i].data());
        }
        if(active_point > -1)
        {
            glColor4f(0.0, 1.0, 1.0, 1.0);
            glVertex3fv(path.points_[active_point].data());
            glColor4f(1.0, 1.0, 0.0, 1.0);
        }
        for(size_t i = active_point+1; i < path.points_.size(); ++i)
        {
            glVertex3fv(path.points_[i].data());
        }
        glEnd();

        glColor4f(0.5, 0.5, 1.0, 1.0);
        glBegin(GL_LINE_STRIP);
        BOOST_FOREACH(const vertex &v, extracted)
        {
            glVertex3fv(v.position.data());
        }
        glEnd();

        glEnable(GL_TEXTURE_2D);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        glPointSize(5*point_size/scale);

        glError();

        glUseProgram(program);
        GLuint texLoc = glGetUniformLocation(program, "splatTexture");
        glUniform1i(texLoc, 0);

        glBindTexture(GL_TEXTURE_2D, texture);

        glColor4f(1.0, 1.0, 0.0, 1.0);
        glBegin(GL_POINTS);
        for(int i = 0; i < active_point; ++i)
        {
            glVertex3fv(path.points_[i].data());
        }
        if(active_point > -1)
        {
            glColor4f(0.0, 1.0, 1.0, 1.0);
            glVertex3fv(path.points_[active_point].data());
            glColor4f(1.0, 1.0, 0.0, 1.0);
        }
        for(size_t i = active_point+1; i < path.points_.size(); ++i)
        {
            glVertex3fv(path.points_[i].data());
        }
        glEnd();

        glUseProgram(0);
    }

    arc_road               path;
    std::vector<vertex>    extracted;
    int                    active_point;
    partition01<float>     scales;

    GLuint program;
    GLuint texture;

    float point_size;
    bool  obey_scale;
    bool  obey_position;
};

static void draw_roadblock(const hybrid::roadblock &r, float width, float aspect)
    {
        const mat4x4f trans(r.l->parent->point_frame(r.p));
        const mat4x4f ttrans(tvmet::trans(trans));

        glPushMatrix();
        glMultMatrixf(ttrans.data());
        glRotatef(90.0, 0.0, 0.0, 1.0);
        glBegin(GL_QUADS);
        glTexCoord2f(0.0, 0.0);
        glVertex2f(-width/2, -ROADBLOCK_SCALE/2);
        glTexCoord2f(1.0/ROADBLOCK_SCALE*width/aspect, 0.0);
        glVertex2f(width/2, -ROADBLOCK_SCALE/2);
        glTexCoord2f(1.0/ROADBLOCK_SCALE*width/aspect, 1.0);
        glVertex2f(width/2, ROADBLOCK_SCALE/2);
        glTexCoord2f(0.0, 1.0);
        glVertex2f(-width/2, ROADBLOCK_SCALE/2);
        glEnd();
        glPopMatrix();
    }

static void draw_roadblock_lights(const hybrid::roadblock &r, float t, float width)
{
    const mat4x4f trans(r.l->parent->point_frame(r.p));
    const mat4x4f ttrans(tvmet::trans(trans));

    const float flash_state = std::fmod(t, 2*ROADBLOCK_FLASH_INTERVAL);
    float neg_opacity;
    float pos_opacity;
    if(flash_state > ROADBLOCK_FLASH_INTERVAL)
    {
        const float param = (flash_state - ROADBLOCK_FLASH_INTERVAL)/ROADBLOCK_FLASH_INTERVAL;
        neg_opacity = std::pow(param, 6.0f);
        pos_opacity = std::pow(1.0f-param, 6.0f);
    }
    else
    {
        const float param = flash_state/ROADBLOCK_FLASH_INTERVAL;
        pos_opacity = std::pow(param, 6.0f);
        neg_opacity = std::pow(1.0f-param, 6.0f);
    }

    glPushMatrix();
    glMultMatrixf(ttrans.data());
    glRotatef(90.0, 0.0, 0.0, 1.0);
    glPushMatrix();
    glTranslatef(-width/2*(1.0-ROADBLOCK_FLASH_OFFSET), 0.0, 0.0);
    glScalef(ROADBLOCK_FLASH_SCALE, ROADBLOCK_FLASH_SCALE, 1.0);
    glColor4f(neg_opacity*ROADBLOCK_FLASH_COLOR[0], neg_opacity*ROADBLOCK_FLASH_COLOR[1], neg_opacity*ROADBLOCK_FLASH_COLOR[2], neg_opacity*ROADBLOCK_FLASH_COLOR[3]);
    glTranslatef(-0.5, -0.5, 0.0);
    glBegin(GL_QUADS);
    glTexCoord2f(0, 0);
    glVertex2f  (0, 0);
    glTexCoord2f(1, 0);
    glVertex2f  (1, 0);
    glTexCoord2f(1, 1);
    glVertex2f  (1, 1);
    glTexCoord2f(0, 1);
    glVertex2f  (0, 1);
    glEnd();
    glPopMatrix();
    glTranslatef(width/2*(1.0-ROADBLOCK_FLASH_OFFSET), 0.0, 0.0);
    glScalef(ROADBLOCK_FLASH_SCALE, ROADBLOCK_FLASH_SCALE, 1.0);
    glColor4f(pos_opacity*ROADBLOCK_FLASH_COLOR[0], pos_opacity*ROADBLOCK_FLASH_COLOR[1], pos_opacity*ROADBLOCK_FLASH_COLOR[2], pos_opacity*ROADBLOCK_FLASH_COLOR[3]);
    glTranslatef(-0.5, -0.5, 0.0);
    glBegin(GL_QUADS);
    glTexCoord2f(0, 0);
    glVertex2f  (0, 0);
    glTexCoord2f(1, 0);
    glVertex2f  (1, 0);
    glTexCoord2f(1, 1);
    glVertex2f  (1, 1);
    glTexCoord2f(0, 1);
    glVertex2f  (0, 1);
    glEnd();
    glPopMatrix();
}

struct roadblock_adder
{
    bool set_candidates(std::vector<hwm::network_aux::road_spatial::entry> &qr)
    {
        candidate_lanes.clear();
        BOOST_FOREACH(hwm::network_aux::road_spatial::entry &e, qr)
        {
            BOOST_FOREACH(hwm::network_aux::road_rev_map::lane_cont::value_type &lcv, *e.lc)
            {
                hwm::lane    *hwm_l = lcv.second.lane;
                hybrid::lane *hyb_l = hwm_l->user_data<hybrid::lane>();
                candidate_lanes.push_back(hyb_l);
            }
        }
        if(candidate_lanes.empty())
            return false;

        active_pick = 0;
        param = 0.5;
        return true;
    }

    void clear()
    {
        candidate_lanes.clear();
    }

    void next_candidate()
    {
        if(active())
            active_pick = (active_pick + 1) % candidate_lanes.size();
    }

    bool active()
    {
        return !candidate_lanes.empty();
    }

    hybrid::roadblock test_roadblock() const
    {
        hybrid::roadblock r;
        r.l = candidate_lanes[active_pick];
        r.p = param;
        return r;
    }

    std::vector<hybrid::lane*> candidate_lanes;
    int                        active_pick;
    float                      param;
};

static const float CAR_LENGTH    = 4.5f;
//* This is the position of the car's axle from the FRONT bumper of the car
static const float CAR_REAR_AXLE = 3.5f;
static const float FRONT_BUMPER_OFFSET = 3.5f;
static const float REAR_BUMPER_OFFSET = -1.0f;

static void draw_callback(void *v);

class fltkview : public Fl_Gl_Window
{
public:
    typedef enum {REGION_MANIP=0, ARC_MANIP=1, MC_PREVIEW=2, BACK_MANIP=3, NONE=4} interaction_mode;

    typedef enum {NET_NONE=0, NET_ABSTRACT=1, NET_TEXTURE=2} network_draw_mode;

    fltkview(int x, int y, int w, int h) : Fl_Gl_Window(x, y, w, h, 0),
                                                          lastpick(0),
                                                          net(0),
                                                          netaux(0),
                                                          glew_state(GLEW_OK+1),
                                                          anim(0),
                                                          back_image(0),
                                                          back_image_overlay(0),
                                                          back_image_center(0),
                                                          back_image_scale(1),
                                                          back_image_yscale(1),
                                                          overlay_tex_(0),
                                                          continuum_tex_(0),
                                                          arrow_tex_(0),
                                                          roadblock_tex_(0),
                                                          roadblock_flash_tex_(0),
                                                          drawing(false),
                                                          network_draw(NET_TEXTURE),
                                                          draw_intersections(false),
                                                          t(0),
                                                          time_offset(0),
                                                          sim_time_scale(1),
                                                          go(false),
                                                          last_drawer_update(-1),
                                                          avg_dt(0),
                                                          avg_step_time(0),
                                                          screenshot_mode(0),
                                                          screenshot_count(0),
                                                          do_lights(true),
                                                          do_cars(true),
                                                          bg_saturation(1.0),
                                                          fg_saturation(1.0),
                                                          view(1.0f),
                                                          path_param(0.0f),
                                                          path_param_rate(0.01f),
                                                          path_auto_advance(false),
                                                          imode(NONE),
                                                          throttle(true),
                                                          show_lengths(true)
    {
        this->resizable(this);
        frame_timer.reset();
        frame_timer.start();
    }

    ~fltkview()
    {
        BOOST_FOREACH(tex_car_draw *tcd, car_drawers)
        {
            delete tcd;
        }
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

    void init_car_drawers(const std::string &dir)
    {
        assert(bf::is_directory(dir));
        bf::directory_iterator end_itr;
        for( bf::directory_iterator itr(dir);
             itr != end_itr;
             ++itr)
        {
            if(! (itr->path().has_filename() && bf::is_directory(itr->path())))
                continue;
            if(bf::exists(itr->path() / "full.png"))
            {
                std::cout << "car drawers adding " << itr->path() << std::endl;
                car_drawers.push_back(new tex_car_draw(2,
                                                       CAR_LENGTH,
                                                       1.5f,
                                                       CAR_REAR_AXLE,
                                                       itr->path().string()));
            }
            else
            {
                std::cout << "car drawers skipping " << itr->path() << std::endl;
            }
        }

        for(int c = 0; c < anim->cars_n; ++c)
        {
            car *current       = anim->cars + c;
            current->body_idx  = rand() % car_drawers.size();
            current->color_idx = rand() % n_car_colors;
        }
    }

    void init_textures()
    {
        GLint biggest_width = biggest_texture();
        std::cout << "Largest texture I support: " << biggest_width << std::endl;
        if(back_image && back_image->tiles.empty())
        {
            back_image->make_tiles(biggest_width/2, true);
        }
        if(back_image_overlay && back_image_overlay->tiles.empty())
        {
            back_image_overlay->make_tiles(biggest_width/2, true);
        }
        if(!glIsTexture(overlay_tex_))
        {
            glGenTextures(1, &overlay_tex_);
            glBindTexture (GL_TEXTURE_2D, overlay_tex_);

            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
            retex_overlay(center, scale, vec2i(w(), h()), screenshot_mode);
        }
        if(!glIsTexture(continuum_tex_))
        {
            glGenTextures(1, &continuum_tex_);
            glBindTexture (GL_TEXTURE_2D, continuum_tex_);

            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
        }
        if(!glIsTexture(arrow_tex_))
        {
            glGenTextures(1, &arrow_tex_);
            glBindTexture (GL_TEXTURE_2D, arrow_tex_);

            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

            const std::string arrow_path((bf::path(RESOURCE_ROOT) / ARROW_TEX).string());
            std::cout << "Looking for arrow texture in " << arrow_path << std::endl;
            Magick::Image aim(arrow_path);
            unsigned char *pix = new unsigned char[aim.columns()*aim.rows()*4];
            arrow_aspect = aim.columns()/static_cast<float>(aim.rows());
            aim.write(0, 0, aim.columns(), aim.rows(), "RGBA", Magick::CharPixel, pix);
            gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGBA8, aim.columns(), aim.rows(),
                              GL_RGBA, GL_UNSIGNED_BYTE, pix);
            delete[] pix;
        }
        if(!glIsTexture(roadblock_tex_))
        {
            glGenTextures(1, &roadblock_tex_);
            glBindTexture (GL_TEXTURE_2D, roadblock_tex_);

            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

            const std::string  roadblock_path((bf::path(RESOURCE_ROOT) / ROADBLOCK_TEX).string());
            std::cout << "Looking for roadblock texture in " << roadblock_path << std::endl;
            Magick::Image      rim(roadblock_path);
            unsigned char     *pix = new unsigned char[rim.columns()*rim.rows()*4];
            rim.write(0, 0, rim.columns(), rim.rows(), "RGBA", Magick::CharPixel, pix);
            roadblock_aspect = rim.columns()/static_cast<float>(rim.rows());
            gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGBA8, rim.columns(), rim.rows(),
                              GL_RGBA, GL_UNSIGNED_BYTE, pix);
            delete[] pix;
        }
        if(!glIsTexture(roadblock_flash_tex_))
        {
            glGenTextures(1, &roadblock_flash_tex_);
            glBindTexture (GL_TEXTURE_2D, roadblock_flash_tex_);

            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

            const std::string  roadblock_flash_path((bf::path(RESOURCE_ROOT) / ROADBLOCK_FLASH_TEX).string());
            std::cout << "Looking for roadblock_flash texture in " << roadblock_flash_path << std::endl;
            Magick::Image      rim(roadblock_flash_path);
            unsigned char     *pix = new unsigned char[rim.columns()*rim.rows()*4];
            rim.write(0, 0, rim.columns(), rim.rows(), "RGBA", Magick::CharPixel, pix);
            gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGBA8, rim.columns(), rim.rows(),
                              GL_RGBA, GL_UNSIGNED_BYTE, pix);
            delete[] pix;
        }
    }

    void retex_overlay(const vec2f &my_center, const float my_scale, const vec2i &im_res, bool text)
    {
        cairo_surface_t *cs = cairo_image_surface_create(CAIRO_FORMAT_ARGB32,
                                                         im_res[0],
                                                         im_res[1]);
        cairo_t         *cr = cairo_create(cs);
        cairo_set_source_rgba(cr, 0.0, 0, 0, 0.0);
        cairo_set_operator(cr, CAIRO_OPERATOR_SOURCE);
        cairo_paint(cr);

        cairo_identity_matrix(cr);

        cairo_set_line_cap(cr, CAIRO_LINE_CAP_ROUND);

        cairo_translate(cr,
                        im_res[0]/2,
                        im_res[1]/2);

        cairo_scale(cr,
                    im_res[0]/my_scale,
                    im_res[1]/my_scale);


        if(im_res[0] > im_res[1])
            cairo_scale(cr,
                        static_cast<float>(im_res[1])/im_res[0],
                        1.0);
        else
            cairo_scale(cr,
                        1.0,
                        static_cast<float>(im_res[0])/im_res[1]);

        cairo_translate(cr,
                        -my_center[0],
                        -my_center[1]);
        cairo_matrix_t cmat;
        cairo_get_matrix(cr, &cmat);

        if(imode == REGION_MANIP)
        {
            cairo_set_operator(cr, CAIRO_OPERATOR_OVER);
            BOOST_FOREACH(const aabb2d &r, rectangles)
            {
                cairo_set_matrix(cr, &cmat);
                cairo_rectangle(cr, r.bounds[0][0], r.bounds[0][1], r.bounds[1][0]-r.bounds[0][0], r.bounds[1][1]-r.bounds[0][1]);
                cairo_set_source_rgba(cr, REGION_BOX_INTERNAL_COLOR[0], REGION_BOX_INTERNAL_COLOR[1], REGION_BOX_INTERNAL_COLOR[2], REGION_BOX_INTERNAL_COLOR[3]);
                cairo_fill_preserve(cr);
                cairo_set_source_rgba(cr, REGION_BOX_BORDER_COLOR[0], REGION_BOX_BORDER_COLOR[1], REGION_BOX_BORDER_COLOR[2], REGION_BOX_BORDER_COLOR[3]);
                cairo_identity_matrix(cr);
                cairo_set_line_width(cr, 2.0);
                cairo_stroke(cr);
            }

            if(drawing)
            {
                const vec2f low(std::min(first_point[0], second_point[0]),
                                std::min(first_point[1], second_point[1]));
                const vec2f high(std::max(first_point[0], second_point[0]),
                                 std::max(first_point[1], second_point[1]));
                cairo_set_matrix(cr, &cmat);
                cairo_rectangle(cr, low[0], low[1], high[0]-low[0], high[1]-low[1]);
                cairo_set_source_rgba(cr, REGION_BOX_INTERNAL_COLOR[0], REGION_BOX_INTERNAL_COLOR[1], REGION_BOX_INTERNAL_COLOR[2], REGION_BOX_INTERNAL_COLOR[3]);
                cairo_fill_preserve(cr);
                cairo_set_source_rgba(cr, REGION_BOX_BORDER_COLOR[0], REGION_BOX_BORDER_COLOR[1], REGION_BOX_BORDER_COLOR[2], REGION_BOX_BORDER_COLOR[3]);
                cairo_identity_matrix(cr);
                cairo_set_line_width(cr, 2.0);
                cairo_stroke(cr);
            }
        }

        cairo_identity_matrix(cr);

        cairo_set_font_size (cr, 20);
        cairo_select_font_face (cr, "SANS",
                                CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
        cairo_set_source_rgba (cr, 1.0f, 1.0f, 1.0f, 1.0f);

        cairo_translate(cr,
                        im_res[0]/2,
                        im_res[1]/2);
        cairo_scale(cr, 1, -1);
        cairo_translate(cr,
                        -im_res[0]/2,
                        -im_res[1]/2);
        if(text)
        {
            const int seconds                         = static_cast<int>(std::fmod(t+time_offset, night_setup.day_length));
            const int milliseconds                    = static_cast<int>((std::fmod(t+time_offset, night_setup.day_length) - seconds)*1000);
            const boost::posix_time::time_duration td = boost::posix_time::seconds(seconds) +
                                                        boost::posix_time::milliseconds(milliseconds);
            put_text(cr, boost::str(boost::format("real time:     %8.3fs") % t), 10, 5, LEFT, TOP);
            put_text(cr, boost::str(boost::format("time of day: %s") % boost::posix_time::to_simple_string(td)), 10, 30, LEFT, TOP);
            put_text(cr, boost::str(boost::format("scaling factor %8.3fx") % sim_time_scale), 270, 5, LEFT, TOP);

            put_text(cr, boost::str(boost::format("# of cars: %ld") % anim->cars_n), 10, 55, LEFT, TOP);

            if(throttle)
                put_text(cr, "throttle", w()-10, 5, RIGHT, TOP);
            if(go)
            {
                cairo_set_source_rgba (cr, 1.0f, 0.0f, 0.0f, 1.0f);
                put_text(cr, "simulating", 10, 75, LEFT, TOP);
                cairo_set_source_rgba (cr, 1.0f, 1.0f, 1.0f, 1.0f);
                put_text(cr, boost::str(boost::format("avg. dt %8.3fs; avg. step time %8.3fs") % avg_dt % avg_step_time), 10, 95, LEFT, TOP);
            }
            cairo_set_source_rgba (cr, 1.0f, 1.0f, 0.0f, 1.0f);
            switch(imode)
            {
            case ARC_MANIP:
                    put_text(cr, "arc manip", w()-10, h()-5, RIGHT, BOTTOM);
                    if(view.path.points_.size() > 2)
                        put_text(cr, boost::str(boost::format("path length: %6.3f") % view.path.length(0)), w()-10, h()-55, RIGHT, BOTTOM);
                break;
            case MC_PREVIEW:
                    put_text(cr, "motion preview", w()-10, h()-5, RIGHT, BOTTOM);
                break;
            case BACK_MANIP:
                put_text(cr, "back manip", w()-10, h()-5, RIGHT, BOTTOM);
                break;
            case REGION_MANIP:
                put_text(cr, "region manip", w()-10, h()-5, RIGHT, BOTTOM);
                break;
            default:
                break;
            };
            cairo_set_source_rgba (cr, 1.0f, 1.0f, 1.0f, 1.0f);
        }

        cairo_destroy(cr);

        glBindTexture(GL_TEXTURE_2D, overlay_tex_);
        glTexImage2D (GL_TEXTURE_2D,
                      0,
                      GL_RGBA,
                      im_res[0],
                      im_res[1],
                      0,
                      GL_BGRA,
                      GL_UNSIGNED_BYTE,
                      cairo_image_surface_get_data(cs));

        cairo_surface_destroy(cs);
    }

    void draw_init()
    {
        std::cout << "Shader version is " << glGetString(GL_SHADING_LANGUAGE_VERSION) << std::endl;
        std::cout << "Window size is " << w() << " " << h() << std::endl;
        glViewport(0, 0, w(), h());
        glClearColor(0.0, 0.0, 0.0, 1.0);

        if(GLEW_OK != glew_state)
            init_glew();

        if(!network_drawer.initialized())
            network_drawer.initialize(net, 0.01f);

        hwm::road_metrics rm;
        rm.lane_width      = net->lane_width;
        rm.shoulder_width  = 2.0f;
        rm.line_width      = 0.125;
        rm.line_sep_width  = 0.125;
        rm.line_length     = 3.0f;
        rm.line_gap_length = 9.0f;

        if(!network_aux_drawer.initialized())
            network_aux_drawer.initialize(netaux, rm, 0.01f);

        init_textures();
        view.initialize();
        if(car_drawers.empty())
            init_car_drawers(RESOURCE_ROOT);

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        night_setup.initialize(RESOURCE_ROOT, FRONT_BUMPER_OFFSET, REAR_BUMPER_OFFSET, vec2i(w(), h()));

        glEnable(GL_MULTISAMPLE);
        glEnable(GL_TEXTURE_2D);
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glDisable(GL_LIGHTING);
        glDepthFunc(GL_LEQUAL);
    }

    void draw_background()
    {//doesn't change state
        if(back_image && !back_image->tiles.empty())
        {
            glPushMatrix();
            glTranslatef(-back_image_center[0], -back_image_center[1], 0);
            glScalef    (back_image_scale, back_image_yscale*back_image_scale,   1);

            vec2i dim(back_image->dim());
            glTranslatef(-dim[0]/2, dim[1]/2, 0);
            back_image->draw();
            glPopMatrix();
        }

    }

    void draw_background_overlay()
    {//doesn't change state
        if(back_image_overlay && !back_image_overlay->tiles.empty())
        {
            glPushMatrix();
            glTranslatef(-back_image_center[0], -back_image_center[1], 0);
            glScalef    (back_image_scale, back_image_yscale*back_image_scale,   1);

            vec2i dim(back_image_overlay->dim());
            glTranslatef(-dim[0]/2, dim[1]/2, 0);
            back_image_overlay->draw();
            glPopMatrix();
        }
    }

    void draw_network()
    {//assumes texture_2d, may unset it
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(20000.0/scale, 0.0);

        switch(network_draw)
        {
        case NET_ABSTRACT:
            {
                glDisable(GL_TEXTURE_2D);
                glColor3fv(ROAD_SURFACE_COLOR);
                network_aux_drawer.draw_roads_solid();

                const float line_width = ROAD_LINE_SCALE/scale;
                if(line_width > 1)
                {
                    glLineWidth(line_width);
                    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                    glColor3fv(ROAD_LINE_COLOR);
                    network_aux_drawer.draw_roads_wire();
                    network_aux_drawer.draw_intersections_wire();
                    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
                }
                glColor3fv(ROAD_SURFACE_COLOR);
                network_aux_drawer.draw_intersections_solid();
                break;
            }
        case NET_TEXTURE:
            glColor3f(1.0, 1.0, 1.0);
            glEnable(GL_TEXTURE_2D);
            network_aux_drawer.draw_roads_solid();
            glDisable(GL_TEXTURE_2D);
            glColor3f(0.4, 0.4, 0.4);
            network_aux_drawer.draw_intersections_solid();
            break;
        case NET_NONE:
            break;
        }

        glDisable(GL_POLYGON_OFFSET_FILL);
    }

    void draw_intersection_arrows()
    {//assumes texture_2d, no state change
        if(draw_intersections)
        {
            const float arrow_length = arrow_aspect*net->lane_width;
            glBindTexture (GL_TEXTURE_2D, arrow_tex_);
            glMatrixMode(GL_TEXTURE);
            BOOST_FOREACH(const hwm::intersection_pair &ip, net->intersections)
            {
                if(ip.second.locked)
                    continue;
                const hwm::intersection::state &s = ip.second.states[ip.second.current_state];
                BOOST_FOREACH(const hwm::lane_pair &lp, s.fict_lanes)
                {
                    const hybrid::lane *hl = lp.second.user_data<const hybrid::lane>();
                    glLoadIdentity();
                    glTranslatef(1.0-hl->length/arrow_length, 0.0, 0.0);
                    glScalef(hl->length/arrow_length, 1.0, 1.0);

                    network_drawer.draw_lane_solid(lp.first);
                }
            }
            glLoadIdentity();
            glMatrixMode(GL_MODELVIEW);
        }
    }

    void draw_cars()
    {//assumes texture_2d, doesn't change state
        glDepthMask(GL_FALSE);
        BOOST_FOREACH(tex_car_draw *drawer, car_drawers)
        {
            drawer->draw_start();
            typedef std::pair<const size_t, car_draw_info> id_car_draw_info;
            BOOST_FOREACH(id_car_draw_info &car, drawer->members)
            {
                glColor3fv(car_colors[car.second.thecar->color_idx]);

                glPushMatrix();
                glMultMatrixf(car.second.frame.data());
                drawer->draw_car_list(1.0);
                glPopMatrix();
            }
            drawer->draw_end();
        }
        glDepthMask(GL_TRUE);
    }

    void draw_cars_lights()
    {//assumes texture_2d, doesn't change state
        BOOST_FOREACH(tex_car_draw *drawer, car_drawers)
        {
            typedef std::pair<const size_t, car_draw_info> id_car_draw_info;
            BOOST_FOREACH(const id_car_draw_info &car, drawer->members)
            {
                glPushMatrix();
                glMultMatrixf(car.second.frame.data());
                night_setup.draw_car_lights(1.0, car.second.braking);
                glPopMatrix();
            }
        }
    }

    void update_drawers(float t)
    {
        BOOST_FOREACH(tex_car_draw *drawer, car_drawers)
        {
            drawer->members.clear();
        }

        car_at_time *cars          = 0;
        int          cars_n        = 0;
        int          cars_n_allocd = 0;
        cars_at_time(&cars, &cars_n, &cars_n_allocd, anim, t);

        for(int c = 0; c < cars_n; ++c)
        {
            car_at_time  *time_car = cars + c;
            car          *the_car  = anim->cars + time_car->car_idx;

            assert(the_car->body_idx >= 0 && the_car->body_idx < car_drawers.size());
            tex_car_draw *tcd      = car_drawers[the_car->body_idx];

            car_draw_info draw_info;
            draw_info.thecar = the_car;

            assert(time_car->frame_idx >= 0 && time_car->frame_idx < the_car->frames_n-1);
            car_frame *f0 = the_car->frames+time_car->frame_idx;
            car_frame *f1 = the_car->frames+time_car->frame_idx+1;

            for(int i = 0; i < 4; ++i)
                for(int j = 0; j < 4; ++j)
                {
                    if(i==j)
                        draw_info.frame(i, j) = 1.0f;
                    else
                        draw_info.frame(i, j) = 0.0f;
                }

            assert( t >= f0->time && t <= f1->time);
            float s = (t - f0->time)/(f1->time-f0->time);
            for(int i = 0; i < 3; ++i)
                draw_info.frame(3, i) = f0->position[i]*(1-s) + s*f1->position[i];

            float acceleration = f0->acceleration*(1-s) + s*f1->acceleration;
            draw_info.braking = acceleration < BRAKING_THRESHOLD;

            draw_info.frame(0,0) = f0->direction[0]*(1-s) + s*f1->direction[0];
            draw_info.frame(0,1) = f0->direction[1]*(1-s) + s*f1->direction[1];
            draw_info.frame(1,0) = f0->direction[1]*(1-s) + s*f1->direction[1];
            draw_info.frame(1,1) = -(f0->direction[0]*(1-s) + s*f1->direction[0]);

            tcd->members.insert(std::make_pair(the_car->id, draw_info));
        }

        free(cars);

        last_drawer_update = t;
    }

    void draw()
    {
        frame_timer.stop();
        if(go)
        {
            if(screenshot_mode)
            {
                t += sim_time_scale*FRAME_RATE;
                if(path_auto_advance)
                    path_param = std::min(1.0f, path_param + FRAME_RATE*path_param_rate);
            }
            else
            {
                t += sim_time_scale*frame_timer.interval_S();
                if(path_auto_advance)
                    path_param = std::min(1.0f, path_param + FRAME_RATE*path_param_rate);
            }
            if(path_auto_advance)
            {
                view_slider->value(path_param);
                view_slider->redraw();
            }
        }
        frame_timer.reset();
        frame_timer.start();

        if (!valid())
            draw_init();

        if(last_drawer_update != t)
            update_drawers(t);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();

        if((imode == NONE || imode == REGION_MANIP || imode == MC_PREVIEW) && view.path.points_.size() > 2)
        {
            if(view.obey_position)
                center = sub<0,2>::vector(view.path.point(path_param, 0));
            if(view.obey_scale)
                view.get_scale(scale, path_param);
        }

        vec2f lo, hi;
        cscale_to_box(lo, hi, center, scale, vec2i(w(), h()));
        glOrtho(lo[0], hi[0], lo[1], hi[1], -50.0, 50.0);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glRotatef(-90.0f, 0.0f, 0.0f, 1.0f);

        night_setup.start_to_light();
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
        {
            glColor4f(1.0, 1.0, 1.0, 1.0);
            draw_background();
            glEnable(GL_DEPTH_TEST);
            draw_network();
            glColor4f(0.0, 0.0, 0.0, 1.0);
            glDisable(GL_TEXTURE_2D);
            glEnable(GL_TEXTURE_2D);
            glColor4f(1.0, 1.0, 1.0, 1.0);
            draw_intersection_arrows();
            if(do_cars)
                draw_cars();
        }
        night_setup.finish_to_light();
        glError();
        night_setup.start_lum();
        {
            glClear(GL_COLOR_BUFFER_BIT);
            if(do_lights)
            {
                glDepthMask(GL_FALSE);
                glBlendFunc(GL_ONE, GL_ONE);
                if(do_cars && night_setup.draw_lights(t+time_offset))
                    draw_cars_lights();
                glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                glDepthMask(GL_TRUE);
                glDisable(GL_DEPTH_TEST);
            }
        }
        night_setup.finish_lum();
        glLoadIdentity();
        night_setup.compose(t+time_offset, lo, hi, bg_saturation, fg_saturation);
        glRotatef(-90.0f, 0.0f, 0.0f, 1.0f);
        vec4f ambient_color_vec;
        sub<0,3>::vector(ambient_color_vec) = night_setup.ambient_color(t);
        ambient_color_vec[3]                = 1.0f;
        glColor4fv(&ambient_color_vec[0]);
        draw_background_overlay();

        glLoadIdentity();
        if(imode == ARC_MANIP)
            view.draw(scale);

        glColor4f(1.0, 1.0, 1.0, 1.0);
        glBindTexture (GL_TEXTURE_2D, overlay_tex_);
        retex_overlay(center, scale, vec2i(w(), h()), !screenshot_mode);
        glPushMatrix();
        glBegin(GL_QUADS);
        glTexCoord2f(0.0, 0.0);
        glVertex2fv(lo.data());
        glTexCoord2f(1.0, 0.0);
        glVertex2f(hi[0], lo[1]);
        glTexCoord2f(1.0, 1.0);
        glVertex2fv(hi.data());
        glTexCoord2f(0.0, 1.0);
        glVertex2f(lo[0], hi[1]);
        glEnd();
        glPopMatrix();

        glError();

        if(screenshot_mode)
        {
            glFinish();
            screenshot();
        }
    }

    void screenshot()
    {
        const std::string new_file(boost::str(boost::format("screenshot%05d.dump")%screenshot_count++));
        int               f = open(new_file.c_str(), O_CREAT | O_TRUNC | O_RDWR, 00664);
        assert(f != -1);

        struct header
        {
            char magic[5];
            int w;
            int h;
            int bpp;
            char format[5];
        };

        size_t fsize = sizeof(unsigned char)*w()*h()*4 + sizeof(header);
        header head;
        strncpy(head.magic, "DUMP", 5);
        head.w       = w();
        head.h       = h();
        head.bpp     = sizeof(unsigned char);
        strncpy(head.format, "BGRA", 5);
        write(f, &head, sizeof(header));

        lseek(f, fsize, SEEK_SET);
        int m = 0;
        write(f, &m, 1);

        void *newbuff = mmap(0, fsize, PROT_WRITE | PROT_READ, MAP_SHARED, f, 0);
        assert(newbuff && newbuff != MAP_FAILED);
        close(f);

        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glPixelStorei(GL_PACK_ALIGNMENT,   1);
        glReadBuffer(GL_BACK);
        glReadPixels(0, 0, w(), h(), GL_BGRA, GL_UNSIGNED_BYTE, static_cast<unsigned char*>(newbuff)+sizeof(header));

        munmap(newbuff, fsize);
        std::cout << "Wrote " << new_file << " : " << fsize << " bytes" << std::endl;
    }

    void set_throttle()
    {
        if(throttle)
        {
            std::cout << "Throttling FPS to " << FRAME_RATE << std::endl;
            Fl::remove_idle(draw_callback, this);
            Fl::add_timeout(FRAME_RATE, draw_callback, this);
        }
        else
        {
            std::cout << "Removing throttling" << std::endl;
            Fl::remove_timeout(draw_callback);
            Fl::add_idle(draw_callback, this);
        }
    }

    int handle(int event)
    {
        switch(event)
        {
        case FL_FOCUS:
            return 1;
        case FL_PUSH:
            {
                const vec2i xy(Fl::event_x(),
                               Fl::event_y());
                const vec2f world(world_point(vec2i(xy[0], h()-xy[1]), center, scale, vec2i(w(), h())));

                if(Fl::event_button() == FL_LEFT_MOUSE)
                {
                    switch(imode)
                    {
                    case REGION_MANIP:
                        first_point  = world;
                        break;
                    case ARC_MANIP:
                        {
                            const int pt = view.add_point(vec3f(world[0], world[1], 0.0f));
                            view.set_active_point(pt);
                            if(pt > 0)
                                drawing      = true;
                            break;
                        }
                    case NONE:
                        {
                            aabb2d r;
                            r.enclose_point(world[0], world[1]);
                            std::vector<hwm::network_aux::road_spatial::entry> qr(netaux->road_space.query(r));
                            rb_add.set_candidates(qr);
                        }
                        break;
                    default:
                        break;
                    };
                }
                else if(Fl::event_button() == FL_RIGHT_MOUSE)
                {
                    switch(imode)
                    {
                    case ARC_MANIP:
                        {
                            view.pick_point(vec3f(world[0], world[1], 0.0f), view.point_size*5*scale);
                            drawing = true;
                            break;
                        }
                    case NONE:
                        break;
                    default:
                        break;
                    };
                }

                lastpick = world;
            }
            take_focus();
            return 1;
        case FL_RELEASE:
            {
                if(Fl::event_button() == FL_LEFT_MOUSE && drawing)
                {
                    switch(imode)
                    {
                    case REGION_MANIP:
                        break;
                    case ARC_MANIP:
                        drawing = false;
                        break;
                    default:
                        break;
                    }
                }
            }
            return 1;
        case FL_DRAG:
            {
                const vec2i xy(Fl::event_x(),
                               Fl::event_y());
                const vec2f world(world_point(vec2i(xy[0], h()-xy[1]), center, scale, vec2i(w(), h())));
                vec2f dvec(0);
                if(Fl::event_button() == FL_LEFT_MOUSE)
                {
                    switch(imode)
                    {
                    case REGION_MANIP:
                        drawing      = true;
                        second_point = world;
                        break;
                    case ARC_MANIP:
                        drawing = true;
                        view.update_active(vec3f(world[0], world[1], 0.0f));
                        break;
                    default:
                        break;
                    }
                }
                else if(Fl::event_button() == FL_MIDDLE_MOUSE)
                {
                    dvec = vec2f(world - lastpick);
                    center -= dvec;
                }
                else if(Fl::event_button() == FL_RIGHT_MOUSE)
                {
                    dvec = vec2f(world - lastpick);
                    switch(imode)
                    {
                    case BACK_MANIP:
                        back_image_center -= dvec;
                        break;
                    case ARC_MANIP:
                        drawing = true;
                        view.update_active(vec3f(world[0], world[1], 0.0f));
                        break;
                    default:
                        break;
                    }
                    dvec = 0;
                }
                lastpick = world-dvec;
            }
            take_focus();
            return 1;
        case FL_KEYBOARD:
            switch(Fl::event_key())
            {
            case '1':
                t -= 0.025;
                break;
            case '2':
                t += 0.025;
                break;
            case '0':
                bg_saturation = std::max(0.0f, bg_saturation - 0.05f);
                break;
            case '9':
                bg_saturation = std::min(1.0f, bg_saturation + 0.05f);
                break;
            case 'w':
                rb_add.param = std::max(rb_add.param-0.05f, 0.0f);
                break;
            case 'e':
                rb_add.param = std::min(rb_add.param+0.05f, 1.0f);
                break;
            case 'q':
                rb_add.next_candidate();
                break;
            case 'n':
                if(network_draw == NET_NONE)
                    network_draw = NET_ABSTRACT;
                else if(network_draw == NET_ABSTRACT)
                    network_draw = NET_TEXTURE;
                else if(network_draw == NET_TEXTURE)
                    network_draw = NET_NONE;
                break;
            case 'c':
                switch(imode)
                {
                case ARC_MANIP:
                    view.clear();
                    break;
                case NONE:
                    rb_add.clear();
                    break;
                default:
                    break;
                }
                break;
            case 'r':
                switch(imode)
                {
                case MC_PREVIEW:
                    t = 0.0f;
                    break;
                case NONE:
                    break;
                default:
                    break;
                }
                break;
            case 'k':
                switch(imode)
                {
                case MC_PREVIEW:
                    view.insert_scale_keyframe(path_param, scale);
                    break;
                default:
                    break;
                }
                break;
            case 's':
                screenshot_mode = !screenshot_mode;
                break;
            case '-':
                if(Fl::event_state() & FL_SHIFT)
                    sim_time_scale *= 0.5;
                else
                    sim_time_scale -= 0.5;
                if(sim_time_scale < 0.0)
                    sim_time_scale = 0;
                break;
            case '=':
                if(Fl::event_state() & FL_SHIFT)
                    sim_time_scale *= 2.0;
                else
                    sim_time_scale += 0.5;
                break;
            case 'm':
                switch(imode)
                {
                case REGION_MANIP:
                    imode = ARC_MANIP;
                    break;
                case ARC_MANIP:
                    imode = MC_PREVIEW;
                    break;
                case MC_PREVIEW:
                    imode = BACK_MANIP;
                    break;
                case BACK_MANIP:
                    imode = NONE;
                    break;
                case NONE:
                    imode = REGION_MANIP;
                    break;
                }
                break;
            case 't':
                throttle = !throttle;
                set_throttle();
                break;
            case ' ':
                go = !go;
                if(go)
                    std::cout << "Simulating" << std::endl;
                else
                    std::cout << "Simulating stopped" << std::endl;
                break;
            case 'i':
                draw_intersections = !draw_intersections;
                break;
            case 'b':
                if(back_image && !back_image->tiles.empty())
                {
                    std::cout << " scale: "  << back_image_scale  << std::endl
                              << " center: " << back_image_center << std::endl
			      << " y offset: " << back_image_yscale << std::endl;
                }
                else
                    std::cout << "No image" << std::endl;
                break;
            }
            return 1;
        case FL_MOUSEWHEEL:
            {
                const vec2i xy(Fl::event_x(),
                               Fl::event_y());
                const vec2i dxy(Fl::event_dx(),
                                Fl::event_dy());
                const float fy = copysign(0.5, dxy[1]);

                if(Fl::event_state() & FL_SHIFT)
                {
                    if(imode == BACK_MANIP)
                    {
                        if(Fl::event_state() & FL_CTRL)
                            back_image_yscale *= std::pow(2.0f, 0.01f*fy);
                        else
                            back_image_scale  *= std::pow(2.0f, 0.01f*fy);
                    }
                }
                else
                {
                    if(Fl::event_state() & FL_CTRL)
                        scale *= std::pow(2.0f, 0.01f*fy);
                    else
                        scale *= std::pow(2.0f, fy);
                }
            }
            take_focus();
            return 1;
        default:
            // pass other events to the base class...
            return Fl_Gl_Window::handle(event);
        }
    }

    vec2f lastpick;

    hwm::network     *net;
    hwm::network_aux *netaux;
    vec2f             tex_low;
    vec2f             tex_high;
    vec2f             center;
    float             scale;

    car_animation *anim;

    GLuint     glew_state;
    big_image *back_image;
    big_image *back_image_overlay;
    vec2i      back_image_dim;
    vec2f      back_image_center;
    float      back_image_scale;
    float      back_image_yscale;

    GLuint   overlay_tex_;
    GLuint   continuum_tex_;
    GLuint   arrow_tex_;
    float    arrow_aspect;

    GLuint          roadblock_tex_;
    GLuint          roadblock_flash_tex_;
    float           roadblock_aspect;
    roadblock_adder rb_add;

    std::vector<aabb2d>                                rectangles;
    bool                                               drawing;
    vec2f                                              first_point;
    vec2f                                              second_point;
    std::vector<hwm::network_aux::road_spatial::entry> query_results;

    std::vector<tex_car_draw*> car_drawers;
    float                      last_drawer_update;
    hwm::network_draw          network_drawer;
    hwm::network_aux_draw      network_aux_drawer;
    network_draw_mode          network_draw;
    bool                       draw_intersections;

    float               t;
    float               time_offset;
    float               sim_time_scale;
    timer               frame_timer;
    bool                go;
    float               avg_dt;
    float               avg_step_time;

    bool                                            screenshot_mode;
    int                                             screenshot_count;
    std::tr1::unordered_map<size_t, tex_car_draw*>  car_map;

    bool              do_lights;
    bool              do_cars;
    night_render      night_setup;
    float             bg_saturation;
    float             fg_saturation;
    view_path         view;
    float             path_param;
    float             path_param_rate;
    bool              path_auto_advance;
    interaction_mode  imode;
    bool              throttle;

    bool show_lengths;
};

static void draw_callback(void *v)
{
    reinterpret_cast<fltkview*>(v)->redraw();
    if(reinterpret_cast<fltkview*>(v)->throttle)
        Fl::repeat_timeout(FRAME_RATE, draw_callback, v);
}

#include "helper-window.cpp"

int main(int argc, char *argv[])
{
    std::cout << libroad_package_string() << std::endl;
    std::cout << libhybrid_package_string() << std::endl;
    if(argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <input network> <trajectory file> [background image] " << std::endl;
        return 1;
    }
    RESOURCE_ROOT = getenv(RESOURCE_ROOT_ENV_NAME);
    if(!RESOURCE_ROOT)
    {
        std::cerr << "Couldn't locate environment variable " << RESOURCE_ROOT_ENV_NAME << " for needed resources!" << std::endl;
        return 1;
    }
    if(!bf::exists(RESOURCE_ROOT))
    {
        std::cerr << "RESOURCE_ROOT path " << RESOURCE_ROOT << " doesn't exist!" << std::endl;
        return 1;
    }

    hwm::network net(hwm::load_xml_network(argv[1], vec3f(1.0, 1.0, 1.0f)));

    net.build_intersections();
    net.build_fictitious_lanes();
    net.auto_scale_memberships();
    std::cerr << "HWM net loaded successfully" << std::endl;

    try
    {
        net.check();
        std::cerr << "HWM net checks out" << std::endl;
    }
    catch(std::runtime_error &e)
    {
        std::cerr << "HWM net doesn't check out: " << e.what() << std::endl;
        exit(1);
    }

    hwm::network_aux neta(net);

    car_animation anim;
    load_trajectory_data(&anim, argv[2]);

    Fl_Double_Window *helper = make_window();
    helper->show(1, argv);

    sim_win->net    = &net;
    sim_win->netaux = &neta;
    sim_win->anim   = &anim;

    sim_win->time_offset = 0;

    if(argc == 5)
    {
    // scale: 1.9185
    //     center: [378.73, 64.5092]
    //     y offset: 0.993091
 // scale: 0.959254
 // center: [1306.83, -509.796]
 // y offset: 1.00346

 // scale: 0.952628
 // center: [1316.74, -486.947]
 // y offset: 1.02454

        sim_win->back_image = new big_image(argv[3]);
        sim_win->back_image_overlay = new big_image(argv[4]);

        sim_win->back_image_center = vec2f((float)-41.8057, (float)94.5195);
        sim_win->back_image_scale =  0.423366;
        sim_win->back_image_yscale = 1.10954;

        // sim_win->back_image_center = vec2f(-16.8949, -974.423);
        // sim_win->back_image_scale =  0.420447;
        // sim_win->back_image_yscale = 0.8179020;

    }

    Fl::add_timeout(FRAME_RATE, draw_callback, sim_win);

    vec3f low(FLT_MAX);
    vec3f high(-FLT_MAX);
    net.bounding_box(low, high);
    box_to_cscale(sim_win->center, sim_win->scale, sub<0,2>::vector(low), sub<0,2>::vector(high), vec2i(500,500));

    Fl::visual(FL_DOUBLE|FL_DEPTH|FL_MULTISAMPLE);

    helper->show(1, argv);

    return Fl::run();
}
