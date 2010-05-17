#include <Magick++.h>
#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glext.h>
#include <FL/Fl.H>
#include <FL/Fl_Gl_Window.H>
#include <FL/Fl_Menu_Button.H>
#include <boost/regex.hpp>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "libroad/hwm_network.hpp"
#include "libroad/geometric.hpp"
#include "libhybrid/hybrid-sim.hpp"
#include "libroad/hwm_draw.hpp"
#include "libhybrid/timer.hpp"
#include "big-image-tile.hpp"
#include <png.h>

#define FRAME_RATE (1.0/24.0)

static bool checkFramebufferStatus()
{
    const GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    switch(status)
    {
    case GL_FRAMEBUFFER_COMPLETE:
		return true;
    case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:
		printf("Framebuffer incomplete, incomplete attachment\n");
		return false;
    case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT:
		printf("Framebuffer incomplete, missing attachment\n");
		return false;
    case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER:
		printf("Framebuffer incomplete, missing draw buffer\n");
		return false;
    case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER:
		printf("Framebuffer incomplete, missing read buffer\n");
		return false;
    case GL_FRAMEBUFFER_UNSUPPORTED:
		printf("Unsupported framebuffer format\n");
		return false;
	}
    return false;
}

static void printShaderInfoLog(GLuint obj)
{
    int infologLength = 0;
    int charsWritten  = 0;
    char *infoLog;

    glGetShaderiv(obj, GL_INFO_LOG_LENGTH,&infologLength);

    if (infologLength > 1)
    {
        infoLog = (char *)alloca(infologLength);
        glGetShaderInfoLog(obj, infologLength, &charsWritten, infoLog);
        printf("%s\n",infoLog);
    }
}

static void printProgramInfoLog(GLuint obj)
{
    int infologLength = 0;
    int charsWritten  = 0;
    char *infoLog;

    glGetProgramiv(obj, GL_INFO_LOG_LENGTH,&infologLength);

    if (infologLength > 1)
    {
        infoLog = (char *)alloca(infologLength);
        glGetProgramInfoLog(obj, infologLength, &charsWritten, infoLog);
        printf("%s\n",infoLog);
    }
}

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

static const char *lshader       =
"#version 150                                                               \n"
"#extension GL_ARB_texture_multisample : enable                             \n"
"uniform sampler2D lum_tex;                                                 \n"
"uniform sampler2DMS to_light_tex;                                          \n"
"uniform int       nsamples;                                                \n"
"uniform vec4      light_level, ambient_level;                              \n"
"out vec4 FragColor;                                                        \n"
"                                                                           \n"
"void main()                                                                \n"
"{                                                                          \n"
"    ivec2 screen_coord = ivec2(gl_FragCoord.xy);                           \n"
"    vec4 back = vec4(0.0);                                                 \n"
"    for(int i = 0; i < nsamples; ++i)                                      \n"
"        back += texelFetch(to_light_tex, screen_coord, i);                 \n"
"    back *= vec4(1.0/nsamples);                                            \n"
"    vec4               lum  = texelFetch(lum_tex, screen_coord,0);         \n"
"    vec4      effective_lum = clamp(lum, 0.0, 0.4)/0.6;                    \n"
"    vec4              light = clamp(effective_lum*back, 0.0, 0.9);         \n"
"    FragColor               = light_level*light + ambient_level*back;      \n"
"}                                                                          \n";

#define RESOURCE_ROOT "/home/sewall/Dropbox/Shared/siga10/"

#define HEADLIGHT_TEX RESOURCE_ROOT "small-headlight-pair.png"
#define TAILLIGHT_TEX RESOURCE_ROOT "taillight.png"
#define AMBIENT_TEX   RESOURCE_ROOT "ambient-timeofday.png"

struct night_render
{
    night_render() : lum_fb(0),
                     lum_tex(0),
                     to_light_fb(0),
                     to_light_tex(0),
                     headlight_tex(0),
                     taillight_tex(0),
                     light_list(0),
                     ambient_dim(0),
                     ambient_data(0),
                     nsamples(4),
                     day_length(24*60*60),
                     sunrise_interval(60*60*vec2f(5.5, 7.5)),
                     sunset_interval(60*60*vec2f(18.0, 20.0))
                     {}

    ~night_render()
    {
        if(ambient_data)
            delete[] ambient_data;
    }

    void initialize(hybrid::simulator *sim, const vec2i &dim)
    {
        {
            if(ambient_data)
                delete[] ambient_data;
            Magick::Image am_im(AMBIENT_TEX);
            assert(am_im.rows() == 1);
            ambient_dim          = am_im.columns();
            ambient_data         = new unsigned char[ambient_dim*3];
            am_im.write(0, 0, ambient_dim, 1, "RGB", Magick::CharPixel, ambient_data);
        }

        if(glIsTexture(headlight_tex))
            glDeleteTextures(1, &headlight_tex);
        glGenTextures(1, &headlight_tex);
        glBindTexture(GL_TEXTURE_2D, headlight_tex);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        Magick::Image im(HEADLIGHT_TEX);
        im.sample(Magick::Geometry("40x40"));
        unsigned char *pix = new unsigned char[im.columns()*im.rows()*4];
        im.write(0, 0, im.columns(), im.rows(), "RGBA", Magick::CharPixel, pix);
        gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGBA8, im.columns(), im.rows(),
                          GL_RGBA, GL_UNSIGNED_BYTE, pix);
        delete[] pix;
        headlight_aspect = im.columns()/static_cast<float>(im.rows());

        if(glIsTexture(taillight_tex))
            glDeleteTextures(1, &taillight_tex);
        glGenTextures(1, &taillight_tex);
        glBindTexture(GL_TEXTURE_2D, taillight_tex);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        Magick::Image tim(TAILLIGHT_TEX);
        tim.sample(Magick::Geometry("40x40"));
        pix = new unsigned char[tim.columns()*tim.rows()*4];
        tim.write(0, 0, tim.columns(), tim.rows(), "RGBA", Magick::CharPixel, pix);
        gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGBA8, tim.columns(), tim.rows(),
                          GL_RGBA, GL_UNSIGNED_BYTE, pix);
        delete[] pix;
        taillight_aspect = tim.columns()/static_cast<float>(tim.rows());


        if(glIsFramebuffer(lum_fb))
            glDeleteFramebuffers(1, &lum_fb);
        glGenFramebuffers(1, &lum_fb);

        if(glIsTexture(lum_tex))
            glDeleteTextures(1, &lum_tex);
        glGenTextures(1, &lum_tex);

        glBindFramebuffer(GL_FRAMEBUFFER, lum_fb);
        glBindTexture(GL_TEXTURE_2D, lum_tex);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, dim[0], dim[1], 0, GL_RGBA, GL_UNSIGNED_BYTE, 0);
        glFramebufferTexture2D(GL_FRAMEBUFFER,
                               GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, lum_tex, 0);
        checkFramebufferStatus();

        if(glIsFramebuffer(to_light_fb))
            glDeleteFramebuffers(1, &to_light_fb);
        glGenFramebuffers(1, &to_light_fb);

        if(glIsTexture(to_light_tex))
            glDeleteTextures(1, &to_light_tex);
        glGenTextures(1, &to_light_tex);

        glBindFramebuffer(GL_FRAMEBUFFER, to_light_fb);
        glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, to_light_tex);
        glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, nsamples, GL_RGBA8, dim[0], dim[1], false);
        glFramebufferTexture2D(GL_FRAMEBUFFER,
                               GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D_MULTISAMPLE, to_light_tex, 0);
        glFramebufferTexture2D(GL_FRAMEBUFFER,
                               GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D_MULTISAMPLE, to_light_tex, 0);
        checkFramebufferStatus();

        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        glError();

        GLuint shader = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(shader, 1, &lshader, 0);
        glCompileShader(shader);

        printShaderInfoLog(shader);

        lprogram = glCreateProgram();
        glAttachShader(lprogram, shader);
        glLinkProgram(lprogram);
        glError();

        light_list = glGenLists(1);
        glNewList(light_list, GL_COMPILE);

        glPushMatrix();
        {
            glTranslatef(sim->front_bumper_offset()-1, 0, 0);
            glColor3f(0.4*255/255.0, 0.4*254/255.0, 0.4*149/255.0);
            draw_headlight();
        }
        glPopMatrix();

        glColor3f(0.6*122/255.0, 0.6*15/255.0, 0.6*25/255.0);
        glTranslatef(sim->rear_bumper_offset()-0.1, 0, 0);
        draw_taillight();
        glEndList();
    }

    void draw_car_lights()
    {
        glCallList(light_list);
    }

    void draw_headlight()
    {
        glBindTexture (GL_TEXTURE_2D, headlight_tex);
        glPushMatrix();
        glScalef(14.0/headlight_aspect, 14.0/headlight_aspect, 1);
        glTranslatef(0, -0.5, 0);
        glEnable(GL_TEXTURE_2D);
        glBegin(GL_QUADS);
        glTexCoord2f(0.0, 0.0);
        glVertex2f(0, 0);
        glTexCoord2f(1.0, 0.0);
        glVertex2f(headlight_aspect, 0);
        glTexCoord2f(1.0, 1.0);
        glVertex2f(headlight_aspect, 1);
        glTexCoord2f(0.0, 1.0);
        glVertex2f(0, 1);
        glEnd();
        glPopMatrix();
    }

    void draw_taillight()
    {
        glBindTexture (GL_TEXTURE_2D, taillight_tex);
        glPushMatrix();
        glScalef(5.0/taillight_aspect, 3.0/taillight_aspect, 1);
        glTranslatef(-taillight_aspect+0.05, -0.5, 0);
        glEnable(GL_TEXTURE_2D);
        glBegin(GL_QUADS);
        glTexCoord2f(0.0, 0.0);
        glVertex2f(0, 0);
        glTexCoord2f(1.0, 0.0);
        glVertex2f(taillight_aspect, 0);
        glTexCoord2f(1.0, 1.0);
        glVertex2f(taillight_aspect, 1);
        glTexCoord2f(0.0, 1.0);
        glVertex2f(0, 1);
        glEnd();
        glPopMatrix();
    }

    void start_to_light()
    {
        glBindFramebuffer(GL_FRAMEBUFFER, to_light_fb);
    }

    void finish_to_light()
    {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    void start_lum()
    {
        glBindFramebuffer(GL_FRAMEBUFFER, lum_fb);
    }

    void finish_lum()
    {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    vec3f ambient_color(float t) const
    {
        t                           = std::fmod(t, day_length);
        const float          lookup = t * ambient_dim/day_length;
        const unsigned char *base   = ambient_data + static_cast<int>(std::floor(lookup))*3;
        return vec3f(vec3f(base[0], base[1], base[2])/255.0);
    }

    void compose(const float t, const vec2f &lo, const vec2f &hi)
    {
        glClear(GL_COLOR_BUFFER_BIT);

        glUseProgram(lprogram);
        int lum_uniform_location = glGetUniformLocationARB(lprogram, "lum_tex");
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, lum_tex);
        glUniform1i(lum_uniform_location, 0);

        int to_light_uniform_location = glGetUniformLocationARB(lprogram, "to_light_tex");
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, to_light_tex);
        glUniform1i(to_light_uniform_location, 1);

        glBindFragDataLocation(lprogram, 0, "FragColor") ;

        int nsamples_uniform_location = glGetUniformLocation(lprogram, "nsamples");
        glUniform1i(nsamples_uniform_location, nsamples);

        vec2f lighting_factors(ambient_level(t));
        int light_level_uniform_location = glGetUniformLocation(lprogram, "light_level");
        vec4f lighting_color(lighting_factors[0]);
        lighting_color[3] = 1.0f;
        glUniform4fv(light_level_uniform_location, 1, lighting_color.data());

        int ambient_level_uniform_location = glGetUniformLocation(lprogram, "ambient_level");

        vec4f ambient_color_vec;
        if(ambient_data)
            sub<0,3>::vector(ambient_color_vec) = ambient_color(t);
        else
            sub<0,3>::vector(ambient_color_vec) = vec3f(lighting_factors[1]);
        ambient_color_vec[3] = 1.0f;
        glUniform4fv(ambient_level_uniform_location, 1, ambient_color_vec.data());

        glRectfv(lo.data(),
                 hi.data());

        glUseProgram(0);
        glActiveTexture(GL_TEXTURE0);

        glError();
    }

    bool draw_lights(const float t) const
    {
        vec3f ambient_color_vec;
        if(ambient_data)
            ambient_color_vec = ambient_color(t);
        else
        {
            vec2f lighting_factors(ambient_level(t));
            ambient_color_vec = vec3f(lighting_factors[1]);
        }

        return length(ambient_color_vec) < 0.65*1.732;
    }

    vec2f ambient_level(float t) const
    {
        static const float DARKNESS = 0.3;
        static const float DAYLIGHT = 1.0;

        static const float DIMMEST_LIGHT   = 0.0;
        static const float BRIGHTEST_LIGHT = 0.9;

        t = std::fmod(t, day_length);
        if(t < sunrise_interval[0] || t > sunset_interval[1])
            return vec2f(BRIGHTEST_LIGHT, DARKNESS);
        else if(t > sunrise_interval[0] && t < sunrise_interval[1])
        {
            const float scale = (t - sunrise_interval[0])/(sunrise_interval[1] - sunrise_interval[0]);
            return vec2f( (DIMMEST_LIGHT-BRIGHTEST_LIGHT)*scale + BRIGHTEST_LIGHT, (DAYLIGHT-DARKNESS)*scale + DARKNESS);
        }
        else if(t > sunset_interval[0] && t < sunset_interval[1])
        {
            const float scale = (t - sunset_interval[0])/(sunset_interval[1] - sunset_interval[0]);
            return vec2f( (BRIGHTEST_LIGHT-DIMMEST_LIGHT)*scale + DIMMEST_LIGHT, (DARKNESS-DAYLIGHT)*scale + DAYLIGHT);
        }
        else
            return vec2f(DIMMEST_LIGHT, DAYLIGHT);
    }

    GLuint         lum_fb;
    GLuint         lum_tex;
    GLuint         to_light_fb;
    GLuint         to_light_tex;
    GLuint         lprogram;
    GLuint         headlight_tex;
    float          headlight_aspect;
    GLuint         taillight_tex;
    GLuint         light_list;
    float          taillight_aspect;
    int            ambient_dim;
    unsigned char *ambient_data;
    int            nsamples;

    float  day_length;
    vec2f  sunrise_interval;
    vec2f  sunset_interval;
};

static const char *fshader =
"uniform sampler2D full_tex, body_tex;                                    \n"
"                                                                         \n"
"void main()                                                              \n"
"{                                                                        \n"
"vec4               color   = texture2D(full_tex, gl_TexCoord[0].st);     \n"
"float              mask    = texture2D(body_tex, gl_TexCoord[0].st).r;   \n"
"gl_FragColor               = mix(color, gl_Color*color, mask);           \n"
"}                                                                        \n";

struct car_draw_info
{
    car_draw_info()
    {}
    car_draw_info(int c) : color(c)
    {}

    int     color;
    mat4x4f frame;
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
        const vec2i dim(full_im.columns(), full_im.rows());
        if(dim[0] > dim[1])
            extents = vec2f(1.0, static_cast<float>(dim[1])/dim[0]);
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

        full_uniform_location = glGetUniformLocationARB(fprogram, "full_tex");
        body_uniform_location = glGetUniformLocationARB(fprogram, "body_tex");

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
        glUniform1iARB(full_uniform_location, 0);

        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, body_tex);
        glUniform1iARB(body_uniform_location, 1);
    }

    void draw_car_list() const
    {
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

    std::tr1::unordered_map<size_t, car_draw_info> members;
};

static const char *pointVertexShader =
    "void main()                                                            \n"
    "{                                                                      \n"
    "    gl_PointSize = 500*gl_Point.size;                                  \n"
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
    view_path(float psize) : program(0), texture(0), point_size(psize)
    {}

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

            delete data;
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
        BOOST_FOREACH(const vec3f &p, path.points_)
        {
            glVertex3fv(p.data());
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
        BOOST_FOREACH(const vec3f &p, path.points_)
        {
            glVertex3fv(p.data());
        }
        glEnd();

        glUseProgram(0);
    }

    arc_road path;

    GLuint program;
    GLuint texture;

    float point_size;
};

static const float CAR_LENGTH    = 4.5f;
//* This is the position of the car's axle from the FRONT bumper of the car
static const float CAR_REAR_AXLE = 3.5f;

static void draw_callback(void *v);

class fltkview : public Fl_Gl_Window
{
public:
    typedef enum {REGION_MANIP, ARC_MANIP, BACK_MANIP, NONE} interaction_mode;

    fltkview(int x, int y, int w, int h, const char *l) : Fl_Gl_Window(x, y, w, h, l),
                                                          lastpick(0),
                                                          net(0),
                                                          netaux(0),
                                                          glew_state(GLEW_OK+1),
                                                          back_image(0),
                                                          back_image_center(0),
                                                          back_image_scale(1),
                                                          back_image_yscale(1),
                                                          overlay_tex_(0),
                                                          continuum_tex_(0),
                                                          drawing(false),
                                                          sim(0),
                                                          t(0),
                                                          time_offset(0),
                                                          sim_time_scale(1),
                                                          go(false),
                                                          avg_dt(0),
                                                          avg_step_time(0),
                                                          screenshot_mode(0),
                                                          screenshot_count(0),
                                                          view(1.0f),
                                                          imode(NONE),
                                                          throttle(true)
    {
        this->resizable(this);
        frame_timer.reset();
        frame_timer.start();
        view.path.points_.push_back(vec3f(0.0, 0.0, 0.0));
        view.path.points_.push_back(vec3f(10.0, 10.0, 0.0));
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
            if(itr->path().has_filename() && bf::is_directory(itr->path())
               && bf::exists(itr->path() / "full.png"))
            {
                car_drawers.push_back(new tex_car_draw(2,
                                                       CAR_LENGTH,
                                                       1.5f,
                                                       CAR_REAR_AXLE,
                                                       itr->path().string()));
            }
        }
    }

    void init_textures()
    {
        GLint biggest_width = biggest_texture();
        std::cout << "Largest texture I support: " << biggest_width << std::endl;
        if(back_image && back_image->tiles.empty())
        {
            back_image->make_tiles(biggest_width/2, false);
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
            boost::posix_time::time_duration td(0,
                                                0,
                                                std::floor(std::fmod(t+time_offset, night_setup.day_length)),
                                                0);
            put_text(cr, boost::str(boost::format("real time:     %8.3fs") % t), 10, 5, LEFT, TOP);
            put_text(cr, boost::str(boost::format("time of day: %s") % boost::posix_time::to_simple_string(td)), 10, 30, LEFT, TOP);
            put_text(cr, boost::str(boost::format("scaling factor %8.3fx") % sim_time_scale), 270, 5, LEFT, TOP);
            if(throttle)
                put_text(cr, "throttle", w(), 5, RIGHT, TOP);
            if(go)
            {
                cairo_set_source_rgba (cr, 1.0f, 0.0f, 0.0f, 1.0f);
                put_text(cr, "simulating", 10, 50, LEFT, TOP);
                cairo_set_source_rgba (cr, 1.0f, 1.0f, 1.0f, 1.0f);
                put_text(cr, boost::str(boost::format("avg. dt %8.3fs; avg. step time %8.3fs") % avg_dt % avg_step_time), 10, 70, LEFT, TOP);
            }
            cairo_set_source_rgba (cr, 1.0f, 1.0f, 0.0f, 1.0f);
            switch(imode)
            {
            case ARC_MANIP:
                put_text(cr, "arc manip", w(), h(), RIGHT, BOTTOM);
                break;
            case BACK_MANIP:
                put_text(cr, "back manip", w(), h(), RIGHT, BOTTOM);
                break;
            case REGION_MANIP:
                put_text(cr, "region manip", w(), h(), RIGHT, BOTTOM);
                break;
            default:
                break;
            };
            cairo_set_source_rgba (cr, 1.0f, 1.0f, 1.0f, 1.0f);
        }

        cairo_identity_matrix(cr);

        cairo_set_line_cap(cr, CAIRO_LINE_CAP_ROUND);
        cairo_set_operator(cr, CAIRO_OPERATOR_SOURCE);

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
            BOOST_FOREACH(const aabb2d &r, rectangles)
            {
                cairo_set_matrix(cr, &cmat);
                cairo_rectangle(cr, r.bounds[0][0], r.bounds[0][1], r.bounds[1][0]-r.bounds[0][0], r.bounds[1][1]-r.bounds[0][1]);
                cairo_set_source_rgba(cr, 67/255.0, 127/255.0, 195/255.0, 0.2);
                cairo_fill_preserve(cr);
                cairo_set_source_rgba(cr, 17/255.0, 129/255.0, 255/255.0, 0.7);
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
                cairo_set_source_rgba(cr, 67/255.0, 127/255.0, 195/255.0, 0.2);
                cairo_fill_preserve(cr);
                cairo_set_source_rgba(cr, 17/255.0, 129/255.0, 255/255.0, 0.7);
                cairo_identity_matrix(cr);
                cairo_set_line_width(cr, 2.0);
                cairo_stroke(cr);
            }
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

    void update_drawers()
    {
        {
            std::vector<hybrid::car_interp::car_spatial> gone_cars;
            std::set_difference(hci->car_data[0].begin(), hci->car_data[0].end(),
                                hci->car_data[1].begin(), hci->car_data[1].end(),
                                std::inserter(gone_cars, gone_cars.end()),
                                hci->car_data[0].value_comp());

            BOOST_FOREACH(hybrid::car_interp::car_spatial &cs, gone_cars)
            {
                std::tr1::unordered_map<size_t, tex_car_draw*>::iterator to_remove = car_map.find(cs.c.id);
                assert(to_remove != car_map.end());
                to_remove->second->members.erase(cs.c.id);
                car_map.erase(to_remove);
            }
        }
        {
            std::vector<hybrid::car_interp::car_spatial> new_cars;
            std::set_difference(hci->car_data[1].begin(), hci->car_data[1].end(),
                                hci->car_data[0].begin(), hci->car_data[0].end(),
                                std::inserter(new_cars, new_cars.end()),
                                hci->car_data[0].value_comp());

            BOOST_FOREACH(hybrid::car_interp::car_spatial &cs, new_cars)
            {
                std::tr1::unordered_map<size_t, tex_car_draw*>::iterator drawer(car_map.find(cs.c.id));
                assert(drawer == car_map.end());
                tex_car_draw *draw_pick = car_drawers[rand() % car_drawers.size()];
                drawer                  = car_map.insert(drawer, std::make_pair(cs.c.id, draw_pick));
                draw_pick->members.insert(std::make_pair(cs.c.id, car_draw_info(rand() % n_car_colors)));
                drawer->second          = draw_pick;
            }
        }
    }

    void draw()
    {
        float dt = 0;
        frame_timer.stop();
        if(go)
        {
            if(screenshot_mode)
                t += sim_time_scale*FRAME_RATE;
            else
                t += sim_time_scale*frame_timer.interval_S();
        }
        frame_timer.reset();
        frame_timer.start();

        if(sim && hci)
        {
            timer step_timer;
            float dt_accum  = 0;
            int   num_steps = 0;
            step_timer.start();
            while(t > hci->times[1])
            {
                hci->capture(*sim);
                update_drawers();
                dt = sim->hybrid_step();
                sim->advance_intersections(dt);
                ++num_steps;
                dt_accum += dt;
            }
            step_timer.stop();
            if(num_steps > 0)
            {
                avg_dt = dt_accum / num_steps;
                avg_step_time = step_timer.interval_S()/num_steps;
            }
        }

        if (!valid())
        {
            std::cout << "Shader version is " << glGetString(GL_SHADING_LANGUAGE_VERSION) << std::endl;
            std::cout << "Window size is " << w() << " " << h() << std::endl;
            glViewport(0, 0, w(), h());
            glClearColor(0.0, 0.0, 0.0, 1.0);

            if(GLEW_OK != glew_state)
                init_glew();

            if(!network_drawer.initialized())
                network_drawer.initialize(sim->hnet, 0.01f);

            if(!network_aux_drawer.initialized())
                network_aux_drawer.initialize(netaux, 0.01f);

            init_textures();
            view.initialize();
            if(car_drawers.empty())
            {
                init_car_drawers(RESOURCE_ROOT);

                if(sim && hci)
                {
                    update_drawers();

                    sim->hybrid_step();
                    hci->capture(*sim);
                    update_drawers();
                    sim->hybrid_step();

                    t = hci->times[0];
                }
            }

            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

            night_setup.initialize(sim, vec2i(w(), h()));
            glEnable(GL_MULTISAMPLE);
            glEnable(GL_TEXTURE_2D);
            glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            glDisable(GL_LIGHTING);
        }

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();

        vec2f lo, hi;
        cscale_to_box(lo, hi, center, scale, vec2i(w(), h()));
        glOrtho(lo[0], hi[0], lo[1], hi[1], -20, 20.0);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        night_setup.start_to_light();
        glClear(GL_COLOR_BUFFER_BIT);

        glColor4f(1.0, 1.0, 1.0, 1.0);
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

        glDisable(GL_TEXTURE_2D);
        glColor3f(237.0/255, 234.0/255, 186.0/255);
        network_aux_drawer.draw_roads_solid();
        network_aux_drawer.draw_intersections_solid();

        const float line_width = 1000.0/scale;
        if(line_width > 1)
        {
            glLineWidth(line_width);
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            glColor3f(135.0/255, 103.0/255, 61.0/255);
            network_aux_drawer.draw_roads_wire();
            network_aux_drawer.draw_intersections_wire();
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        }
        glEnable(GL_TEXTURE_2D);

        glBindTexture (GL_TEXTURE_2D, continuum_tex_);
        glColor4f(1.0, 1.0, 1.0, 1.0);
        std::vector<vec4f> colors;
        BOOST_FOREACH(hybrid::lane &l, sim->lanes)
        {
            if(!l.parent->active || !l.is_macro())
                continue;

            colors.resize(l.N);
            for(size_t i = 0; i < l.N; ++i)
            {
                float val    = l.q[i].rho();
                blackbody(colors[i].data(), val);
                colors[i][3] = 1.0f;
            }

            glTexImage2D (GL_TEXTURE_2D,
                          0,
                          GL_RGBA,
                          l.N,
                          1,
                          0,
                          GL_RGBA,
                          GL_FLOAT,
                          colors[0].data());

            network_drawer.draw_lane_solid(l.parent->id);
        }

        if(hci)
        {
            const float car_draw_time = cog::clamp(t, hci->times[0], hci->times[1]);
            BOOST_FOREACH(tex_car_draw *drawer, car_drawers)
            {
                drawer->draw_start();
                typedef std::pair<const size_t, car_draw_info> id_car_draw_info;
                BOOST_FOREACH(id_car_draw_info &car, drawer->members)
                {
                    glColor3fv(car_colors[car.second.color]);
                    const mat4x4f trans(hci->point_frame(car.first, car_draw_time, sim->hnet->lane_width));
                    car.second.frame = tvmet::trans(trans);
                    glPushMatrix();
                    glMultMatrixf(car.second.frame.data());
                    drawer->draw_car_list();
                    glPopMatrix();
                }
                drawer->draw_end();
            }
        }

        night_setup.finish_to_light();
        glError();

        night_setup.start_lum();
        glClear(GL_COLOR_BUFFER_BIT);

        if(night_setup.draw_lights(t+time_offset))
        {
            glColor3f(0.2*255/255.0, 0.2*254/255.0, 0.2*149/255.0);

            glBlendFunc(GL_ONE, GL_ONE);
            if(hci)
            {
                BOOST_FOREACH(tex_car_draw *drawer, car_drawers)
                {
                    typedef std::pair<const size_t, car_draw_info> id_car_draw_info;
                    BOOST_FOREACH(const id_car_draw_info &car, drawer->members)
                    {
                        glColor3fv(car_colors[car.second.color]);
                        glPushMatrix();
                        glMultMatrixf(car.second.frame.data());
                        night_setup.draw_car_lights();
                        glPopMatrix();
                    }
                }
            }
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        }
        night_setup.finish_lum();

        night_setup.compose(t+time_offset, lo, hi);

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

    int handle(int event)
    {
        switch(event)
        {
        case FL_PUSH:
            {
                const vec2i xy(Fl::event_x(),
                               Fl::event_y());
                const vec2f world(world_point(vec2i(xy[0], h()-xy[1]), center, scale, vec2i(w(), h())));

                if(Fl::event_button() == FL_LEFT_MOUSE)
                    first_point = world;

                lastpick = world;
            }
            take_focus();
            return 1;
        case FL_RELEASE:
            {
                if(Fl::event_button() == FL_LEFT_MOUSE)
                {
                    if(imode == REGION_MANIP && drawing)
                    {
                        rectangles.clear();
                        rectangles.push_back(aabb2d());
                        rectangles.back().enclose_point(first_point[0], first_point[1]);
                        rectangles.back().enclose_point(second_point[0], second_point[1]);
                        drawing = false;
                        query_results = netaux->road_space.query(rectangles.back());

                        BOOST_FOREACH(hybrid::lane &l, sim->lanes)
                        {
                            l.updated_flag = false;
                        }
                        BOOST_FOREACH(hwm::network_aux::road_spatial::entry &e, query_results)
                        {
                            BOOST_FOREACH(hwm::network_aux::road_rev_map::lane_cont::value_type &lcv, *e.lc)
                            {
                                hwm::lane    &hwm_l = *(lcv.second.lane);
                                hybrid::lane &hyb_l = *(hwm_l.user_data<hybrid::lane>());
                                if(!hyb_l.updated_flag && hyb_l.active())
                                {
                                    hyb_l.convert_to_micro(*sim);
                                    hyb_l.updated_flag = true;
                                }
                            }
                        }
                        BOOST_FOREACH(hwm::intersection_pair &ip, sim->hnet->intersections)
                        {
                            BOOST_FOREACH(hwm::intersection::state &s, ip.second.states)
                            {
                                BOOST_FOREACH(hwm::lane_pair &lp, s.fict_lanes)
                                {
                                    hybrid::lane &hyb_l = *(lp.second.user_data<hybrid::lane>());
                                    if(!hyb_l.updated_flag && hyb_l.active())
                                    {
                                        hybrid::lane *up_l(hyb_l.upstream_lane());
                                        hybrid::lane *dn_l(hyb_l.downstream_lane());
                                        if((up_l && up_l->updated_flag && up_l->is_micro()) ||
                                           (dn_l && dn_l->updated_flag && dn_l->is_micro()))
                                        {
                                            hyb_l.convert_to_micro(*sim);
                                            hyb_l.updated_flag = true;
                                        }
                                    }
                                }
                            }
                        }

                        BOOST_FOREACH(hybrid::lane &l, sim->lanes)
                        {
                            if(!l.updated_flag && l.active())
                            {
                                l.convert_to_macro(*sim);
                                l.updated_flag = true;
                            }
                        }
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
                    if(imode == REGION_MANIP)
                        drawing      = true;
                    second_point = world;
                }
                else if(Fl::event_button() == FL_MIDDLE_MOUSE)
                {
                    dvec = vec2f(world - lastpick);
                    center -= dvec;
                }
                else if(Fl::event_button() == FL_RIGHT_MOUSE)
                {
                    dvec = vec2f(world - lastpick);
                    if(imode == BACK_MANIP)
                        back_image_center -= dvec;
                    dvec = 0;
                }
                lastpick = world-dvec;
            }
            take_focus();
            return 1;
        case FL_KEYBOARD:
            switch(Fl::event_key())
            {
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
                if(imode == REGION_MANIP)
                    imode = ARC_MANIP;
                else if(imode == ARC_MANIP)
                    imode = BACK_MANIP;
                else if(imode == BACK_MANIP)
                    imode = NONE;
                else if(imode == NONE)
                    imode = REGION_MANIP;
                break;
            case 't':
                throttle = !throttle;
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
                break;
            case ' ':
                go = !go;
                if(go)
                    std::cout << "Simulating" << std::endl;
                else
                    std::cout << "Simulating stopped" << std::endl;
                break;
            case 'i':
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
                            back_image_yscale *= std::pow(2.0f, 0.1f*fy);
                        else
                            back_image_scale  *= std::pow(2.0f, 0.1f*fy);
                    }
                }
                else
                {
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

    GLuint     glew_state;
    GLuint     background_tex_;
    big_image *back_image;
    vec2i      back_image_dim;
    vec2f      back_image_center;
    float      back_image_scale;
    float      back_image_yscale;

    GLuint   overlay_tex_;
    GLuint   continuum_tex_;

    std::vector<aabb2d>                                rectangles;
    bool                                               drawing;
    vec2f                                              first_point;
    vec2f                                              second_point;
    std::vector<hwm::network_aux::road_spatial::entry> query_results;

    std::vector<tex_car_draw*> car_drawers;
    hwm::network_draw          network_drawer;
    hwm::network_aux_draw      network_aux_drawer;

    hybrid::simulator  *sim;
    hybrid::car_interp *hci;
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

    night_render     night_setup;
    view_path        view;
    interaction_mode imode;
    bool             throttle;
};

static void draw_callback(void *v)
{
    reinterpret_cast<fltkview*>(v)->redraw();
    if(reinterpret_cast<fltkview*>(v)->throttle)
        Fl::repeat_timeout(FRAME_RATE, draw_callback, v);
}

int main(int argc, char *argv[])
{
    std::cout << libroad_package_string() << std::endl;
    if(argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <input network> [background image]" << std::endl;
        return 1;
    }
    hwm::network net(hwm::load_xml_network(argv[1], vec3f(1.0, 1.0, 1.0f)));

    net.build_intersections();
    net.build_fictitious_lanes();
    net.auto_scale_memberships();
    net.center();
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

    hybrid::simulator s(&net,
                        4.5,
                        1.0);
    s.micro_initialize(0.73,
                       1.67,
                       33,
                       4);
    s.macro_initialize(0.5, 2.1*4.5, 0.0f);

    BOOST_FOREACH(hybrid::lane &l, s.lanes)
    {
        l.current_cars().clear();
        l.sim_type = hybrid::MICRO;

        const int cars_per_lane = 200;
        double    p             = -s.rear_bumper_offset()*l.inv_length;

        for (int i = 0; i < cars_per_lane; i++)
        {
            //TODO Just creating some cars here...
            l.current_cars().push_back(s.make_car(p, 0, 0));

            //Cars need a minimal distance spacing
            p += (30.0 * l.inv_length);
            if(p + s.front_bumper_offset()*l.inv_length >= 1.0)
                break;
        }
    }

    //    s.settle(0.033);

    fltkview mv(0, 0, 1280, 720, "fltk View");
    mv.net    = &net;
    mv.netaux = &neta;
    mv.sim    = &s;

    hybrid::car_interp hci(s);
    mv.hci    = &hci;

    mv.time_offset = 0;

    if(argc == 3)
    {
        mv.back_image = new big_image(argv[2]);
        mv.back_image_center = vec2f(-11.7303, -876.804);
        mv.back_image_scale =  0.378929;
        mv.back_image_yscale = 0.812253;
    }

    Fl::add_timeout(FRAME_RATE, draw_callback, &mv);

    vec3f low(FLT_MAX);
    vec3f high(-FLT_MAX);
    net.bounding_box(low, high);
    box_to_cscale(mv.center, mv.scale, sub<0,2>::vector(low), sub<0,2>::vector(high), vec2i(500,500));

    mv.take_focus();
    Fl::visual(FL_DOUBLE|FL_DEPTH|FL_MULTISAMPLE);

    mv.show(1, argv);
    return Fl::run();
}
