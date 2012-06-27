#ifndef __NIGHT_RENDER_HPP__
#define __NIGHT_RENDER_HPP__

#include "gl-common.hpp"

static const float HEADLIGHT_THRESHOLD          = 0.65*1.732;
static const float HEADLIGHT_COLOR[3]           = {0.5*255/255.0, 0.5*254/255.0, 0.5*149/255.0};
static const float TAILLIGHT_COLOR[3]           = {0.6*122/255.0, 0.6* 15/255.0, 0.6* 25/255.0};
static const float BRAKING_TAILLIGHT_COLOR[3]   = {2.0*122/255.0, 2.0* 15/255.0, 2.0* 25/255.0};

static const char  HEADLIGHT_TEX[]              = "small-headlight-pair.png";
static const char  TAILLIGHT_TEX[]              = "taillight.png";
static const char  AMBIENT_TEX[]                = "ambient-timeofday.png";

static const char *lshader       =
"#version 150                                                               \n"
"#extension GL_ARB_texture_multisample : enable                             \n"
"uniform sampler2DMS lum_tex;                                               \n"
"uniform sampler2DMS to_light_tex;                                          \n"
"uniform int       nsamples;                                                \n"
"uniform float     bg_saturation;                                           \n"
"uniform float     fg_saturation;                                           \n"
"uniform vec4      ambient_level;                                           \n"
"out vec4 FragColor;                                                        \n"
"vec4 desaturate(vec4 val, float sat)                                       \n"
"{                                                                          \n"
"    float lum = dot(val.rgb, vec3(1.0/3.0));                               \n"
"    vec4 lum_v = vec4(lum, lum, lum, val.a);                               \n"
"    return mix(lum_v, val, sat);                                           \n"
"}                                                                          \n"
"                                                                           \n"
"void main()                                                                \n"
"{                                                                          \n"
"    ivec2 screen_coord = ivec2(gl_FragCoord.xy);                           \n"
"    vec4 back = vec4(0.0);                                                 \n"
"    for(int i = 0; i < nsamples; ++i)                                      \n"
"        back += texelFetch(to_light_tex, screen_coord, i);                 \n"
"    back *= vec4(1.0/nsamples);                                            \n"
"    back  = desaturate(back, bg_saturation);                               \n"
"    vec4       lum  = texelFetch(lum_tex, screen_coord,0);                 \n"
"    vec4      light = desaturate(lum, fg_saturation)*(vec4(0.5)+back);     \n"
"    FragColor       = light + ambient_level*back;                          \n"
"}                                                                          \n";

struct night_render
{
    night_render() : lum_fb(0),
                     lum_tex(0),
                     to_light_fb(0),
                     to_light_tex(0),
                     depth_fb(0),
                     headlight_tex(0),
                     headlight_list(0),
                     taillight_tex(0),
                     taillight_list(0),
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

    void initialize(const char *resource_root, float front_bumper_offset, float rear_bumper_offset, const vec2i &dim)
    {
        {
            if(ambient_data)
                delete[] ambient_data;
            const std::string ambient_path((bf::path(resource_root) / AMBIENT_TEX).string());
            std::cout << "Looking for ambient texture in " << ambient_path << std::endl;
            Magick::Image am_im(ambient_path);
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

        const std::string headlight_path((bf::path(resource_root) / HEADLIGHT_TEX).string());
        std::cout << "Looking for headlight texture in " << headlight_path << std::endl;
        Magick::Image im(headlight_path);
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

        const std::string taillight_tex((bf::path(resource_root) / TAILLIGHT_TEX).string());
        std::cout << "Looking for taillight texture in " << headlight_path << std::endl;
        Magick::Image tim(taillight_tex);
        tim.sample(Magick::Geometry("40x40"));
        pix = new unsigned char[tim.columns()*tim.rows()*4];
        tim.write(0, 0, tim.columns(), tim.rows(), "RGBA", Magick::CharPixel, pix);
        gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGBA8, tim.columns(), tim.rows(),
                          GL_RGBA, GL_UNSIGNED_BYTE, pix);
        delete[] pix;
        taillight_aspect = tim.columns()/static_cast<float>(tim.rows());

        if(glIsRenderbuffer(depth_fb))
            glDeleteRenderbuffers(1, &depth_fb);
        glGenRenderbuffers(1, &depth_fb);

        glBindRenderbuffer(GL_RENDERBUFFER, depth_fb);

        glRenderbufferStorageMultisample(GL_RENDERBUFFER, nsamples, GL_DEPTH_COMPONENT32F, dim[0], dim[1]);
        glError();

        if(glIsFramebuffer(lum_fb))
            glDeleteFramebuffers(1, &lum_fb);
        glGenFramebuffers(1, &lum_fb);

        if(glIsTexture(lum_tex))
            glDeleteTextures(1, &lum_tex);
        glGenTextures(1, &lum_tex);

        glBindFramebuffer(GL_FRAMEBUFFER, lum_fb);
        glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, lum_tex);
        glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, nsamples, GL_RGBA8, dim[0], dim[1], false);
        glFramebufferTexture2D(GL_FRAMEBUFFER,
                               GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D_MULTISAMPLE, lum_tex, 0);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth_fb);
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
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth_fb);
        checkFramebufferStatus();
        glError();

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

        headlight_list = glGenLists(1);
        glNewList(headlight_list, GL_COMPILE);

        glPushMatrix();
        {
            glTranslatef(front_bumper_offset-1, 0, 0);
            draw_headlight();
        }
        glPopMatrix();
        glEndList();

        taillight_list = glGenLists(1);
        glNewList(taillight_list, GL_COMPILE);
        glTranslatef(rear_bumper_offset-0.1, 0, 0);
        draw_taillight();
        glEndList();
    }

    void draw_car_lights(float opacity=1.0, bool braking=false)
    {
        glColor3f(opacity*HEADLIGHT_COLOR[0], opacity*HEADLIGHT_COLOR[1], opacity*HEADLIGHT_COLOR[2]);
        glCallList(headlight_list);
        if(braking)
            glColor3f(opacity*BRAKING_TAILLIGHT_COLOR[0], opacity*BRAKING_TAILLIGHT_COLOR[1], opacity*BRAKING_TAILLIGHT_COLOR[2]);
        else
            glColor3f(opacity*TAILLIGHT_COLOR[0], opacity*TAILLIGHT_COLOR[1], opacity*TAILLIGHT_COLOR[2]);
        glCallList(taillight_list);
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

    void compose(const float t, const vec2f &lo, const vec2f &hi, float bg_saturation, float fg_saturation)
    {
        glClear(GL_COLOR_BUFFER_BIT);

        glUseProgram(lprogram);
        int lum_uniform_location = glGetUniformLocation(lprogram, "lum_tex");
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, lum_tex);
        glUniform1i(lum_uniform_location, 0);

        int to_light_uniform_location = glGetUniformLocation(lprogram, "to_light_tex");
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, to_light_tex);
        glUniform1i(to_light_uniform_location, 1);

        int bg_saturation_uniform_location = glGetUniformLocation(lprogram, "bg_saturation");
        glUniform1f(bg_saturation_uniform_location, bg_saturation);

        int fg_saturation_uniform_location = glGetUniformLocation(lprogram, "fg_saturation");
        glUniform1f(fg_saturation_uniform_location, fg_saturation);

        glBindFragDataLocation(lprogram, 0, "FragColor") ;

        int nsamples_uniform_location = glGetUniformLocation(lprogram, "nsamples");
        glUniform1i(nsamples_uniform_location, nsamples);

        int ambient_level_uniform_location = glGetUniformLocation(lprogram, "ambient_level");

        assert(ambient_data);
        vec4f ambient_color_vec;
        sub<0,3>::vector(ambient_color_vec) = ambient_color(t);
        ambient_color_vec[3]                = 1.0f;
        glUniform4fv(ambient_level_uniform_location, 1, ambient_color_vec.data());

        glRectfv(lo.data(),
                 hi.data());

        glUseProgram(0);
        glActiveTexture(GL_TEXTURE0);

        glError();
    }

    bool draw_lights(const float t) const
    {
        return length(ambient_color(t)) < HEADLIGHT_THRESHOLD;
    }

    GLuint         lum_fb;
    GLuint         lum_tex;
    GLuint         to_light_fb;
    GLuint         to_light_tex;
    GLuint         depth_fb;
    GLuint         lprogram;
    GLuint         headlight_tex;
    float          headlight_aspect;
    GLuint         headlight_list;
    GLuint         taillight_tex;
    float          taillight_aspect;
    GLuint         taillight_list;
    int            ambient_dim;
    unsigned char *ambient_data;
    int            nsamples;

    float  day_length;
    vec2f  sunrise_interval;
    vec2f  sunset_interval;
};

#endif
