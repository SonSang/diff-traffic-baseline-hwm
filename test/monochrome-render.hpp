#ifndef __MONOCHROME_RENDER_HPP__
#define __MONOCHROME_RENDER_HPP__

#include "gl-common.hpp"

static const char *mono_shader =
"#version 150                                                               \n"
"#extension GL_ARB_texture_multisample : enable                             \n"
"uniform sampler2DMS mono_tex;                                              \n"
"uniform sampler2DMS front_tex;                                             \n"
"uniform float       saturation;                                            \n"
"uniform int         nsamples;                                              \n"
"out vec4 FragColor;                                                        \n"
"                                                                           \n"
"void main()                                                                \n"
"{                                                                          \n"
"    ivec2 screen_coord = ivec2(gl_FragCoord.xy);                           \n"
"    vec4 back = vec4(0.0);                                                 \n"
"    for(int i = 0; i < nsamples; ++i)                                      \n"
"        back += texelFetch(mono_tex, screen_coord, i);                     \n"
"    back *= vec4(1.0/nsamples);                                            \n"
"    float lum = dot(back.rgb, vec3(1.0/3.0));                              \n"
"    vec4 lum_v = vec4(lum, lum, lum, 1.0);                                 \n"
"    FragColor = mix(back, lum_v, saturation);                              \n"
"}                                                                          \n";

struct monochrome_render
{
    monochrome_render() : mono_fb(0),
                          mono_tex(0),
                          front_fb(0),
                          front_tex(0),
                          depth_fb(0),
                          nsamples(4)
    {}

    void initialize(const vec2i &dim)
    {
        if(glIsRenderbuffer(depth_fb))
            glDeleteRenderbuffers(1, &depth_fb);
        glGenRenderbuffers(1, &depth_fb);

        glBindRenderbuffer(GL_RENDERBUFFER, depth_fb);

        glRenderbufferStorageMultisample(GL_RENDERBUFFER, nsamples, GL_DEPTH_COMPONENT32F, dim[0], dim[1]);
        glError();

        if(glIsFramebuffer(mono_fb))
            glDeleteFramebuffers(1, &mono_fb);
        glGenFramebuffers(1, &mono_fb);

        if(glIsTexture(mono_tex))
            glDeleteTextures(1, &mono_tex);
        glGenTextures(1, &mono_tex);

        glBindFramebuffer(GL_FRAMEBUFFER, mono_fb);
        glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, mono_tex);
        glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, nsamples, GL_RGBA8, dim[0], dim[1], false);
        glFramebufferTexture2D(GL_FRAMEBUFFER,
                               GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D_MULTISAMPLE, mono_tex, 0);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth_fb);
        checkFramebufferStatus();

        if(glIsFramebuffer(front_fb))
            glDeleteFramebuffers(1, &front_fb);
        glGenFramebuffers(1, &front_fb);

        if(glIsTexture(front_tex))
            glDeleteTextures(1, &front_tex);
        glGenTextures(1, &front_tex);

        glBindFramebuffer(GL_FRAMEBUFFER, front_fb);
        glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, front_tex);
        glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, nsamples, GL_RGBA8, dim[0], dim[1], false);
        glFramebufferTexture2D(GL_FRAMEBUFFER,
                               GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D_MULTISAMPLE, front_tex, 0);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth_fb);
        checkFramebufferStatus();
        glError();

        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        glError();

        GLuint shader = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(shader, 1, &mono_shader, 0);
        glCompileShader(shader);

        printShaderInfoLog(shader);

        mono_program = glCreateProgram();
        glAttachShader(mono_program, shader);
        glLinkProgram(mono_program);
        glError();
    }

    void start_front()
    {
        glBindFramebuffer(GL_FRAMEBUFFER, front_fb);
    }

    void finish_front()
    {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    void start_mono()
    {
        glBindFramebuffer(GL_FRAMEBUFFER, mono_fb);
    }

    void finish_mono()
    {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    void compose(const float saturation, const vec2f &lo, const vec2f &hi)
    {
        glClear(GL_COLOR_BUFFER_BIT);

        glUseProgram(mono_program);
        int mono_uniform_location = glGetUniformLocation(mono_program, "mono_tex");
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, mono_tex);
        glUniform1i(mono_uniform_location, 0);

        int front_uniform_location = glGetUniformLocation(mono_program, "front_tex");
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, front_tex);
        glUniform1i(front_uniform_location, 1);

        int saturation_uniform_location = glGetUniformLocation(mono_program, "saturation");
        glUniform1f(saturation_uniform_location, saturation);

        glBindFragDataLocation(mono_program, 0, "FragColor") ;

        int nsamples_uniform_location = glGetUniformLocation(mono_program, "nsamples");
        glUniform1i(nsamples_uniform_location, nsamples);

        glRectfv(lo.data(),
                 hi.data());

        glUseProgram(0);
        glActiveTexture(GL_TEXTURE0);
        glError();
    }

    GLuint         mono_fb;
    GLuint         mono_tex;
    GLuint         front_fb;
    GLuint         front_tex;
    GLuint         depth_fb;
    GLuint         mono_program;
    int            nsamples;
};

#endif
