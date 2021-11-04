#ifndef __GL_COMMON_HPP__
#define __GL_COMMON_HPP__

#include <GL/glew.h>
#include <GL/gl.h>

bool checkFramebufferStatus()
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

void printShaderInfoLog(GLuint obj)
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

void printProgramInfoLog(GLuint obj)
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

#endif
