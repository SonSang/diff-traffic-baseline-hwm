#include <Magick++.h>
#include <GL/glew.h>
#include <FL/Fl.H>
#include <FL/Fl_Gl_Window.H>
#include <FL/Fl_Menu_Button.H>
#include <FL/gl.h>
#include <FL/glu.h>
#include <FL/glut.h>
#include <boost/regex.hpp>
#include <boost/thread.hpp>
#include "libroad/hwm_network.hpp"
#include "libroad/geometric.hpp"
#include "libhybrid/hybrid-sim.hpp"
#include "libroad/hwm_draw.hpp"
#include "libhybrid/timer.hpp"
#include "big-image-tile.hpp"
#include <png.h>

void abort_(const char * s, ...)
{
	va_list args;
	va_start(args, s);
	vfprintf(stderr, s, args);
	fprintf(stderr, "\n");
	va_end(args);
	abort();
}

// writes a RGB png from an BGRA buffer
void write_png_file(const unsigned char *pix, const vec2i &dim, const std::string &fname)
{
    png_byte color_type = PNG_COLOR_TYPE_RGB;
    png_byte bit_depth = 8;

    png_infop info_ptr;

	/* create file */
    FILE *fp = fopen(fname.c_str(), "wb");
	if (!fp)
		abort_("[write_png_file] File %s could not be opened for writing", fname.c_str());

	/* initialize stuff */
	png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);

	if (!png_ptr)
		abort_("[write_png_file] png_create_write_struct failed");

	info_ptr = png_create_info_struct(png_ptr);
	if (!info_ptr)
		abort_("[write_png_file] png_create_info_struct failed");

	if (setjmp(png_jmpbuf(png_ptr)))
		abort_("[write_png_file] Error during init_io");

	png_init_io(png_ptr, fp);

	/* write header */
	if (setjmp(png_jmpbuf(png_ptr)))
		abort_("[write_png_file] Error during writing header");

	png_set_IHDR(png_ptr, info_ptr, dim[0], dim[1],
		     bit_depth, color_type, PNG_INTERLACE_NONE,
		     PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);

	png_write_info(png_ptr, info_ptr);

	/* write bytes */
	if (setjmp(png_jmpbuf(png_ptr)))
		abort_("[write_png_file] Error during writing bytes");

    png_set_compression_level(png_ptr,
                              Z_NO_COMPRESSION);

    png_bytep row = (png_bytep)malloc(sizeof(png_byte)*3*dim[0]);
    for(int i = dim[1]-1; i >= 0; --i)
    {
        png_bytep row_pointer = (png_byte*)pix+i*4*dim[0];
        for(int p = 0; p < dim[0]; ++p)
        {
            row[3*p+0] = row_pointer[4*p+2];
            row[3*p+1] = row_pointer[4*p+1];
            row[3*p+2] = row_pointer[4*p+0];
        }
        png_write_row(png_ptr, row);
    }
    free(row);

	/* end write */
	if (setjmp(png_jmpbuf(png_ptr)))
		abort_("[write_png_file] Error during end of write");

	png_write_end(png_ptr, NULL);

    fclose(fp);
}

inline bool checkFramebufferStatus() {
    GLenum status;
    status = (GLenum) glCheckFramebufferStatus(GL_FRAMEBUFFER);
    switch(status) {
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

struct write_image
{
    write_image(const std::string &fname_, const vec2i &dim, unsigned char *pix) :
        fname(fname_), dimensions(dim), pixels(pix)
    {
    }

    void operator()()
    {
        write_png_file(pixels, dimensions, fname);
        std::cout << "Wrote " << fname << std::endl;
    }

    std::string    fname;
    vec2i          dimensions;
    unsigned char *pixels;
};

static const char *lshader       =
"uniform sampler2D lum_tex, to_light_tex;                                           \n"
"                                                                                   \n"
"void main()                                                                        \n"
"{                                                                                  \n"
"vec4               lum           = texture2D(lum_tex, gl_TexCoord[0].st);          \n"
"vec4               back          = texture2D(to_light_tex, gl_TexCoord[0].st);     \n"
"vec4               effective_lum = clamp(lum, 0.0, 0.4)/0.6;                       \n"
"gl_FragColor                     = clamp(effective_lum*back, 0.0, 0.9) + 0.3*back; \n"
"}                                                                                  \n";

#define HEADLIGHT_TEX "/home/sewall/Desktop/siga10/small-headlight.png"
#define TAILLIGHT_TEX "/home/sewall/Desktop/siga10/taillight.png"

struct night_render
{
    night_render() : lum_fb(0),
                     lum_tex(0),
                     to_light_fb(0),
                     to_light_tex(0),
                     headlight_tex(0),
                     taillight_tex(0)
                     {}

    void initialize(const vec2i &dim)
    {
        if(glIsTexture(headlight_tex))
            glDeleteTextures(1, &headlight_tex);
        glGenTextures(1, &headlight_tex);
        glBindTexture(GL_TEXTURE_2D, headlight_tex);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        Magick::Image im(HEADLIGHT_TEX);
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
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, dim[0], dim[1], 0,
                     GL_RGBA, GL_UNSIGNED_BYTE,NULL);
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
        glBindTexture(GL_TEXTURE_2D, to_light_tex);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, dim[0], dim[1], 0,
                     GL_RGBA, GL_UNSIGNED_BYTE,NULL);
        glFramebufferTexture2D(GL_FRAMEBUFFER,
                                  GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, to_light_tex, 0);
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
    }

    void draw_headlight()
    {
        glEnable(GL_TEXTURE_2D);
        glBindTexture (GL_TEXTURE_2D, headlight_tex);
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
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
        glDisable(GL_TEXTURE_2D);
    }

    void draw_taillight()
    {
        glEnable(GL_TEXTURE_2D);
        glBindTexture (GL_TEXTURE_2D, taillight_tex);
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
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
        glDisable(GL_TEXTURE_2D);
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

    void compose(const vec2f &lo, const vec2f &hi)
    {
        glClear(GL_COLOR_BUFFER_BIT);

        glEnable(GL_TEXTURE_2D);

        glUseProgram(lprogram);
        int lum_uniform_location = glGetUniformLocationARB(lprogram, "lum_tex");
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, lum_tex);
        glUniform1iARB(lum_uniform_location, 0);

        int to_light_uniform_location = glGetUniformLocationARB(lprogram, "to_light_tex");
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, to_light_tex);
        glUniform1iARB(to_light_uniform_location, 1);

        glBindTexture(GL_TEXTURE_2D, to_light_tex);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

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

        glUseProgram(0);
        glActiveTexture(GL_TEXTURE0);

        glError();

        glDisable(GL_TEXTURE_2D);
    }

    GLuint lum_fb;
    GLuint lum_tex;
    GLuint to_light_fb;
    GLuint to_light_tex;
    GLuint lprogram;
    GLuint headlight_tex;
    float  headlight_aspect;
    GLuint taillight_tex;
    float  taillight_aspect;
};

static const char *fshader =
"uniform sampler2D full_tex, body_tex;                                    \n"
"                                                                         \n"
"void main()                                                              \n"
"{                                                                        \n"
"vec4               color   = texture2D(full_tex, gl_TexCoord[0].st);     \n"
"vec4               mask    = texture2D(body_tex, gl_TexCoord[0].st);     \n"
"gl_FragColor               = gl_Color*mask.r*color + (1.0-mask.r)*color; \n"
"}                                                                        \n";

struct tex_car_draw
{
    tex_car_draw() : full_tex(0), body_tex(0)
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
    }

    void draw() const
    {
        glEnable(GL_TEXTURE_2D);

        glUseProgram(fprogram);
        int full_uniform_location = glGetUniformLocationARB(fprogram, "full_tex");
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, full_tex);
        glUniform1iARB(full_uniform_location, 0);

        int body_uniform_location = glGetUniformLocationARB(fprogram, "body_tex");
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, body_tex);
        glUniform1iARB(body_uniform_location, 1);

        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
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
};

struct car_draw_desc
{
    int           color_idx;
    tex_car_draw *drawer;
};

static const float CAR_LENGTH    = 4.5f;
//* This is the position of the car's axle from the FRONT bumper of the car
static const float CAR_REAR_AXLE = 3.5f;

class fltkview : public Fl_Gl_Window
{
public:
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
                                                          light_position(50.0, 100.0, 50.0, 1.0),
                                                          sim(0),
                                                          t(0),
                                                          sim_time_scale(1),
                                                          go(false),
                                                          screenshot_mode(0),
                                                          screenshot_buffer(0),
                                                          screenshot_count(0)
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
        if(screenshot_buffer)
            delete[] screenshot_buffer;
    }

    void setup_light()
    {
        static const GLfloat amb_light_rgba[]  = { 0.1, 0.1, 0.1, 1.0 };
        static const GLfloat diff_light_rgba[] = { 0.7, 0.7, 0.7, 1.0 };
        static const GLfloat spec_light_rgba[] = { 1.0, 1.0, 1.0, 1.0 };
        static const GLfloat spec_material[]   = { 1.0, 1.0, 1.0, 1.0 };
        static const GLfloat material[]        = { 1.0, 1.0, 1.0, 1.0 };
        static const GLfloat shininess         = 100.0;

        glEnable(GL_LIGHTING);
        glEnable(GL_LIGHT0);
        glEnable(GL_COLOR_MATERIAL);
        glPushMatrix();
        glLoadIdentity();
        glLightfv(GL_LIGHT0, GL_POSITION, light_position.data());
        glPopMatrix();
        glLightfv(GL_LIGHT0, GL_AMBIENT, amb_light_rgba );
        glLightfv(GL_LIGHT0, GL_DIFFUSE, diff_light_rgba );
        glLightfv(GL_LIGHT0, GL_SPECULAR, spec_light_rgba );
        glMaterialfv( GL_FRONT, GL_AMBIENT, material );
        glMaterialfv( GL_FRONT, GL_DIFFUSE, material );
        glMaterialfv( GL_FRONT, GL_SPECULAR, spec_material );
        glMaterialfv( GL_FRONT, GL_SHININESS, &shininess);
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
            glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

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
            glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

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
            put_text(cr, boost::str(boost::format("real time:     %8.3fs; scaling factor %8.3fx") % t % sim_time_scale), 10, 5, LEFT, TOP);
            if(go)
            {
                cairo_set_source_rgba (cr, 1.0f, 0.0f, 0.0f, 1.0f);
                put_text(cr, "simulating", 10, 25, LEFT, TOP);
                cairo_set_source_rgba (cr, 1.0f, 1.0f, 1.0f, 1.0f);
            }

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

    void draw()
    {
        float dt = 0;
        frame_timer.stop();
        if(go)
        {
            if(screenshot_mode)
                t += sim_time_scale*1.0/24.0;
            else
                t += sim_time_scale*frame_timer.interval_S();
        }
        frame_timer.reset();
        frame_timer.start();


        if(sim && hci)
        {
            while(t > hci->times[1])
            {
                hci->capture(*sim);
                dt = sim->hybrid_step();
                sim->advance_intersections(dt);
            }
        }

        if (!valid())
        {
            std::cout << "Window size is " << w() << " " << h() << std::endl;
            glViewport(0, 0, w(), h());
            glClearColor(0.0, 0.0, 0.0, 0.0);

            if(screenshot_buffer)
                delete[] screenshot_buffer;
            screenshot_buffer = new unsigned char[w()*h()*4];

            if(GLEW_OK != glew_state)
                init_glew();

            if(!network_drawer.initialized())
                network_drawer.initialize(sim->hnet, 0.01f);

            if(!network_aux_drawer.initialized())
                network_aux_drawer.initialize(netaux, 0.01f);

            init_textures();
            if(car_drawers.empty())
                init_car_drawers("/home/sewall/Desktop/siga10/");

            setup_light();
            glEnable(GL_BLEND);

            night_setup.initialize(vec2i(w(), h()));
            glEnable(GL_POLYGON_SMOOTH);
            glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
        }

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();

        vec2f lo, hi;
        cscale_to_box(lo, hi, center, scale, vec2i(w(), h()));
        glOrtho(lo[0], hi[0], lo[1], hi[1], -20, 20.0);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glDisable(GL_LIGHTING);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        night_setup.start_to_light();
        glClear(GL_COLOR_BUFFER_BIT);

        glEnable(GL_TEXTURE_2D);
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
        glColor3f(1.0, 1.0, 1.0);
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

        glBlendFunc(GL_SRC_ALPHA_SATURATE, GL_ONE);

        glDisable(GL_TEXTURE_2D);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        network_aux_drawer.draw_roads_wire();
        network_aux_drawer.draw_intersections_wire();

        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glEnable(GL_TEXTURE_2D);
        glBindTexture (GL_TEXTURE_2D, continuum_tex_);
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

        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        if(hci)
        {
            BOOST_FOREACH(hybrid::car_interp::car_hash::value_type &cs, hci->car_data[0])
            {
                if(!hci->in_second(cs.first))
                    continue;

                std::tr1::unordered_map<size_t, car_draw_desc>::iterator drawer(car_map.find(cs.first));
                if(drawer == car_map.end())
                {
                    car_draw_desc cdd;
                    cdd.color_idx = rand() % n_car_colors;
                    cdd.drawer = car_drawers[rand() % car_drawers.size()];
                    drawer            = car_map.insert(drawer, std::make_pair(cs.first, cdd));
                }

                glColor3fv(car_colors[drawer->second.color_idx]);
                mat4x4f trans(hci->point_frame(cs.first, t, sim->hnet->lane_width));
                mat4x4f ttrans(tvmet::trans(trans));
                glPushMatrix();
                glMultMatrixf(ttrans.data());
                drawer->second.drawer->draw();
                glPopMatrix();
            }
        }

        glDisable(GL_TEXTURE_2D);
        glFlush();
        night_setup.finish_to_light();
        glError();

        night_setup.start_lum();
        glClear(GL_COLOR_BUFFER_BIT);
        glColor3f(0.2*255/255.0, 0.2*254/255.0, 0.2*149/255.0);
        glBlendFunc(GL_ONE, GL_ONE);
        if(hci)
        {
            BOOST_FOREACH(hybrid::car_interp::car_hash::value_type &cs, hci->car_data[0])
            {
                if(!hci->in_second(cs.first))
                    continue;

                std::tr1::unordered_map<size_t, car_draw_desc>::iterator drawer(car_map.find(cs.first));
                assert(drawer != car_map.end());

                mat4x4f trans(hci->point_frame(cs.first, t, sim->hnet->lane_width));
                mat4x4f ttrans(tvmet::trans(trans));
                glPushMatrix();
                glMultMatrixf(ttrans.data());

                glPushMatrix();
                {
                    glTranslatef(sim->front_bumper_offset()-0.1, 0, 0);
                    glColor3f(0.4*255/255.0, 0.4*254/255.0, 0.4*149/255.0);
                    glPushMatrix();
                    {
                        glTranslatef(0,sim->hnet->lane_width*0.2, 0);
                        glRotatef(3.0, 0.0, 0.0, 1.0);
                        night_setup.draw_headlight();
                    }
                    glPopMatrix();
                    glPushMatrix();
                    {
                        glTranslatef(0,-sim->hnet->lane_width*0.2, 0);
                        glRotatef(-3.0, 0.0, 0.0, 1.0);
                        night_setup.draw_headlight();
                    }
                    glPopMatrix();
                }
                glPopMatrix();

                glColor3f(0.6*122/255.0, 0.6*15/255.0, 0.6*25/255.0);
                glTranslatef(sim->rear_bumper_offset()-0.1, 0, 0);
                night_setup.draw_taillight();

                glPopMatrix();
            }
        }
        glFlush();
        night_setup.finish_lum();
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        night_setup.compose(lo, hi);

        glColor3f(1.0, 1.0, 1.0);
        glEnable(GL_TEXTURE_2D);
        glBindTexture (GL_TEXTURE_2D, overlay_tex_);
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
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

        glFlush();
        glFinish();

        glError();

        if(screenshot_mode)
            screenshot();
    }

    void screenshot()
    {
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadBuffer(GL_BACK);
        glReadPixels(0, 0, w(), h(), GL_BGRA, GL_UNSIGNED_BYTE, screenshot_buffer);
        write_image wi(boost::str(boost::format("screenshot%05d.png")%screenshot_count++), vec2i(w(), h()), screenshot_buffer);
        wi();
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
                    if(drawing)
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
                sim_time_scale -= 0.5;
                if(sim_time_scale < 0.0)
                    sim_time_scale = 0;
                break;
            case '=':
                sim_time_scale += 0.5;
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
                    if(Fl::event_state() & FL_CTRL)
                        back_image_yscale *= std::pow(2.0f, 0.1f*fy);
                    else
                        back_image_scale  *= std::pow(2.0f, 0.1f*fy);
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

    vec4f               light_position;
    hybrid::simulator  *sim;
    hybrid::car_interp *hci;
    float               t;
    float               sim_time_scale;
    timer               frame_timer;
    bool                go;

    bool                                            screenshot_mode;
    unsigned char                                  *screenshot_buffer;
    int                                             screenshot_count;
    std::tr1::unordered_map<size_t, car_draw_desc>  car_map;

    night_render night_setup;
};

void draw_callback(void *v)
{
    reinterpret_cast<fltkview*>(v)->redraw();
    Fl::repeat_timeout(1.0/30.0, draw_callback, v);
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

        const int cars_per_lane = 2;
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

    hybrid::car_interp hci(s);
    s.hybrid_step();
    hci.capture(s);
    s.hybrid_step();

    fltkview mv(0, 0, 1280, 720, "fltk View");
    mv.net    = &net;
    mv.netaux = &neta;
    mv.sim    = &s;
    mv.hci    = &hci;
    mv.t      = hci.times[0];

    if(argc == 3)
      {
        mv.back_image = new big_image(argv[2]);
	mv.back_image_center = vec2f(-12.7838, -971.966);
	mv.back_image_scale = 0.420448;
	mv.back_image_yscale = 0.812253;
      }



    Fl::add_timeout(1.0/30.0, draw_callback, &mv);

    vec3f low(FLT_MAX);
    vec3f high(-FLT_MAX);
    net.bounding_box(low, high);
    box_to_cscale(mv.center, mv.scale, sub<0,2>::vector(low), sub<0,2>::vector(high), vec2i(500,500));

    mv.take_focus();
    Fl::visual(FL_DOUBLE|FL_DEPTH);

    mv.show(1, argv);
    return Fl::run();
}
