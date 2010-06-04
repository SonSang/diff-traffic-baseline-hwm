#include "libroad/libroad_common.hpp"
#include <png.h>
#include <string>
#include <cstdlib>
#include <stdarg.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <cassert>
#include <iostream>

static void abort_(const char * s, ...)
{
	va_list args;
	va_start(args, s);
	vfprintf(stderr, s, args);
	fprintf(stderr, "\n");
	va_end(args);
	abort();
}

// writes an 8-bit RGB png from an BGRA buffer
static void write_png_file(float scale, const float *pix, const int w, const int h, const std::string &fname)
{
    png_byte color_type = PNG_COLOR_TYPE_RGB;
    png_byte bit_depth  = 8;

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

	png_set_IHDR(png_ptr, info_ptr, w, h,
		     bit_depth, color_type, PNG_INTERLACE_NONE,
		     PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);

	png_write_info(png_ptr, info_ptr);

	/* write bytes */
	if (setjmp(png_jmpbuf(png_ptr)))
		abort_("[write_png_file] Error during writing bytes");

    png_set_compression_level(png_ptr,
                              Z_BEST_COMPRESSION);

    png_bytep row = (png_bytep)malloc(sizeof(png_byte)*3*w);
    for(int i = h-1; i >= 0; --i)
    {
        float *row_pointer = (float*)pix+i*4*w;
        for(int p = 0; p < w; ++p)
        {
            row[3*p+0] = row_pointer[4*p+0]*scale;
            row[3*p+1] = row_pointer[4*p+1]*scale;
            row[3*p+2] = row_pointer[4*p+2]*scale;
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

struct header
{
    char magic[5];
    int  w;
    int  h;
    int  bpp;
    char format[5];
};

static void accumulate_dump(int *w, int *h, float **output, const std::string &in_file)
{
    int fin = open(in_file.c_str(), O_RDONLY);
    assert(fin != -1);
    struct stat file_stat;
    fstat(fin, &file_stat);

    void *newbuff = mmap(0, file_stat.st_size, PROT_READ, MAP_SHARED, fin, 0);
    assert(newbuff && newbuff != MAP_FAILED);
    close(fin);

    const header *head = reinterpret_cast<const header*>(newbuff);
    if(strncmp(head->magic, "DUMP", 5) != 0)
    {
        std::cerr << "Dump file " << in_file << " missing magic string DUMP" << std::endl;
        exit(1);
    }
    if(strncmp(head->format, "BGRA", 5) != 0)
    {
        std::cerr << "Dump file " << in_file << " apparently not BGRA" << std::endl;
        exit(1);
    }
    if(head->bpp != 1)
    {
        std::cerr << "Dump file " << in_file << " apparently 1 byte per channel" << std::endl;
        exit(1);
    }

    if(!*output)
    {
        *w = head->w;
        *h = head->h;
        *output = (float *)malloc(4*sizeof(float)* (*w)*(*h));
    }
    else if(head->w != *w || head->h != *h)
    {
        std::cerr << "Dimensions of "<< in_file << " not consistent with accumulation buffer" << std::endl;
        exit(1);
    }

    unsigned char *base = static_cast<unsigned char*>(newbuff) + sizeof(header);

    for(int j = 0; j < *h; ++j)
        for(int i = 0; i < *w; ++i)
        {
            int p = j*(*w) + i;
            (*output)[4*p+0] += base[4*p+2];
            (*output)[4*p+1] += base[4*p+1];
            (*output)[4*p+2] += base[4*p+0];
            (*output)[4*p+3] += base[4*p+3];
        }

    munmap(newbuff, file_stat.st_size);
}

int main(int argc, char *argv[])
{
    if(argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <output filename> <input dumpfile>* " << std::endl;
        return 1;
    }

    int w;
    int h;
    float *output = 0;
    for(int i = 2; i < argc; ++i)
        accumulate_dump(&w, &h, &output, argv[i]);

    write_png_file(1.0f/(argc-2), output, w, h, argv[1]);

    if(output)
        free(output);

    return 0;
}
