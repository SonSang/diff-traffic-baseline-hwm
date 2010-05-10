#ifndef _BIG_IMAGE_TILE_HPP_
#define _BIG_IMAGE_TILE_HPP_

#include <Magick++.h>

struct tile
{
    void draw() const
    {
        glPushMatrix();
        glTranslatef(origin[0], origin[1], 0);
        glScalef(size[0], size[1], 1);
        glBindTexture(GL_TEXTURE_2D, tex);
        glBegin(GL_QUADS);
        glTexCoord2f(0.0, 0.0);
        glVertex2i(0, 0);
        glTexCoord2f(1.0, 0.0);
        glVertex2i(1, 0);
        glTexCoord2f(1.0, 1.0);
        glVertex2i(1, 1);
        glTexCoord2f(0.0, 1.0);
        glVertex2i(0, 1);
        glEnd();
        glPopMatrix();
    }

    vec2i  origin;
    vec2i  size;
    GLuint tex;
};

struct big_image
{
    big_image(const std::string &im_name) : im(im_name)
    {
        std::cout << boost::str(boost::format("Loaded image %s: %dx%d") % im_name % im.columns() % im.rows()) << std::endl;
    }

    vec2i dim() const
    {
        return vec2i(im.columns(), im.rows());
    }

    void draw() const
    {
        glEnable(GL_TEXTURE_2D);
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glPushMatrix();
        glScalef(1, -1, 1);
        BOOST_FOREACH(const tile &t, tiles)
        {
            t.draw();
        }
        glPopMatrix();
    }

    void make_tiles(int max_tile_size)
    {
        const vec2i dim(im.columns(), im.rows());

        vec2i ntiles(dim/max_tile_size);
        const vec2i rest(dim - ntiles*max_tile_size);
        if(rest[0] > 0)
            ++ntiles[0];
        if(rest[1] > 0)
            ++ntiles[1];

        unsigned char *pix = new unsigned char[max_tile_size*max_tile_size*4];
        tiles.reserve(ntiles[0]*ntiles[1]);
        for(int j = 0; j < ntiles[1]; ++j)
            for(int i = 0; i < ntiles[0]; ++i)
            {
                tile t;
                t.origin = vec2i(i*max_tile_size, j*max_tile_size);
                if(i == ntiles[0] - 1 && rest[0] > 0)
                    t.size[0] = rest[0];
                else
                    t.size[0] = max_tile_size;

                if(j == ntiles[1] - 1 && rest[1] > 0)
                    t.size[1] = rest[1];
                else
                    t.size[1] = max_tile_size;

                glGenTextures(1, &(t.tex));
                tiles.push_back(t);

                glBindTexture (GL_TEXTURE_2D, t.tex);
                glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
                glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

                im.write(t.origin[0], t.origin[1], t.size[0], t.size[1], "RGBA", Magick::CharPixel, pix);

                glTexImage2D (GL_TEXTURE_2D,
                              0,
                              GL_R3_G3_B2,//GL_RGB4,
                              t.size[0],
                              t.size[1],
                              0,
                              GL_RGBA,
                              GL_UNSIGNED_BYTE,
                              pix);

                std::cout << boost::str(boost::format("Finished tile %d/%d\r") % tiles.size() % (ntiles[0]*ntiles[1]));
                std::cout.flush();
             }

        delete[] pix;
    }

    std::vector<tile> tiles;
    Magick::Image     im;
};

#endif
