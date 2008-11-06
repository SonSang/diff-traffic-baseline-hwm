#ifndef _PLOT_HPP_
#define _PLOT_HPP_

#include <cairo.h>
#include <cstdlib>

struct spatial_view
{
    spatial_view();

    void zoom(float fac, float dist);
    void zoom_yb(float fac, float dist);
    void translate(float fac, float x, float y);

    void query_point(const float screen[2], float val[2]) const;

    float center_[2];
    float solution_scale_;
    float aspect_scale_;
};

struct plotter1d
{
    typedef enum {SMOOTH, STEPS_CONNECTED, STEPS_BROKEN, STEPS_FULL} drawtype;

    virtual void draw(cairo_t * cxt, float l, float r, float linewidth, drawtype d=SMOOTH) = 0;

    int stride;
    float * data;

    size_t ncells;
    float origin;
    float h;
};

struct cairo_plotter1d : public plotter1d
{
    virtual void draw(cairo_t *cxt, float l, float r, float linewidth, drawtype d=SMOOTH);
};

struct plot_tex
{
    void reset(int w, int h);

    bool prepare_cairo(int w, int h);

    void cairo_grid_ticks(int border_pixels);
    void cairo_overlay(int border_pixels);

    void zoom(float fac, float dist);
    void zoom_yb(float fac, float dist);
    void translate(float fac, float x, float y);

    void query_point(const int pix[2], float val[2]) const;

    cairo_surface_t *csurface_;
    cairo_t *ccontext_;

    bool do_corners_;

    float base_extents_[4]; // left, right, bottom, top

    spatial_view *sv_;
    plotter1d *plt_;
};
#endif
