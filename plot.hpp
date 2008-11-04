#ifndef _PLOT_HPP_
#define _PLOT_HPP_

#include <cairo.h>

struct draw_metrics
{
    draw_metrics(int w, int h);
    void dim_update(int neww, int newh);

    void zoom(float fac, float dist);
    void zoom_yb(float fac, float dist);
    void translate(float fac, float x, float y);

    void query_point(const int pix[2], float val[2]) const;

    float base_extents_[4]; // left, right, bottom, top
    float center_[2];
    float solution_scale_;
    float aspect_scale_;

    int w_;
    int h_;
};

struct plot_tex
{
    bool prepare_cairo();

    void cairo_grid_ticks(int border_pixels);
    void cairo_overlay(int border_pixels);

    cairo_surface_t * csurface_;
    cairo_t * ccontext_;

    bool do_corners_;

    draw_metrics * dm_;
};
#endif
