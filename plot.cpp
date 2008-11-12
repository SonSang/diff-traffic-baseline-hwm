#include "plot.hpp"

#include <cmath>
#include <cstdio>
#include <algorithm>

static const float plotcolors[][4] = {{1.0, 0.0, 0.0, 1.0},
                                      {0.0, 1.0, 0.0, 1.0},
                                      {0.0, 0.0, 1.0, 1.0}};

static const int num_plotcolors = sizeof(plotcolors)/sizeof(plotcolors[0]);

inline static float ceil_div(float x, float div)
{
    return div*std::ceil(x/div);
}

typedef enum { LEFT, CENTER_X, RIGHT }  txt_align_x;
typedef enum { TOP, CENTER_Y, BOTTOM }  txt_align_y;

static void put_text(cairo_t *cr, const char *str, float x, float y, txt_align_x align_x, txt_align_y align_y)
{
    cairo_save (cr);

    cairo_font_extents_t fe;
    cairo_text_extents_t te;

    cairo_font_extents (cr, &fe);
    cairo_text_extents (cr, str, &te);

    switch(align_x)
    {
    case LEFT:
        x += -te.x_bearing;
        break;
    case CENTER_X:
        x += -te.width/2-te.x_bearing;
        break;
    case RIGHT:
    default:
        x += - (te.x_bearing + te.width);
        break;

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
    default:
        y +=  0.0;
        break;
    };

    cairo_move_to(cr, x, y);
    cairo_show_text(cr, str);
    cairo_restore(cr);
}

spatial_view::spatial_view() : solution_scale_(1.0f),
                               aspect_scale_(1.0f)
{
    center_[0] = 0.0f;
    center_[1] = 0.0f;
}

void spatial_view::zoom(float fac, float dist)
{
    solution_scale_ *= std::pow(fac, dist);
}

void spatial_view::zoom_yb(float fac, float dist)
{
    aspect_scale_ *= std::pow(fac, dist);
}

void spatial_view::translate(float fac, float x, float y)
{
    center_[0] += x*fac/solution_scale_;
    center_[1] -= y*fac/solution_scale_;
}

void spatial_view::query_point(const float screen[2], float val[2]) const
{
    val[0] = (screen[0]/solution_scale_ - center_[0]);
    val[1] = (screen[1]/solution_scale_ - center_[1])/aspect_scale_;
}

void cairo_plotter1d::draw(cairo_t *cxt, float l, float r, float linewidth, drawtype dt)
{
    size_t lo = std::max(std::floor((l - origin)/h), 0.0f);
    size_t hi = std::min(std::max(0.0f, std::ceil ((r - origin)/h)+1.0f), (float)ncells);

    float last;
    switch(dt)
    {
    case SMOOTH:
    default:
        cairo_move_to(cxt, lo*h+origin, data[lo*stride]);
        for(size_t i = lo+1; i < hi; ++i)
            cairo_line_to(cxt, i*h+origin, data[i*stride]);
        break;
    case STEPS_CONNECTED:
        cairo_move_to(cxt, lo*h+origin-h*0.5, data[lo*stride]);
        last = data[lo*stride];
        for(size_t i = lo+1; i < hi; ++i)
        {
            cairo_rel_line_to(cxt, h, 0);
            cairo_rel_line_to(cxt, 0, data[i*stride]-last);
            last = data[i*stride];
        }
        cairo_rel_line_to(cxt, h, 0);
        break;
    case STEPS_BROKEN:
    case STEPS_FULL:
        cairo_move_to(cxt, lo*h+origin-h*0.5, data[lo*stride]);
        last = data[lo*stride];
        for(size_t i = lo+1; i < hi; ++i)
        {
            cairo_rel_line_to(cxt, h, 0);
            cairo_rel_move_to(cxt, 0, data[i*stride]-last);
            last = data[i*stride];
        }
        cairo_rel_line_to(cxt, h, 0);

        if(dt == STEPS_BROKEN)
            break;

        cairo_move_to(cxt, lo*h+origin-h*0.5, 0);
        cairo_rel_line_to(cxt, 0, data[lo*stride]);
        for(size_t i = lo; i < hi-1; ++i)
        {
            float lowy  = std::min(0.0f, std::min(data[i*stride], data[(i+1)*stride]));
            float highy = std::max(0.0f, std::max(data[i*stride], data[(i+1)*stride]));

            cairo_move_to(cxt, i*h+origin+h*0.5, lowy);
            cairo_line_to(cxt, i*h+origin+h*0.5, highy);
        }
        cairo_move_to(cxt, (hi-1)*h+origin+h*0.5, 0);
        cairo_rel_line_to(cxt, 0, data[(hi-1)*stride]);
    };

    cairo_save(cxt);
    cairo_identity_matrix(cxt);
    cairo_set_line_width(cxt, 2.0);
    cairo_stroke(cxt);
    cairo_restore(cxt);
}

void plot_tex::reset(int w, int h)
{
    if(w > h)
    {
        float aspect = (float)w/(float)h;

        base_extents_[2] = -1.0f;
        base_extents_[3] =  1.0f;

        base_extents_[0] =  -aspect;
        base_extents_[1] =  aspect;
    }
    else
    {
        float aspect = (float)h/(float)w;

        base_extents_[0] = -1.0f;
        base_extents_[1] =  1.0f;

        base_extents_[2] =  -aspect;
        base_extents_[3] =  aspect;
    }
    prepare_cairo(w, h);
}

bool plot_tex::prepare_cairo(int w, int h)
{
    int ow, oh;
    if(ccontext_)
    {
        ow = cairo_image_surface_get_width(csurface_);
        oh = cairo_image_surface_get_height(csurface_);
        if(ow != w || oh != h)
        {
            float wproportion = (float)(w-ow)/(float)ow;
            float hproportion = (float)(h-oh)/(float)oh;

            float xdelta = wproportion*(base_extents_[1]-base_extents_[0]);
            float ydelta = hproportion*(base_extents_[3]-base_extents_[2]);

            base_extents_[0] -= xdelta*0.5f;
            base_extents_[1] += xdelta*0.5f;

            base_extents_[2] -= ydelta*0.5f;
            base_extents_[3] += ydelta*0.5f;
        }
        else
            return true;

        cairo_surface_destroy(csurface_);
        cairo_destroy(ccontext_);
    }

    csurface_ = cairo_image_surface_create(CAIRO_FORMAT_ARGB32,
                                           w,
                                           h);

    if(cairo_surface_status(csurface_) != CAIRO_STATUS_SUCCESS)
        return false;

    ccontext_ = cairo_create(csurface_);
    if(cairo_status (ccontext_) != CAIRO_STATUS_SUCCESS)
        return false;

    return true;
}

void plot_tex::cairo_grid_ticks(int border_pixels)
{
    int w = cairo_image_surface_get_width(csurface_);
    int h = cairo_image_surface_get_height(csurface_);

    // x ticks
    {
        float left_soln[2];
        int left_screen[2] = { border_pixels, 0};

        float right_soln[2];
        int right_screen[2] = { w-border_pixels, 0};

        query_point(left_screen, left_soln);
        query_point(right_screen, right_soln);

        float soln_width = right_soln[0]-left_soln[0];
        int npot = (int)std::floor(std::log(soln_width)/log(10.0));

        cairo_save(ccontext_);
        cairo_set_operator(ccontext_, CAIRO_OPERATOR_OVER);
        cairo_set_source_rgba(ccontext_, 0.0, 0.0, 0.0, 1.0);

        cairo_scale(ccontext_,  w/(base_extents_[1] - base_extents_[0]), 1);

        cairo_translate(ccontext_,  -base_extents_[0], 0);

        cairo_scale(ccontext_, sv_->solution_scale_, 1.0);
        cairo_translate(ccontext_, sv_->center_[0], 0.0);

        char text[500];
        for(int j = 0; j < 3; ++j)
        {
            float power = std::pow(10.0, npot-j);
            for(float i = ceil_div(left_soln[0], power); i <= right_soln[0]; i+= power)
            {
                if(j == 0)
                {
                    snprintf(text, 500, "%g", i);
                    cairo_set_font_size (ccontext_, std::max(border_pixels/4, 10));

                    cairo_select_font_face (ccontext_, "SANS",
                                            CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);

                    cairo_set_source_rgba (ccontext_, 0.0f, 0.0f, 0.0f, 1.0f);

                    double coords[2] = { i, border_pixels };
                    cairo_user_to_device(ccontext_, &(coords[0]), &(coords[1]));
                    cairo_save(ccontext_);
                    cairo_identity_matrix(ccontext_);
                    put_text(ccontext_, text, coords[0], coords[1]-border_pixels/8, CENTER_X, BOTTOM);
                    put_text(ccontext_, text, coords[0], h-border_pixels+border_pixels/8, CENTER_X, TOP);

                    cairo_restore(ccontext_);
                    cairo_move_to(ccontext_, i, border_pixels);
                    cairo_rel_line_to(ccontext_, 0.0, h-2*border_pixels);

                    cairo_save(ccontext_);
                    cairo_set_source_rgba (ccontext_, 0.0f, 0.0f, 0.0f, 0.1f);
                    cairo_identity_matrix(ccontext_);
                    cairo_set_line_width(ccontext_, 0.5*std::pow(4.0, 1-j));
                    cairo_stroke(ccontext_);
                    cairo_restore(ccontext_);
                }

                cairo_move_to(ccontext_, i, border_pixels);
                cairo_rel_line_to(ccontext_, 0, border_pixels*0.25*std::pow(2.0, 1-j));

                cairo_move_to(ccontext_, i, h-border_pixels);
                cairo_rel_line_to(ccontext_, 0, -border_pixels*0.25*std::pow(2.0, 1-j));

                cairo_save(ccontext_);
                cairo_identity_matrix(ccontext_);
                cairo_set_line_width(ccontext_, 0.5*std::pow(4.0, 1-j));
                cairo_stroke(ccontext_);
                cairo_restore(ccontext_);
            }
        }
    }

    cairo_restore(ccontext_);
    cairo_save(ccontext_);

    // y ticks
    {
        float top_soln[2];
        int top_screen[2] = { 0, border_pixels };

        float bottom_soln[2];
        int bottom_screen[2] = { 0, h-border_pixels };

        query_point(top_screen, top_soln);
        query_point(bottom_screen, bottom_soln);

        float soln_width = top_soln[1]-bottom_soln[1];
        int npot = (int)std::floor(std::log(soln_width)/log(10.0));

        cairo_save(ccontext_);
        cairo_set_operator(ccontext_, CAIRO_OPERATOR_OVER);
        cairo_set_source_rgba(ccontext_, 0.0, 0.0, 0.0, 1.0);

        cairo_scale(ccontext_,  1, h/(base_extents_[3] - base_extents_[2]));

        cairo_translate(ccontext_,  0.0, -base_extents_[2]);

        cairo_scale(ccontext_, 1.0, -sv_->solution_scale_);
        cairo_translate(ccontext_, 0.0, sv_->center_[1]);
        cairo_scale(ccontext_, 1.0, sv_->aspect_scale_);

        char text[500];
        for(int j = 0; j < 2; ++j)
        {
            float power = std::pow(10.0, npot-j);
            for(float i = ceil_div(bottom_soln[1], power); i <= top_soln[1]; i+= power)
            {
                if(j == 0)
                {
                    snprintf(text, 500, "%g", i);
                    cairo_set_font_size (ccontext_, std::max(border_pixels/4, 10));

                    cairo_select_font_face (ccontext_, "SANS",
                                            CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);

                    double coords[2] = { border_pixels, i };
                    cairo_user_to_device(ccontext_, &(coords[0]), &(coords[1]));
                    cairo_save(ccontext_);
                    cairo_identity_matrix(ccontext_);
                    put_text(ccontext_, text, coords[0]-border_pixels/8,     coords[1], RIGHT, CENTER_Y);
                    put_text(ccontext_, text, w-coords[0]+border_pixels/8, coords[1], LEFT,  CENTER_Y);

                    cairo_restore(ccontext_);

                    cairo_move_to(ccontext_, border_pixels, i);
                    cairo_rel_line_to(ccontext_,  w-2*border_pixels, 0.0);

                    cairo_save(ccontext_);
                    cairo_set_source_rgba (ccontext_, 0.0f, 0.0f, 0.0f, 0.1f);
                    cairo_identity_matrix(ccontext_);
                    cairo_set_line_width(ccontext_, 0.5*std::pow(4.0, 1-j));
                    cairo_stroke(ccontext_);
                    cairo_restore(ccontext_);
                }

                cairo_move_to(ccontext_, border_pixels, i);
                cairo_rel_line_to(ccontext_,  border_pixels*0.25*std::pow(2.0, 1-j), 0.0);

                cairo_move_to(ccontext_, w-border_pixels, i);
                cairo_rel_line_to(ccontext_, -border_pixels*0.25*std::pow(2.0, 1-j), 0.0);

                cairo_save(ccontext_);
                cairo_identity_matrix(ccontext_);
                cairo_set_line_width(ccontext_, 0.5*std::pow(4.0, 1-j));
                cairo_stroke(ccontext_);
                cairo_restore(ccontext_);
            }
        }
    }

    cairo_restore(ccontext_);
}

void plot_tex::cairo_overlay(int border_pixels)
{
    int w =  cairo_image_surface_get_width(csurface_);
    int h = cairo_image_surface_get_height(csurface_);

    cairo_save(ccontext_);

    cairo_set_operator(ccontext_, CAIRO_OPERATOR_SOURCE);
    cairo_set_source_rgba (ccontext_, 1.0f, 1.0f, 1.0f, 1.0f);
    cairo_paint(ccontext_);

    cairo_set_operator(ccontext_, CAIRO_OPERATOR_OVER);
    //    cairo_set_source_rgba(ccontext_, 0.0f, 0.0f, 0.0f, 1.0f);

    cairo_save(ccontext_);
    cairo_scale(ccontext_,  w/(base_extents_[1] - base_extents_[0]), -h/(base_extents_[3] - base_extents_[2]));
    cairo_translate(ccontext_,  -base_extents_[0], base_extents_[2]);
    cairo_scale(ccontext_, sv_->solution_scale_, sv_->solution_scale_);
    cairo_translate(ccontext_, sv_->center_[0], sv_->center_[1]);
    cairo_scale(ccontext_, 1.0, sv_->aspect_scale_);
    int px[2] = {0, 0};
    float opt[4];
    query_point(px, opt);
    px[0] = w;
    query_point(px, opt+2);
    for(size_t i = 0; i < plts_.size(); ++i)
    {
        cairo_set_source_rgba(ccontext_, plotcolors[i][0], plotcolors[i][1], plotcolors[i][2], plotcolors[i][3]);
        plts_[i]->draw(ccontext_, opt[0], opt[2], 2.0, plotter1d::STEPS_CONNECTED);
    }

    cairo_restore(ccontext_);

    cairo_rectangle(ccontext_, 0, 0,       w, border_pixels);
    cairo_rectangle(ccontext_, 0, h-border_pixels, w, border_pixels);

    cairo_rectangle(ccontext_, 0, border_pixels, border_pixels, h-border_pixels*2);
    cairo_rectangle(ccontext_, w-border_pixels, border_pixels, border_pixels, h-border_pixels*2);

    cairo_set_operator(ccontext_, CAIRO_OPERATOR_OVER);
    cairo_set_source_rgba (ccontext_, 0.9f, 0.9f, 0.9f, 0.9f);
    cairo_fill(ccontext_);

    char text[500];

    cairo_set_font_size(ccontext_, std::max(border_pixels/4, 10));

    cairo_select_font_face(ccontext_, "SANS",
                           CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);

    if(do_corners_)
    {
        int inpos[2];
        float outpos[2];

        // NW corner
        inpos[0] = border_pixels; inpos[1] = border_pixels;
        query_point(inpos, outpos);

        snprintf(text, 500, "(%4.3f,%4.3f)", outpos[0], outpos[1]);
        put_text(ccontext_, text, inpos[0], inpos[1], RIGHT, BOTTOM);

        // NE corner
        inpos[0] = w-border_pixels; inpos[1] = border_pixels;
        query_point(inpos, outpos);

        snprintf(text, 500, "(%4.3f,%4.3f)", outpos[0], outpos[1]);
        put_text(ccontext_, text, inpos[0], inpos[1], LEFT, BOTTOM);

        // SE corner
        inpos[0] = w-border_pixels; inpos[1] = h-border_pixels;
        query_point(inpos, outpos);

        snprintf(text, 500, "(%4.3f,%4.3f)", outpos[0], outpos[1]);
        put_text(ccontext_, text, inpos[0], inpos[1], LEFT, TOP);

        // SW corner
        inpos[0] = border_pixels; inpos[1] = h-border_pixels;
        query_point(inpos, outpos);

        snprintf(text, 500, "(%4.3f,%4.3f)", outpos[0], outpos[1]);
        put_text(ccontext_, text, inpos[0], inpos[1], RIGHT, TOP);

    }
    cairo_grid_ticks(border_pixels);

    cairo_restore(ccontext_);
}

void plot_tex::zoom(float fac, float dist)
{
    sv_->zoom(fac, dist);
}

void plot_tex::zoom_yb(float fac, float dist)
{
    sv_->zoom_yb(fac, dist);
}

void plot_tex::translate(float fac, float x, float y)
{
    sv_->translate(fac, x*(base_extents_[1] - base_extents_[0]), y*(base_extents_[3] - base_extents_[2]));
}

void plot_tex::query_point(const int pix[2], float val[2]) const
{
    //    from (0, w_) x (h_, 0) to (be[0], be[1]) x (be[2] x be[3])
    int w = cairo_image_surface_get_width(csurface_);
    int h = cairo_image_surface_get_height(csurface_);

    float screen[2];
    screen[0] =  pix[0]/(float)w*(base_extents_[1] - base_extents_[0]) + base_extents_[0];
    screen[1] = -pix[1]/(float)h*(base_extents_[3] - base_extents_[2]) - base_extents_[2];

    sv_->query_point(screen, val);
}
