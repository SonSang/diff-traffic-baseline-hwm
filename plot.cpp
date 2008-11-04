#include "plot.hpp"

#include <cmath>
#include <cstdio>
#include <algorithm>

inline static float ceil_div(float x, float div)
{
    return div*std::ceil(x/div);
}

typedef enum { LEFT, CENTER_X, RIGHT }  txt_align_x;
typedef enum { TOP, CENTER_Y, BOTTOM }  txt_align_y;

static void put_text(cairo_t * cr, const char * str, float x, float y, txt_align_x align_x, txt_align_y align_y)
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

draw_metrics::draw_metrics(int w, int h) : solution_scale_(1.0f),
                                           aspect_scale_(1.0f),
                                           w_(w),
                                           h_(w)
{
    center_[0] = 0.0f;
    center_[1] = 0.0f;

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
}

void draw_metrics::dim_update(int neww, int newh)

{
    float wproportion = (float)(neww-w_)/(float)w_;
    float hproportion = (float)(newh-h_)/(float)h_;

    float xdelta = wproportion*(base_extents_[1]-base_extents_[0]);
    float ydelta = hproportion*(base_extents_[3]-base_extents_[2]);

    base_extents_[0] -= xdelta*0.5f;
    base_extents_[1] += xdelta*0.5f;

    base_extents_[2] -= ydelta*0.5f;
    base_extents_[3] += ydelta*0.5f;

    w_ = neww;
    h_ = newh;
}

void draw_metrics::query_point(const int inscr[2], float val[2]) const
{
    //    from (0, w_) x (h_, 0) to (be[0], be[1]) x (be[2] x be[3])

    float screen[2];
    screen[0] =  inscr[0]/(float)w_*(base_extents_[1] - base_extents_[0]) + base_extents_[0];
    screen[1] = -inscr[1]/(float)h_*(base_extents_[3] - base_extents_[2]) - base_extents_[2];

    val[0] = (screen[0]/solution_scale_ - center_[0]);
    val[1] = (screen[1]/solution_scale_ - center_[1])/aspect_scale_;
}

bool plot_tex::prepare_cairo()
{
    if(ccontext_)
    {
        cairo_surface_destroy(csurface_);
        cairo_destroy(ccontext_);
    }

    csurface_ = cairo_image_surface_create(CAIRO_FORMAT_ARGB32,
                                           dm_->w_,
                                           dm_->h_);

    if(cairo_surface_status(csurface_) != CAIRO_STATUS_SUCCESS)
        return false;

    ccontext_ = cairo_create(csurface_);
    if(cairo_status (ccontext_) != CAIRO_STATUS_SUCCESS)
        return false;

    return true;
}

void plot_tex::cairo_grid_ticks()
{
    // x ticks
    {
        float left_soln[2];
        int left_screen[2] = { border_pixels_, 0};

        float right_soln[2];
        int right_screen[2] = { dm_->w_-border_pixels_, 0};

        dm_->query_point(left_screen, left_soln);
        dm_->query_point(right_screen, right_soln);

        float soln_width = right_soln[0]-left_soln[0];
        int npot = (int)std::floor(std::log(soln_width)/log(10.0));

        cairo_save(ccontext_);
        cairo_set_operator(ccontext_, CAIRO_OPERATOR_OVER);
        cairo_set_source_rgba(ccontext_, 0.0, 0.0, 0.0, 1.0);

        cairo_scale(ccontext_,  dm_->w_/(dm_->base_extents_[1] - dm_->base_extents_[0]), 1);

        cairo_translate(ccontext_,  -dm_->base_extents_[0], 0);

        cairo_scale(ccontext_, dm_->solution_scale_, 1.0);
        cairo_translate(ccontext_, dm_->center_[0], 0.0);

        char text[500];
        for(int j = 0; j < 3; ++j)
        {
            float power = std::pow(10.0, npot-j);
            for(float i = ceil_div(left_soln[0], power); i <= right_soln[0]; i+= power)
            {
                if(j == 0)
                {
                    snprintf(text, 500, "%g", i);
                    cairo_set_font_size (ccontext_, std::max(border_pixels_/4, 10));

                    cairo_select_font_face (ccontext_, "SANS",
                                            CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);

                    cairo_set_source_rgba (ccontext_, 0.0f, 0.0f, 0.0f, 1.0f);

                    double coords[2] = { i, border_pixels_ };
                    cairo_user_to_device(ccontext_, &(coords[0]), &(coords[1]));
                    cairo_save(ccontext_);
                    cairo_identity_matrix(ccontext_);
                    put_text(ccontext_, text, coords[0], coords[1]-border_pixels_/8, CENTER_X, BOTTOM);
                    put_text(ccontext_, text, coords[0], dm_->h_-border_pixels_+border_pixels_/8, CENTER_X, TOP);

                    cairo_restore(ccontext_);
                    cairo_move_to(ccontext_, i, border_pixels_);
                    cairo_rel_line_to(ccontext_, 0.0, dm_->h_-2*border_pixels_);

                    cairo_save(ccontext_);
                    cairo_set_source_rgba (ccontext_, 0.0f, 0.0f, 0.0f, 0.1f);
                    cairo_identity_matrix(ccontext_);
                    cairo_set_line_width(ccontext_, 0.5*std::pow(4.0, 1-j));
                    cairo_stroke(ccontext_);
                    cairo_restore(ccontext_);
                }

                cairo_move_to(ccontext_, i, border_pixels_);
                cairo_rel_line_to(ccontext_, 0, border_pixels_*0.25*std::pow(2.0, 1-j));

                cairo_move_to(ccontext_, i, dm_->h_-border_pixels_);
                cairo_rel_line_to(ccontext_, 0, -border_pixels_*0.25*std::pow(2.0, 1-j));

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
        int top_screen[2] = { 0, border_pixels_ };

        float bottom_soln[2];
        int bottom_screen[2] = { 0, dm_->h_-border_pixels_ };

        dm_->query_point(top_screen, top_soln);
        dm_->query_point(bottom_screen, bottom_soln);

        float soln_width = top_soln[1]-bottom_soln[1];
        int npot = (int)std::floor(std::log(soln_width)/log(10.0));

        cairo_save(ccontext_);
        cairo_set_operator(ccontext_, CAIRO_OPERATOR_OVER);
        cairo_set_source_rgba(ccontext_, 0.0, 0.0, 0.0, 1.0);

        cairo_scale(ccontext_,  1, dm_->h_/(dm_->base_extents_[3] - dm_->base_extents_[2]));

        cairo_translate(ccontext_,  0.0, -dm_->base_extents_[2]);

        cairo_scale(ccontext_, 1.0, -dm_->solution_scale_);
        cairo_translate(ccontext_, 0.0, dm_->center_[1]);
        cairo_scale(ccontext_, 1.0, dm_->aspect_scale_);

        char text[500];
        for(int j = 0; j < 2; ++j)
        {
            float power = std::pow(10.0, npot-j);
            for(float i = ceil_div(bottom_soln[1], power); i <= top_soln[1]; i+= power)
            {
                if(j == 0)
                {
                    snprintf(text, 500, "%g", i);
                    cairo_set_font_size (ccontext_, std::max(border_pixels_/4, 10));

                    cairo_select_font_face (ccontext_, "SANS",
                                            CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);

                    double coords[2] = { border_pixels_, i };
                    cairo_user_to_device(ccontext_, &(coords[0]), &(coords[1]));
                    cairo_save(ccontext_);
                    cairo_identity_matrix(ccontext_);
                    put_text(ccontext_, text, coords[0]-border_pixels_/8,     coords[1], RIGHT, CENTER_Y);
                    put_text(ccontext_, text, dm_->w_-coords[0]+border_pixels_/8, coords[1], LEFT,  CENTER_Y);

                    cairo_restore(ccontext_);

                    cairo_move_to(ccontext_, border_pixels_, i);
                    cairo_rel_line_to(ccontext_,  dm_->w_-2*border_pixels_, 0.0);

                    cairo_save(ccontext_);
                    cairo_set_source_rgba (ccontext_, 0.0f, 0.0f, 0.0f, 0.1f);
                    cairo_identity_matrix(ccontext_);
                    cairo_set_line_width(ccontext_, 0.5*std::pow(4.0, 1-j));
                    cairo_stroke(ccontext_);
                    cairo_restore(ccontext_);
                }

                cairo_move_to(ccontext_, border_pixels_, i);
                cairo_rel_line_to(ccontext_,  border_pixels_*0.25*std::pow(2.0, 1-j), 0.0);

                cairo_move_to(ccontext_, dm_->w_-border_pixels_, i);
                cairo_rel_line_to(ccontext_, -border_pixels_*0.25*std::pow(2.0, 1-j), 0.0);

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

void plot_tex::cairo_overlay()
{
    int width =  cairo_image_surface_get_width(csurface_);
    int height = cairo_image_surface_get_height(csurface_);

    if(width != dm_->w_ || height != dm_->h_)
        prepare_cairo();

    cairo_save(ccontext_);

    cairo_set_operator(ccontext_, CAIRO_OPERATOR_SOURCE);
    cairo_set_source_rgba (ccontext_, 0.3f, 0.3f, 0.3f, 1.0f);
    cairo_paint(ccontext_);

    cairo_rectangle(ccontext_, 0, 0,       dm_->w_, border_pixels_);
    cairo_rectangle(ccontext_, 0, dm_->h_-border_pixels_, dm_->w_, border_pixels_);

    cairo_rectangle(ccontext_, 0, border_pixels_, border_pixels_, dm_->h_-border_pixels_*2);
    cairo_rectangle(ccontext_, dm_->w_-border_pixels_, border_pixels_, border_pixels_, dm_->h_-border_pixels_*2);

    cairo_set_operator(ccontext_, CAIRO_OPERATOR_OVER);
    cairo_set_source_rgba (ccontext_, 1.0f, 1.0f, 1.0f, 0.9f);
    cairo_fill(ccontext_);

    char text[500];

    cairo_set_font_size(ccontext_, std::max(border_pixels_/4, 10));

    cairo_select_font_face(ccontext_, "SANS",
                            CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);

    cairo_set_source_rgba(ccontext_, 0.0f, 0.0f, 0.0f, 1.0f);

    if(do_corners_)
    {
        int inpos[2];
        float outpos[2];

        // NW corner
        inpos[0] = border_pixels_; inpos[1] = border_pixels_;
        dm_->query_point(inpos, outpos);

        snprintf(text, 500, "(%4.3f,%4.3f)", outpos[0], outpos[1]);
        put_text(ccontext_, text, inpos[0], inpos[1], RIGHT, BOTTOM);

        // NE corner
        inpos[0] = dm_->w_-border_pixels_; inpos[1] = border_pixels_;
        dm_->query_point(inpos, outpos);

        snprintf(text, 500, "(%4.3f,%4.3f)", outpos[0], outpos[1]);
        put_text(ccontext_, text, inpos[0], inpos[1], LEFT, BOTTOM);

        // SE corner
        inpos[0] = dm_->w_-border_pixels_; inpos[1] = dm_->h_-border_pixels_;
        dm_->query_point(inpos, outpos);

        snprintf(text, 500, "(%4.3f,%4.3f)", outpos[0], outpos[1]);
        put_text(ccontext_, text, inpos[0], inpos[1], LEFT, TOP);

        // SW corner
        inpos[0] = border_pixels_; inpos[1] = dm_->h_-border_pixels_;
        dm_->query_point(inpos, outpos);

        snprintf(text, 500, "(%4.3f,%4.3f)", outpos[0], outpos[1]);
        put_text(ccontext_, text, inpos[0], inpos[1], RIGHT, TOP);

    }
    cairo_grid_ticks();

    cairo_restore(ccontext_);
}
