#include <cstdio>
#include <cstdlib>
#include <FL/Fl.H>
#include <FL/Fl_Double_Window.H>
#include <FL/fl_draw.H>

#include <cmath>
#include "plot.hpp"

draw_metrics * dm;
plot_tex pt;

struct cairo_window : public Fl_Double_Window
{
    cairo_window(int x, int y, int w, int h, const char * label = 0) :
        Fl_Double_Window(x, y, w, h, label)

    {
        Fl::visual(FL_RGB);
        Fl_Group::current()->resizable(this);
    }

    virtual void draw()
    {
        dm->dim_update(w(), h());
        pt.cairo_overlay();
        unsigned char * dat = cairo_image_surface_get_data(pt.csurface_);
        fl_draw_image(dat, 0, 0, w(), h(), 4, 0);
    }

    int handle(int event)
    {
        switch(event)
        {
        case FL_PUSH:
            {
                int x = Fl::event_x();
                int y = Fl::event_y();
                if(Fl::event_button() == FL_MIDDLE_MOUSE)
                {
                    lastmouse_[0] = x;
                    lastmouse_[1] = y;
                }
            }
            take_focus();
            return 1;
        case FL_DRAG:
            {
                int x = Fl::event_x();
                int y = Fl::event_y();

                if(Fl::event_button() == FL_MIDDLE_MOUSE)
                {
                    if(Fl::event_state() & FL_BUTTON3)
                    {
                        float fac = (Fl::event_state() & FL_CTRL) ? 1.05 : 1.5;
                        if(Fl::event_state() & FL_BUTTON1)
                            dm->aspect_scale_ *= pow(fac, -(y-lastmouse_[1])/10.0);
                        else
                            dm->solution_scale_ *= pow(fac, -(y-lastmouse_[1])/10.0);
                        lastmouse_[0] = x;
                        lastmouse_[1] = y;
                        redraw();
                    }
                    else
                    {
                        float fac = (Fl::event_state() & FL_CTRL) ? 0.1 : 1.0;
                        dm->center_[0] += fac*(dm->base_extents_[1] - dm->base_extents_[0])*(float)(x-lastmouse_[0])/((float) w()*dm->solution_scale_);
                        dm->center_[1] -= fac*(dm->base_extents_[3] - dm->base_extents_[2])*(float)(y-lastmouse_[1])/((float) h()*dm->solution_scale_);
                        lastmouse_[0] = x;
                        lastmouse_[1] = y;
                        redraw();
                    }
                }
            }
            take_focus();
            return 1;
        case FL_MOUSEWHEEL:
            {
                float fac = (Fl::event_state() & FL_CTRL) ? 1.05 : 1.5;
                if(Fl::event_state() & FL_BUTTON1)
                    dm->aspect_scale_ *= pow(fac, -Fl::event_dy());
                else
                    dm->solution_scale_ *= pow(fac, -Fl::event_dy());

                redraw();
            }
            take_focus();
            return 1;
        case FL_FOCUS:
            return 1;
        case FL_UNFOCUS:
            return 1;
        case FL_KEYBOARD:
            {
                int key = Fl::event_key();
                switch(key)
                {
//                 case '.':
//                     fltk::lock();
//                     border_pixels_++;
//                     redraw();
//                     fltk::unlock();
//                     break;
//                 case ',':
//                     fltk::lock();
//                     border_pixels_--;
//                     redraw();
//                     fltk::unlock();
//                     break;
                case FL_Escape:
                    exit(0);
                    break;
                default:
                    return 0;
                }
            }
        default:
            {
                // pass other events to the base class...
                int rval = Fl_Double_Window::handle(event);
                return rval;
            }
        }
    }

    int lastmouse_[2];
};

int main(int argc, char * argv[])
{
    cairo_window cw(0, 0, 500, 500, "Tick test");

    dm = new draw_metrics(500, 500);
    pt.dm_ = dm;
    pt.prepare_cairo();
    pt.border_pixels_ = 50;
    pt.do_corners_ = true;

    cw.show();

    return Fl::run();
};
