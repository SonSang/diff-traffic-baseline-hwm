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
