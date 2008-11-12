#include <cstdio>
#include <cstdlib>
#include <FL/Fl.H>
#include <FL/Fl_Double_Window.H>
#include <FL/fl_draw.H>

#include <cmath>
#include "plotwin.hpp"

spatial_view *sv;
plot_tex pt;

int main(int argc, char *argv[])
{
    sv = new spatial_view();
    pt.reset(500, 500);
    pt.sv_ = sv;
    pt.do_corners_ = true;

    {
        cairo_plotter1d *cp1d = new cairo_plotter1d;
        cp1d->stride = 1;
        cp1d->ncells = 1000;
        cp1d->data = (float*) malloc(sizeof(float) * cp1d->ncells);
        cp1d->h = 10.0/cp1d->ncells;
        cp1d->origin = -5.0f;

        float x = cp1d->origin;
        for(size_t i = 0; i < cp1d->ncells; ++i)
        {
            cp1d->data[i] = std::sin(10*x);
            x += cp1d->h;
        }

        pt.plts_.push_back(cp1d);
    }
    {
        cairo_plotter1d *cp1d = new cairo_plotter1d;
        cp1d->stride = 1;
        cp1d->ncells = 1000;
        cp1d->data = (float*) malloc(sizeof(float) * cp1d->ncells);
        cp1d->h = 10.0/cp1d->ncells;
        cp1d->origin = -5.0f;

        float x = cp1d->origin;
        for(size_t i = 0; i < cp1d->ncells; ++i)
        {
            cp1d->data[i] = std::atan(x);
            x += cp1d->h;
        }

        pt.plts_.push_back(cp1d);
    }

    cairo_window cw(0, 0, 500, 500, &pt, "Tick test");

    cw.show();

    return Fl::run();
};
