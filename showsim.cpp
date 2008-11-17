#include <cstdio>
#include <cstdlib>
#include <FL/Fl.H>
#include <FL/Fl_Double_Window.H>
#include <FL/fl_draw.H>

#include <cmath>
#include "plotwin.hpp"

#include "arz.hpp"

#include <cstdlib>
#include <cstring>
#include <algorithm>

static const float gamma_c = 0.5f;
static const float inv_gamma = 1.0f/gamma_c;

static const float u_max = 1.0f;
static const float inv_u_max = 1.0f/u_max;

static const float del_h = 0.05f;

static const char * fieldnames[] = {"rho", "y"};

static const size_t ncells = 100;
static riemann_solution *rs;
static q *data;

static spatial_view *sv;
static plot_tex pt;

static float sim_step()
{
    full_q fq_buff[2];
    full_q * fq[2] = {fq_buff, fq_buff + 1};

    float maxspeed = 0.0f;

    memset(rs, 0, sizeof(riemann_solution));

    (*fq[0]) = full_q(data,
                      u_max, gamma_c);

    for(size_t i = 1; i < ncells; ++i)
    {
        (*fq[1]) = full_q(data + i,
                          u_max, gamma_c);

        riemann(rs + i,
                fq[0],
                fq[1],
                u_max,
                inv_u_max,
                gamma_c,
                inv_gamma);

        maxspeed = std::max(maxspeed, std::max(std::abs(rs[i].speeds[0]), std::abs(rs[i].speeds[1])));

        std::swap(fq[0], fq[1]);
    }

    memset(rs+ncells, 0, sizeof(riemann_solution));

    printf("maxspeed: %f\n", maxspeed);

    float dt = del_h/maxspeed;
    float coeff = dt/maxspeed;

    for(size_t i = 0; i < ncells; ++i)
    {
        data[i].rho -= coeff*(rs[i].fluct_r.rho + rs[i+1].fluct_l.rho);
        data[i].y   -= coeff*(rs[i].fluct_r.y   + rs[i+1].fluct_l.y);
    }

    return dt;
}

struct sim_window : public cairo_window
{
    sim_window(int x, int y, int w, int h, plot_tex * pt, const char *label = 0) :
        cairo_window(x, y, w, h, pt, label)
    {}

    int handle(int event)
    {
        switch(event)
        {
        case FL_KEYBOARD:
            {
                int key = Fl::event_key();
                switch(key)
                {
                case ' ':
                    sim_step();
                    redraw();
                    break;
                default:
                    break;
                }
            }
            break;
        default:
            break;
        };

        return cairo_window::handle(event);
    };
};

int main(int argc, char * argv[])
{
    data = (q*) malloc(ncells * sizeof (q));
    rs = (riemann_solution*) malloc((ncells+1) * sizeof (riemann_solution));

    float x = 0.0f;
    for(size_t i = 0; i < ncells; ++i)
    {
        data[i].rho = 0.5f;
        data[i].y   = to_y(data[i].rho, (i < 50) ? 0.2f : 0.1f, u_max, gamma_c);
        x += del_h;
    }

    float the_time = 0.0f;

    sv = new spatial_view();
    pt.reset(1000, 500);
    pt.sv_ = sv;
    pt.do_corners_ = true;

    {
        cairo_plotter1d *cp1d = new cairo_plotter1d;
        cp1d->stride = 2;
        cp1d->ncells = ncells;
        cp1d->data = &(data[0].rho);
        cp1d->h = del_h;
        cp1d->origin = 0.0;

        pt.plts_.push_back(cp1d);
    }
    {
        cairo_plotter1d *cp1d = new cairo_plotter1d;
        cp1d->stride = 2;
        cp1d->ncells = ncells;
        cp1d->data = &(data[0].y);
        cp1d->h = del_h;
        cp1d->origin = 0.0;

        pt.plts_.push_back(cp1d);
    }

    sim_window cw(0, 0, 1000, 500, &pt, "Tick test");

    cw.show();

    return Fl::run();
    free(rs);
    free(data);

};
