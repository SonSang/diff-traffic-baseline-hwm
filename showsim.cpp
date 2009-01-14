#include <cstdio>
#include <cstdlib>
#include <FL/Fl.H>
#include <FL/Fl_Double_Window.H>
#include <FL/fl_draw.H>

#include "boost/foreach.hpp"

#define foreach BOOST_FOREACH

#include <cmath>
#include <cfloat>
#include "plotwin.hpp"

#include "arz.hpp"

#include <cstdlib>
#include <cstring>
#include <algorithm>

static const float gamma_c = 0.5f;
static const float inv_gamma = 1.0f/gamma_c;

struct lane
{
    float u_max;
    float h;
    size_t ncells;

    riemann_solution *rs;
    q *data;
};

static const char * fieldnames[] = {"rho", "y"};

static spatial_view *sv;
static plot_tex pt;

std::vector<lane> lanes;

static float sim_step()
{
    full_q fq_buff[2];
    full_q * fq[2] = {fq_buff, fq_buff + 1};

    float maxspeed = 0.0f;
    float min_h = FLT_MAX;
    size_t count = 0;
    foreach(const lane &la, lanes)
    {
        std::swap(fq[0], fq[1]);
        fq[0]->from_q(la.data,
                      la.u_max, gamma_c);

        if(count == 0)
        {
            //memset(la.rs, 0, sizeof(riemann_solution));
            starvation_riemann(la.rs,
                               fq[0],
                               la.u_max,
                               1.0f/la.u_max,
                               gamma_c,
                               1.0f/gamma_c);
            maxspeed = std::max(maxspeed, std::max(std::abs(la.rs[0].speeds[0]), std::abs(la.rs[0].speeds[1])));
        }
        else
        {
            lebacque_inhomogeneous_riemann(la.rs,
                                           fq[1],
                                           fq[0],
                                           lanes[count-1].u_max,
                                           lanes[count].u_max,
                                           gamma_c,
                                           1.0f/gamma_c);
            maxspeed = std::max(maxspeed, std::max(std::abs(la.rs[0].speeds[0]), std::abs(la.rs[0].speeds[1])));
            assert(std::isfinite(la.rs[0].speeds[0]));
            assert(std::isfinite(la.rs[0].speeds[1]));

            memcpy(lanes[count-1].rs + lanes[count-1].ncells, la.rs, sizeof(riemann_solution));
        }

        min_h = std::min(min_h, la.h);

        for(size_t i = 1; i < la.ncells; ++i)
        {
            fq[1]->from_q(la.data + i,
                          la.u_max, gamma_c);

            riemann(la.rs + i,
                    fq[0],
                    fq[1],
                    la.u_max,
                    1.0f/la.u_max,
                    gamma_c,
                    inv_gamma);

            assert(std::isfinite(la.rs[i].speeds[0]));
            assert(std::isfinite(la.rs[i].speeds[1]));

            maxspeed = std::max(maxspeed, std::max(std::abs(la.rs[i].speeds[0]), std::abs(la.rs[i].speeds[1])));
            std::swap(fq[0], fq[1]);
        }

        memset(la.rs+la.ncells, 0, sizeof(riemann_solution));

        ++count;
    }

    printf("maxspeed: %f\n", maxspeed);

    float dt = min_h/maxspeed;

    foreach(lane &la, lanes)
    {
        float coeff = dt/la.h;

        q limited_base[2];
        q *limited[2] = {limited_base, limited_base+1};

        (*limited[1]) = flux_correction(la.rs,
                                        la.rs + 1,
                                        la.rs + 2,
                                        coeff);
        size_t i = 0;

        la.data[i].rho -= coeff*(la.rs[i].fluct_r.rho + la.rs[i+1].fluct_l.rho + (*limited[1]).rho - (*limited[0]).rho);
        la.data[i].y   -= coeff*(la.rs[i].fluct_r.y   + la.rs[i+1].fluct_l.y   + (*limited[1]).y   - (*limited[0]).y);

        assert(std::isfinite(la.data[i].rho) && std::isfinite(la.data[i].y));

        if(la.data[i].rho < FLT_EPSILON)
            la.data[i].rho  = FLT_EPSILON;
        if(la.data[i].y > FLT_EPSILON)
            la.data[i].y = FLT_EPSILON;

        std::swap(limited[0], limited[1]);

        for(i = 1; i < la.ncells-1; ++i)
        {
            (*limited[1]) = flux_correction(la.rs + i,
                                            la.rs + i + 1,
                                            la.rs + i + 2,
                                            coeff);

            la.data[i].rho -= coeff*(la.rs[i].fluct_r.rho + la.rs[i+1].fluct_l.rho + (*limited[1]).rho - (*limited[0]).rho);
            la.data[i].y   -= coeff*(la.rs[i].fluct_r.y   + la.rs[i+1].fluct_l.y   + (*limited[1]).y   - (*limited[0]).y);

            assert(std::isfinite(la.data[i].rho) && std::isfinite(la.data[i].y));

            if(la.data[i].rho < FLT_EPSILON)
                la.data[i].rho  = FLT_EPSILON;
            if(la.data[i].y > FLT_EPSILON)
                la.data[i].y = FLT_EPSILON;

            std::swap(limited[0], limited[1]);
        }

        memset(limited[1], 0, sizeof(q));

        la.data[i].rho -= coeff*(la.rs[i].fluct_r.rho + la.rs[i+1].fluct_l.rho + (*limited[1]).rho - (*limited[0]).rho);
        la.data[i].y   -= coeff*(la.rs[i].fluct_r.y   + la.rs[i+1].fluct_l.y   + (*limited[1]).y   - (*limited[0]).y);

        assert(std::isfinite(la.data[i].rho) && std::isfinite(la.data[i].y));

        if(la.data[i].rho < FLT_EPSILON)
            la.data[i].rho  = FLT_EPSILON;
        if(la.data[i].y > FLT_EPSILON)
            la.data[i].y = FLT_EPSILON;
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

    {
        lanes.push_back(lane());

        lane &la = lanes.back();

        la.ncells = 100;
        la.data = (q*) malloc(la.ncells * sizeof (q));
        la.rs = (riemann_solution*) malloc((la.ncells+1) * sizeof (riemann_solution));
        la.h     = 1000.0f/(la.ncells-1);
        la.u_max = 60.0f/3.6f;

        float x = 0.0f;
        for(size_t i = 0; i < la.ncells; ++i)
        {
            la.data[i].rho = (i < 50) ? 0.4f : 0.4f;
            //            la.data[i].y   = to_y(la.data[i].rho, (i < 50) ? 0.4f*la.u_max : 0.20f*la.u_max, la.u_max, gamma_c);
            la.data[i].y   = to_y(la.data[i].rho, 10/3.6f, la.u_max, gamma_c);

            x += la.h;
        }
    }

    {
        lanes.push_back(lane());

        lane &la = lanes.back();

        la.ncells = 100;
        la.data = (q*) malloc(la.ncells * sizeof (q));
        la.rs = (riemann_solution*) malloc((la.ncells+1) * sizeof (riemann_solution));
        la.h     = 1000.0f/(la.ncells-1);
        la.u_max = 40.0f/3.6f;

        float x = 0.0f;
        for(size_t i = 0; i < la.ncells; ++i)
        {
            la.data[i].rho = (i < 50) ? 0.4f : 0.4f;
            //            la.data[i].y   = to_y(la.data[i].rho, (i < 50) ? 0.4f*la.u_max : 0.20f*la.u_max, la.u_max, gamma_c);
            la.data[i].y   = to_y(la.data[i].rho, 10/3.6f, la.u_max, gamma_c);
            x += la.h;
        }
    }

    float the_time = 0.0f;

    sv = new spatial_view();
    pt.reset(1000, 500);
    pt.sv_ = sv;
    pt.do_corners_ = true;

    float total_length = 0.0f;
    foreach(lane &la, lanes)
    {
        total_length += la.ncells*la.h;
    }

    float used_length = 0.0f;
    foreach(lane &la, lanes)
    {
        {
            cairo_plotter1d *cp1d = new cairo_plotter1d;
            cp1d->stride = 2;
            cp1d->ncells = la.ncells;
            cp1d->data = &(la.data[0].rho);
            cp1d->h = la.h;
            cp1d->origin = -total_length*0.5 + used_length;

            pt.plts_.push_back(cp1d);
        }
        {
            cairo_plotter1d *cp1d = new cairo_plotter1d;
            cp1d->stride = 2;
            cp1d->ncells = la.ncells;
            cp1d->data = &(la.data[0].y);
            cp1d->h = la.h;
            cp1d->origin = -total_length*0.5 + used_length;

            pt.plts_.push_back(cp1d);
        }
        float length = la.ncells * la.h;

        used_length += length;
    }

    sim_window cw(0, 0, 1000, 500, &pt, "Tick test");

    cw.show();

    return Fl::run();
};
