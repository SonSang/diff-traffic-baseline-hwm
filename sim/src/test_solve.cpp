#include "arz.hpp"

#include <cstdlib>
#include <cstring>
#include <algorithm>

#include "timeseries.hpp"

static const float gamma_c = 0.5f;
static const float inv_gamma = 1.0f/gamma_c;

static const float u_max = 1.0f;
static const float inv_u_max = 1.0f/u_max;

static const float del_h = 0.05f;

static const char * fieldnames[] = {"rho", "y"};

static const size_t ncells = 100;
static riemann_solution *rs;
static q *data;

static float sim_step()
{
    full_q fq_buff[2];
    full_q * fq[2] = {fq_buff, fq_buff + 1};

    float maxspeed = 0.0f;

    memset(rs, 0, sizeof(riemann_solution));

    fq[0]->from_q(data,
                  u_max, gamma_c);

    for(size_t i = 1; i < ncells; ++i)
    {
        fq[1]->from_q(data + i,
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
    float coeff = dt/del_h;

    for(size_t i = 1; i < ncells; ++i)
    {
        data[i].rho -= coeff*(rs[i].fluct_r.rho + rs[i+1].fluct_l.rho);
        data[i].y   -= coeff*(rs[i].fluct_r.y   + rs[i+1].fluct_l.y);

        assert(data[i].rho > 0.0f);
        assert(data[i].y < 0.0f);
    }

    return dt;
}

int main(int argc, char * argv[])
{
    data = (q*) malloc(ncells * sizeof (q));
    rs = (riemann_solution*) malloc((ncells+1) * sizeof (riemann_solution));

    float x = 0.0f;
    for(size_t i = 0; i < ncells; ++i)
    {
        data[i].rho = 0.3f;
        data[i].y   = (i < 50) ? 0.2f : 0.1f;
        x += del_h;
    }

    grid_info gi(ncells, del_h*ncells);
    gi.calculate_strides();
    gi.bc_[0][0] = WABSORB;
    gi.bc_[0][1] = WABSORB;

    timeseries ts("out",
                  2,
                  fieldnames,
                  gi);

    ts.append(&(data[0].rho), 0.0f);

    float the_time = 0.0f;

    for(size_t t = 0; t < 100; ++t)
    {
        the_time += sim_step();
        ts.append(&(data[0].rho), the_time);

    }

    free(rs);
    free(data);
    return 0;
}
