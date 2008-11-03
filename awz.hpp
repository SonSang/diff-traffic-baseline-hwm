#ifndef _HWM_HPP_
#define _HWM_HPP_

#include <cmath>

/** Some equations:
  *equilibrium velocity:
  *\f[ u_{\textrm{eq}}\left(\rho\right) = u_{\textrm{max}}\left(1 - \rho^\gamma\right) \f]
  *
  *inverse of above:
  *\f[ \rho\left(u_{\textrm{eq}}\right) = \left(1 - \frac{u_{\textrm{eq}}}{u_{\textrm{max}}}\right)^{\frac{1}{\gamma}} \f]
  *
  *\f[ u'_{\textrm{eq}}\left(\rho\right) = - u_{\textrm{max}}\rho^{\left(\gamma-1\right)} \f]
  *
  *u from rho \& y;
  *\f[ u\left(\rho, y\right) = \frac{y}{\rho} + u_\textrm{eq}\left(\rho\right) \f]
  *
  *y from rho \& u:
  *\f[ y\left(\rho, u\right) = \rho\left(u - u_{\textrm{eq}}\right) \f]
*/

inline float eq_u(float rho, float u_max, float gamma)
{
    return u_max*(1.0f - std::pow(rho, gamma));
}

inline float inv_eq_u(float u_eq, float inv_u_max, float inv_gamma)
{
    return std::pow(1.0f - u_eq*inv_u_max, inv_gamma);
}

inline float eq_u_prime(float rho, float u_max, float gamma)
{
    return -u_max*std::pow(rho, gamma-1.0f);
}

struct q
{
    float rho;
    float y;
};

struct riemann_solution
{
    q waves[2];
    float speeds[2];
    q fluct_l;
    q fluct_r;
};

struct full_q
{
    full_q()
    {}

    full_q(const q *in,
           float u_max, float gamma) :
        rho(in->rho),
        y(in->y),
        u_eq(eq_u(rho, u_max, gamma)),
        u(y/rho + u_eq)
    {}

    full_q(float irho, float iu,
           float u_max, float gamma)
        : rho(irho),
          u(iu)
    {
        u_eq = eq_u(rho, u_max, gamma);
        y = rho*(u - u_eq);
    }

    inline void transonic_rarefaction(const full_q *o,
                                      float u_max, float gamma, float inv_gamma)
    {
        // we can simplify this
        (*this) = full_q(std::pow((o->u - o->u_eq + u_max)/(u_max*(gamma+1.0f)), inv_gamma),
                         gamma*(o->u - o->u_eq + u_max)/(gamma+1.0f),
                         u_max, gamma);
    }

    inline float lambda0(float u_max, float gamma) const
    {
        return u + rho*eq_u_prime(rho, u_max, gamma);
    }

    inline float lambda1() const
    {
        return u;
    }

    float rho;
    float y;

    float u_eq;
    float u;

    static inline void riemann(riemann_solution *rs,
                               const full_q *__restrict__ q_r,
                               const full_q *__restrict__ q_l,
                               float u_max,
                               float inv_u_max,
                               float gamma,
                               float inv_gamma)
    {
        const full_q *q_0;
        full_q q_m;

        if(q_l->u > q_r->u)
        {
            // we can simplify this
            q_m = full_q(inv_eq_u(q_r->u - (q_l->u - q_l->u_eq), inv_u_max, inv_gamma),
                         q_r->u,
                         u_max,
                         gamma);

            // Rankine-Hugoniot equation
            rs->speeds[0] = (q_m.rho * q_m.u - q_l->rho * q_l->u)/(q_m.rho - q_l->rho);

            q_0 = (rs->speeds[0] < 0.0f) ? &q_m : q_l;
        }
        else
        {
            float lambda0_l = q_l->lambda0(u_max, gamma);
            float lambda0_m;
            if(q_l->u - q_l->u_eq < q_r->u)
            {
                // we can simplify this
                q_m = full_q(inv_eq_u(q_r->u - (q_l->u - q_l->u_eq), inv_u_max, inv_gamma),
                             q_r->u,
                             u_max,
                             gamma);

                lambda0_m = q_m.lambda0(u_max, gamma);
            }
            else
                lambda0_m = q_l->u - q_l->u_eq;

            if(lambda0_l > 0.0f)
                q_0 = q_l;
            else
            {
                q_0 = &q_m;
                if(lambda0_m > 0.0f)
                    q_m.transonic_rarefaction(q_l,
                                              u_max, gamma, inv_gamma);
            }

            rs->speeds[0] = 0.5f*(lambda0_m+lambda0_l);
        }

        rs->speeds[1] = q_r->lambda1();

        rs->waves[0].rho = q_0->rho - q_l->rho;
        rs->waves[0].y   = q_0->y   - q_l->y;

        rs->waves[1].rho = q_r->rho - q_0->rho;
        rs->waves[1].y   = q_r->y   - q_0->y;

        rs->fluct_l.rho = q_0->rho*q_0->u - q_l->rho*q_l->u;
        rs->fluct_l.y   =   q_0->y*q_0->u -   q_l->y*q_l->u;
        rs->fluct_r.rho = q_r->rho*q_r->u - q_0->rho*q_0->u;
        rs->fluct_r.y   =   q_r->y*q_r->u -   q_0->y*q_0->u;
    }
};

#endif
