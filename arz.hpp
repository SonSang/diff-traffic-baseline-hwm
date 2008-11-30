#ifndef _HWM_HPP_
#define _HWM_HPP_

#include <cmath>
#include <cstring>
#include <cstdio>

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

// umax*rho^gamma = u_max - eq_u
inline float eq_u(float rho, float u_max, float gamma)
{
    return u_max*(1.0f - std::pow(rho, gamma));
}

inline float to_y(float rho, float u, float u_max, float gamma)
{
    return rho*(u - eq_u(rho, u_max, gamma));
}

inline float to_u(float rho, float y, float u_max, float gamma)
{
    return y/rho + eq_u(rho, u_max, gamma);
}

inline float inv_eq_u(float u_eq, float inv_u_max, float inv_gamma)
{
    return std::pow(1.0f - u_eq*inv_u_max, inv_gamma);
}

inline float eq_u_prime(float rho, float u_max, float gamma)
{
    return -u_max*gamma*std::pow(rho, gamma-1.0f);
}

inline float lambda_0(float rho, float u, float u_max, float gamma)
{
    return u + rho*eq_u_prime(rho, u_max, gamma);
}

inline float lambda_1(float u)
{
    return u;
}

inline float m_rho(float rho_l, float u_l, float u_r, float u_max, float inv_u_max, float gamma, float inv_gamma)
{
    return inv_eq_u(u_r - u_l + eq_u(rho_l, u_max, gamma), inv_u_max, inv_gamma);
}

inline float centered_rarefaction_rho(float rho_l, float u_l, float u_max, float gamma, float inv_gamma)
{
    return std::pow((u_l + u_max*std::pow(rho_l, gamma))/(u_max*(gamma+1.0f)), inv_gamma);
}

inline float centered_rarefaction_u(float rho_l, float u_l, float u_max, float gamma, float inv_gamma)
{
    return gamma/(gamma+1.0f)*(u_l + u_max*std::pow(rho_l, gamma));
}

struct q
{
    float rho;
    float y;
};

struct full_q
{
    void from_q(const q *inq, float u_max, float gamma)
    {
        rho  = inq->rho;
        y    = inq->y;
        u_eq = eq_u(rho, u_max, gamma);
        u    = to_u(rho, y, u_max, gamma);
    }

    float rho;
    float y;
    float u;
    float u_eq;
};

inline void centered_rarefaction(full_q *q_e, const full_q *q_l,
                                 float u_max, float gamma, float inv_gamma)
{
    q_e->rho = centered_rarefaction_rho(q_l->rho, q_l->u,
                                       u_max, gamma, inv_gamma);
    q_e->u = centered_rarefaction_u(q_l->rho, q_l->u,
                                   u_max, gamma, inv_gamma);

}

struct riemann_solution
{
    q waves[2];
    float speeds[2];
    q fluct_l;
    q fluct_r;
};

inline void riemann(riemann_solution *rs,
                    const full_q *__restrict__ q_l,
                    const full_q *__restrict__ q_r,
                    float u_max,
                    float inv_u_max,
                    float gamma,
                    float inv_gamma)
{
    const full_q *q_0;
    full_q q_m;

    if(std::abs(q_l->rho - q_r->rho) < 1e-6  && std::abs(q_l->y - q_r->y) < 1e-6)
    {
        memset(rs, 0, sizeof(rs));
        return;
    }
    else if(std::abs(q_l->u - q_r->u) < 1e-6)
    {
        q_0 = q_l;

        rs->waves[0].rho = 0.0f;
        rs->waves[0].y = 0.0f;
    }
    else if(q_l->u > q_r->u) // Case 1: left speed is greater than right speed
    {
        // we can simplify this
        q_m.rho = m_rho(q_l->rho, q_l->u,
                        q_r->u,
                        u_max, inv_u_max, gamma, inv_gamma);

        q_m.u    = q_r->u;
        q_m.u_eq = eq_u(q_m.rho, u_max, gamma);
        q_m.y    = to_y(q_m.rho, q_m.u, u_max, gamma);

        float lambda0_l = lambda_0(q_l->rho, q_l->u, u_max, gamma);
        float lambda0_m = lambda_0(q_m.rho,  q_m.u,  u_max, gamma);

        // Rankine-Hugoniot equation
        rs->speeds[0] = (q_m.rho * q_m.u - q_l->rho * q_l->u)/(q_m.rho - q_l->rho);

        rs->waves[0].rho = q_m.rho - q_l->rho;
        rs->waves[0].y   = q_m.y   - q_l->y;

        q_0 = (rs->speeds[0] > 0.0f) ? q_l : &q_m;
    }
    else
    {
        // in both case 2 and case 3, if lambda0_l > 0, q_0 = q_l
        float lambda0_l = lambda_0(q_l->rho, q_l->u, u_max, gamma);

        if(lambda0_l > 0.0f)
        {
            q_0 = q_l;
            rs->speeds[0] = lambda0_l;
        }
        else
        {
            float lambda0_m;

            bool case3;
            if(q_r->u - (u_max - q_l->u_eq) < q_l->u) // Case 2:
            {
                // we can simplify this
                q_m.rho = m_rho(q_l->rho, q_l->u,
                                q_r->u,
                                u_max, inv_u_max, gamma, inv_gamma);
                q_m.u    = q_r->u;
                q_m.u_eq = eq_u(q_m.rho, u_max, gamma);
                q_m.y    = to_y(q_m.rho, q_m.u, u_max, gamma);

                lambda0_m = lambda_0(q_m.rho, q_m.u, u_max, gamma);
                case3 = false;
            }
            else
            {
                lambda0_m = q_l->u + (u_max - q_l->u_eq);
                case3 = true;
            }

            if(case3 || lambda0_m > 0.0f)
                centered_rarefaction(&q_m, q_l,
                                     u_max, gamma, inv_gamma);
            else
                q_0 = &q_m;

            rs->speeds[0] = 0.5f*(lambda0_m+lambda0_l);
        }

    }

    rs->speeds[1] = lambda_1(q_r->u);

    rs->waves[1].rho = q_r->rho - q_0->rho;
    rs->waves[1].y   = q_r->y   - q_0->y;

    rs->fluct_l.rho = q_0->rho*q_0->u - q_l->rho*q_l->u;
    rs->fluct_l.y   = q_0->y  *q_0->u - q_l->y  *q_l->u;
    rs->fluct_r.rho = q_r->rho*q_r->u - q_0->rho*q_0->u;
    rs->fluct_r.y   = q_r->y  *q_r->u - q_0->y  *q_0->u;
};

#endif
