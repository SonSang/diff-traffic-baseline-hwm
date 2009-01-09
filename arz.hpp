#ifndef _HWM_HPP_
#define _HWM_HPP_

#include <cmath>
#include <cstring>
#include <cstdio>
#include <cassert>

// u_max*rho^gamma = u_max - eq_u
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
    if(rho == 0.0f)
        return 0.0f;
    else
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
        from_rho_y(inq->rho, inq->y, u_max, gamma);
    }

    void from_rho_y(float inrho, float iny, float u_max, float gamma)
    {
        rho  = inrho;
        y    = iny;
        u_eq = eq_u(rho, u_max, gamma);
        u    = to_u(rho, y, u_max, gamma);
    }

    void from_rho_u(float inrho, float inu, float u_max, float gamma)
    {
        rho  = inrho;
        u    = inu;
        y    = to_y(rho, u, u_max, gamma);
        u_eq = eq_u(rho, u_max, gamma);
    }

    float rho;
    float y;
    float u;
    float u_eq;
};

inline void centered_rarefaction(full_q *q_e, const full_q *q_l,
                                 float u_max, float gamma, float inv_gamma)
{
    q_e->from_rho_u(centered_rarefaction_rho(q_l->rho, q_l->u, u_max, gamma, inv_gamma),
                    centered_rarefaction_u(q_l->rho, q_l->u, u_max, gamma, inv_gamma),
                    u_max,
                    gamma);
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

    if(q_l->rho == 0.0f)
    {
        rs->speeds[0]    = 0.0f;
        rs->waves[0].rho = 0.0f;
        rs->waves[0].y   = 0.0f;

        rs->speeds[1]    = q_r->u;
        rs->waves[1].rho = q_r->rho;
        rs->waves[1].y   = q_r->y;

        memset(&q_m, 0, sizeof(q_m));
        q_0 = &q_m;
    }
    else if(q_r->rho == 0.0f)
    {
        // we can simplify this
        q_m.from_rho_u(0.0f,
                       q_l->u + (u_max - q_l->u_eq),
                       u_max, gamma);

        float lambda0_l = lambda_0(q_l->rho, q_l->u, u_max, gamma);
        float lambda0_m = q_m.u; //lambda_0(q_m.rho,  q_m.u,  u_max, gamma);

        assert(0.0f < lambda0_m);
        assert(lambda0_l < lambda0_m);

        rs->speeds[0] = 0.5f*(lambda0_l + lambda0_m);
        rs->waves[0].rho = q_m.rho - q_l->rho;
        rs->waves[0].y   = q_m.y   - q_l->y;

        rs->speeds[1] = 0.0f;
        rs->waves[1].rho = 0.0f;
        rs->waves[1].y   = 0.0f;

        if(lambda0_l > 0.0f)
            q_0 = q_l;
        else
        {
            centered_rarefaction(&q_m, q_l,
                                 u_max, gamma, inv_gamma);
            q_0 = &q_m;
        }
    }
    else if(std::abs(q_l->u - q_r->u) < 1e-6) // Case 0; u_l = u_r
    {
        rs->speeds[0]    = 0.0f;
        rs->waves[0].rho = 0.0f;
        rs->waves[0].y   = 0.0f;

        rs->speeds[1]    = q_r->u;
        rs->waves[1].rho = q_r->rho - q_l->rho;
        rs->waves[1].y   = q_r->y   - q_l->y;

        q_0 = q_l;
    }
    else if(q_l->u > q_r->u)
    {
        // we can simplify this
        q_m.from_rho_u(m_rho(q_l->rho, q_l->u,
                             q_r->u,
                             u_max, inv_u_max, gamma, inv_gamma),
                       q_r->u,
                       u_max, gamma);

        // Rankine-Hugoniot equation
        rs->speeds[0] = (q_m.rho * q_m.u - q_l->rho * q_l->u)/(q_m.rho - q_l->rho);
        rs->waves[0].rho = q_m.rho - q_l->rho;
        rs->waves[0].y   = q_m.y   - q_l->y;

#ifndef NDEBUG
        float lambda0_l = lambda_0(q_l->rho, q_l->u, u_max, gamma);
        float lambda0_m = lambda_0(q_m.rho,  q_m.u,  u_max, gamma);

        if(!((lambda0_l > lambda0_m) && (lambda0_l > rs->speeds[0]) && (lambda0_m < rs->speeds[0])))
        {
            printf("lambda0_l: %f\n", lambda0_l);
            printf("lambda0_m: %f\n", lambda0_m);
            printf("s        : %f\n", rs->speeds[0]);
            printf("q_l->rho: %f\n   ->u: %f\n   ->u_eq: %f\n   ->y: %f\n", q_l->rho, q_l->u, q_l->u_eq, q_l->y);
            printf("q_m->rho: %f\n   ->u: %f\n   ->u_eq: %f\n   ->y: %f\n", q_m.rho, q_m.u, q_m.u_eq, q_m.y);
            printf("q_r->rho: %f\n   ->u: %f\n   ->u_eq: %f\n   ->y: %f\n", q_r->rho, q_r->u, q_r->u_eq, q_r->y);
        }

        assert(lambda0_l > lambda0_m);
        assert(lambda0_l > rs->speeds[0]);
        assert(lambda0_m < rs->speeds[0]);

#endif

        rs->speeds[1] = q_r->u;
        rs->waves[1].rho = q_r->rho - q_m.rho;
        rs->waves[1].y   = q_r->y   - q_m.y;

        q_0 = (rs->speeds[0] > 0.0f) ? q_l : &q_m;
    }
    else if(q_l->u + (u_max - q_l->u_eq) > q_r->u) // Case 2
    {
        // we can simplify this
        q_m.from_rho_u(m_rho(q_l->rho, q_l->u,
                             q_r->u,
                             u_max, inv_u_max, gamma, inv_gamma),
                       q_r->u,
                       u_max, gamma);

        float lambda0_l = lambda_0(q_l->rho, q_l->u, u_max, gamma);
        float lambda0_m = lambda_0(q_m.rho,  q_m.u,  u_max, gamma);

        assert(lambda0_l < lambda0_m);

        rs->speeds[0] = 0.5f*(lambda0_l + lambda0_m);
        rs->waves[0].rho = q_m.rho - q_l->rho;
        rs->waves[0].y   = q_m.y   - q_l->y;

        rs->speeds[1] = q_r->u;
        rs->waves[1].rho = q_r->rho - q_m.rho;
        rs->waves[1].y   = q_r->y   - q_m.y;

        if(lambda0_l > 0.0f)
            q_0 = q_l;
        else if(lambda0_m < 0.0f)
            q_0 = &q_m;
        else
        {
            centered_rarefaction(&q_m, q_l,
                                 u_max, gamma, inv_gamma);
            q_0 = &q_m;
        }
    }
    else
    {
        // we can simplify this
        q_m.from_rho_u(0.0f,
                       q_l->u + (u_max - q_l->u_eq),
                       u_max, gamma);

        float lambda0_l = lambda_0(q_l->rho, q_l->u, u_max, gamma);
        float lambda0_m = q_m.u; //lambda_0(q_m.rho,  q_m.u,  u_max, gamma);

        assert(0.0f < lambda0_m);
        assert(lambda0_l < lambda0_m);

        rs->speeds[0] = 0.5f*(lambda0_l + lambda0_m);

        rs->waves[0].rho = q_m.rho - q_l->rho;
        rs->waves[0].y   = q_m.y   - q_l->y;

        rs->speeds[1] = q_r->u;
        rs->waves[1].rho = q_r->rho - q_m.rho;
        rs->waves[1].y   = q_r->y   - q_m.y;

        if(lambda0_l > 0.0f)
            q_0 = q_l;
        else
        {
            centered_rarefaction(&q_m, q_l,
                                 u_max, gamma, inv_gamma);
            q_0 = &q_m;
        }
    }

    //    assert(rs->speeds[0] < rs->speeds[1]);

    rs->fluct_l.rho = q_0->rho*q_0->u - q_l->rho*q_l->u;
    rs->fluct_l.y   = q_0->y  *q_0->u - q_l->y  *q_l->u;
    rs->fluct_r.rho = q_r->rho*q_r->u - q_0->rho*q_0->u;
    rs->fluct_r.y   = q_r->y  *q_r->u - q_0->y  *q_0->u;
};

inline void starvation_riemann(riemann_solution *rs,
                               const full_q *__restrict__ q_r,
                               float u_max,
                               float inv_u_max,
                               float gamma,
                               float inv_gamma)
{
    if(q_r->rho == 0.0f)
        memset(rs, 0, sizeof(riemann_solution));
    else
    {
        rs->speeds[0]    = 0.0f;
        rs->waves[0].rho = 0.0f;
        rs->waves[0].y   = 0.0f;
        rs->fluct_l.rho  = 0.0f;
        rs->fluct_l.y    = 0.0f;

        rs->speeds[1]    = q_r->u;
        rs->waves[1].rho = q_r->rho;
        rs->waves[1].y   = q_r->y;

        rs->fluct_r.rho = q_r->rho*q_r->u;
        rs->fluct_r.y   = q_r->y  *q_r->u;
    }
}

inline void stop_riemann(riemann_solution *rs,
                         const full_q *__restrict__ q_l,
                         float u_max,
                         float inv_u_max,
                         float gamma,
                         float inv_gamma)
{
    full_q q_m;

    q_m.from_rho_u(inv_eq_u(q_l->u_eq - q_l->u, u_max, gamma),
                   0.0,
                   u_max, gamma);

    rs->speeds[0]    = 0.0f;
    if(q_m.rho - q_l->rho != 0.0f)
        rs->speeds[0] = -q_l->rho * q_l->u/(q_m.rho - q_l->rho);

    rs->waves[0].rho = -q_l->rho;
    rs->waves[0].y   = -q_l->y;
    rs->fluct_l.rho  = -q_l->rho * q_l->u;
    rs->fluct_l.y    = -q_l->y   * q_l->u;

    rs->speeds[1]    = 0.0f;
    rs->waves[1].rho = 0.0f;
    rs->waves[1].y   = 0.0f;
    rs->fluct_r.rho  = 0.0f;
    rs->fluct_r.y    = 0.0f;
}

inline float MC_limiter(float x)
{
    if (x <= 0.0f)
        return 0.0f;
    if (x <= 1.0f/3.0f)
        return 2.0f*x;
    if (x <= 3.0f)
        return x*0.5f+0.5f;
    return 2.0f;
}

inline q flux_correction(riemann_solution *__restrict__ soln_m,
                         riemann_solution *__restrict__ soln,
                         riemann_solution *__restrict__ soln_p,
                         float coeff)
{
    float aspeed = std::abs(soln->speeds[0]);
    float fcc = 0.5f*aspeed*(1.0f-coeff*aspeed);

    const q * upwind;
    if(soln->speeds[0] > 0.0f)
        upwind = &(soln_m->waves[0]);
    else
        upwind = &(soln_p->waves[0]);

    float len = soln->waves[0].rho*soln->waves[0].rho + soln->waves[0].y*soln->waves[0].y;
    if(len > 0.0f)
        len = 1.0f/len;

    float theta = (*upwind).rho*soln->waves[0].rho + (*upwind).y*soln->waves[0].y;

    theta = MC_limiter(theta*len);

    q res = {fcc*theta*soln->waves[0].rho,
             fcc*theta*soln->waves[0].y};

    fcc = 0.5f*soln->speeds[1]*(1.0f-coeff*soln->speeds[1]);

    len = soln->waves[1].rho*soln->waves[1].rho + soln->waves[1].y*soln->waves[1].y;
    if(len > 0.0f)
        len = 1.0f/len;

    theta = soln_m->waves[1].rho*soln->waves[1].rho + soln_m->waves[1].y*soln->waves[1].y;

    theta = MC_limiter(theta*len);

    res.rho += fcc*theta*soln->waves[1].rho;
    res.y   += fcc*theta*soln->waves[1].y;

    return res;
}
#endif
