#ifndef _HWM_HPP_
#define _HWM_HPP_

#include <cmath>
#include <cstring>
#include <cfloat>
#include <cstdio>
#include <cassert>

template <class f>
inline float secant(float x0, float x1, float bottom, float top, float tol, int maxiter, f &fnc)
{
    float xn_1 = x0;
    float xn   = x1;
    float fn_1 = fnc(xn_1);
    float fn   = fnc(xn);
    float denom = fn-fn_1;
    for(int i = 0; std::abs(fn) > tol && std::abs(denom) > tol && i < maxiter; ++i)
    {
        float newxn = xn - (xn - xn_1)/denom * fn;
        while(newxn <= bottom)
            newxn = (newxn + xn)*0.5f;
        while(newxn >= top)
            newxn = (newxn + xn)*0.5f;
        xn_1 = xn;
        xn = newxn;
        fn_1 = fn;
        fn = fnc(xn);
        denom = (fn-fn_1);
    }
    return xn;
}

struct inv_rho_m_l
{
    inv_rho_m_l(float flow_m_r, float relv, float u_max_l, float gamma) :
        flow_m_r_(flow_m_r), relv_(relv), u_max_l_(u_max_l), gamma_(gamma)
    {}

    float operator()(float rho_m_l) const
    {
        return flow_m_r_/rho_m_l - u_max_l_*(1.0 - std::pow(rho_m_l, gamma_)) - relv_;
    }

    float flow_m_r_;
    float relv_;
    float u_max_l_;
    float gamma_;
};

inline float rho_m_l_solve(float flow_m_r, float relv, float u_max_l, float gamma)
{
    inv_rho_m_l solver(flow_m_r, relv, u_max_l, gamma);

    return secant<inv_rho_m_l>(0.3f, 0.7f, 1e-4f, 1.0f, 5e-6, 100, solver);
}

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
    if(rho < FLT_EPSILON)
        return u_max;
    else
    {
        float res = y/rho + eq_u(rho, u_max, gamma);
        if(res < FLT_EPSILON)
            return 0.0f;
        else
            return res;
    }
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

    if(q_l->rho < 1e-4)
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
    else if(q_r->rho < 1e-4)
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

        rs->speeds[1]    = rs->speeds[0];
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

#if 0
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
    assert(rs->speeds[0] <= rs->speeds[1]);

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
    if(q_r->rho < FLT_EPSILON)
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

    q_m.from_rho_u(inv_eq_u(q_l->u_eq - q_l->u, u_max, inv_gamma),
                   0.0,
                   u_max, gamma);

    rs->speeds[0]    = 0.0f;
    if(std::abs(q_m.rho - q_l->rho) > FLT_EPSILON)
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

inline void inhomogeneous_riemann(riemann_solution *rs,
                                  const full_q *__restrict__ q_l,
                                  const full_q *__restrict__ q_r,
                                  float u_max_l,
                                  float u_max_r,
                                  float gamma,
                                  float inv_gamma)
{
    full_q q_m_l, q_m_r;

    printf("q_r->u - q_l->u + q_l->u_eq: %f\n", q_r->u - q_l->u + q_l->u_eq);

    q_m_r.from_rho_u(inv_eq_u(q_r->u - q_l->u + q_l->u_eq, 1.0f/u_max_r, inv_gamma),
                     q_r->u,
                     u_max_r,
                     gamma);

    float rho_m_l = rho_m_l_solve(q_m_r.rho*q_r->u, q_l->u - q_l->u_eq, u_max_l, gamma);
    q_m_l.from_rho_u(rho_m_l,
                     q_m_r.rho*q_r->u/rho_m_l,
                     u_max_l,
                     gamma);

    printf("          q_l:        q_m_l:      q_m_r:      q_r\n");
    printf("rho:      %8.5f       %8.5f       %8.5f       %8.5f\n", q_l->rho,q_m_l.rho, q_m_r.rho,q_r->rho);
    printf("y:        %8.5f       %8.5f       %8.5f       %8.5f\n", q_l->y,  q_m_l.y, q_m_r.y,    q_r->y  );
    printf("u:        %8.5f       %8.5f       %8.5f       %8.5f\n", q_l->u,  q_m_l.u, q_m_r.u,    q_r->u  );
    printf("lambda_0: %8.5f       %8.5f       %8.5f       %8.5f\n",
           lambda_0(q_l->rho, q_l->u, u_max_l, gamma),
           lambda_0(q_m_l.rho, q_m_l.u, u_max_l, gamma),
           lambda_0(q_m_r.rho, q_m_r.u, u_max_r, gamma),
           lambda_0(q_r->rho, q_r->u, u_max_r, gamma));

    memset(rs, 0, sizeof(riemann_solution));

    if(std::abs(q_m_l.rho - q_l->rho) < FLT_EPSILON)
    {
        printf("numer %f, denom %f\n", (q_m_l.rho * q_m_l.u - q_l->rho * q_l->u),(q_m_l.rho - q_l->rho));
        rs->speeds[0] = 0.0f;
    }
    else
        rs->speeds[0] = (q_m_l.rho * q_m_l.u - q_l->rho * q_l->u)/(q_m_l.rho - q_l->rho);

    printf("s = %f\n", rs->speeds[0]);

    rs->speeds[1] = q_r->u;

    rs->fluct_l.rho = q_m_l.rho*q_m_l.u - q_l->rho*q_l->u;
    rs->fluct_l.y   = q_m_l.y  *q_m_l.u - q_l->y  *q_l->u;
    rs->fluct_r.rho = q_r->rho*q_r->u   - q_m_r.rho*q_m_r.u;
    rs->fluct_r.y   = q_r->y  *q_r->u   - q_m_r.y  *q_m_r.u;
};

inline float fundamental_diagram(float rho, float relv, float u_max, float gamma)
{
    return rho*(eq_u(rho, u_max, gamma) + relv);
}

inline float critical_density(float relv, float u_max, float gamma)
{
    return std::pow((u_max + relv)/(u_max*(1.0f+gamma)), 1.0f/gamma);
}

inline float max_flow(float relv, float u_max, float gamma)
{
    return fundamental_diagram(critical_density(relv, u_max, gamma), relv, u_max, gamma);
}

inline float demand(float rho, float relv, float u_max, float gamma)
{
    float crit = critical_density(relv, u_max, gamma);
    if(rho > crit)
        rho = crit;
    return fundamental_diagram(rho, relv, u_max, gamma);
}

struct inv_fd
{
    inv_fd(float flow, float relv, float u_max, float gamma) :
        flow_(flow), relv_(relv), u_max_(u_max), gamma_(gamma)
    {}

    float operator()(float rho) const
    {
        float fd0 = fundamental_diagram(rho, relv_, u_max_, gamma_);
        return fd0-flow_;
    }

    float flow_;
    float relv_;
    float u_max_;
    float gamma_;
};

inline float inv_demand(float flow, float relv, float u_max, float gamma)
{
    float crit = critical_density(relv, u_max, gamma);
    float max_flow = fundamental_diagram(crit, relv, u_max, gamma);

    if(flow >= max_flow)
        return crit;

    inv_fd solver(flow, relv, u_max, gamma);
    return secant<inv_fd>(crit*0.3f, crit, crit*0.05f, crit, 5e-8, 500, solver);
}

inline float inv_supply(float flow, float relv, float u_max, float gamma)
{
    float crit = critical_density(relv, u_max, gamma);
    float max_flow = fundamental_diagram(crit, relv, u_max, gamma);

    if(flow >= max_flow)
        return crit;

    inv_fd solver(flow, relv, u_max, gamma);
    return secant<inv_fd>(crit*1.3f, 1.0, crit, 1.0, 5e-8, 500, solver);
}

inline float supply(float rho, float relv, float u_max, float gamma)
{
    float crit = critical_density(relv, u_max, gamma);
    if(rho <= crit)
        rho = crit;
    return fundamental_diagram(rho, relv, u_max, gamma);
}

inline void lebacque_inhomogeneous_riemann(riemann_solution *rs,
                                           const full_q *__restrict__ q_l,
                                           const full_q *__restrict__ q_r,
                                           float u_max_l,
                                           float u_max_r,
                                           float gamma,
                                           float inv_gamma)
{
    full_q q_m_l, q_m_r;

    float rho_m = inv_eq_u(q_r->u - q_l->u + q_l->u_eq, 1.0f/u_max_r, 1.0f/gamma);

    float demand_l = demand(q_l->rho,  q_l->u - q_l->u_eq, u_max_l, gamma);
    float supply_r = supply(rho_m,     q_l->u - q_l->u_eq, u_max_r, gamma);

    if(demand_l <= supply_r)
    {
        q_m_l.rho = q_l->rho;
        q_m_r.rho = inv_demand(demand_l, q_l->u - q_l->u_eq, u_max_r, gamma);
    }
    else
    {
        q_m_l.rho = inv_supply(supply_r, q_l->u - q_l->u_eq, u_max_l, gamma);
        q_m_r.rho = rho_m;
    }

    q_m_l.u = q_l->u - q_l->u_eq + eq_u(q_m_l.rho, u_max_l, gamma);
    q_m_r.u = q_l->u - q_l->u_eq + eq_u(q_m_r.rho, u_max_r, gamma);

    q_m_l.y = to_y(q_m_l.rho, q_m_l.u, u_max_l, gamma);
    q_m_r.y = to_y(q_m_r.rho, q_m_r.u, u_max_r, gamma);

    printf("          q_l:        q_m_l:      q_m_r:      q_r\n");
    printf("rho:      %8.5f       %8.5f       %8.5f       %8.5f\n", q_l->rho,q_m_l.rho, q_m_r.rho,q_r->rho);
    printf("y:        %8.5f       %8.5f       %8.5f       %8.5f\n", q_l->y,  q_m_l.y, q_m_r.y,    q_r->y  );
    printf("u:        %8.5f       %8.5f       %8.5f       %8.5f\n", q_l->u,  q_m_l.u, q_m_r.u,    q_r->u  );
    printf("lambda_0: %8.5f       %8.5f       %8.5f       %8.5f\n",
           lambda_0(q_l->rho, q_l->u, u_max_l, gamma),
           lambda_0(q_m_l.rho, q_m_l.u, u_max_l, gamma),
           lambda_0(q_m_r.rho, q_m_r.u, u_max_r, gamma),
           lambda_0(q_r->rho, q_r->u, u_max_r, gamma));

    memset(rs, 0, sizeof(riemann_solution));

    if(std::abs(q_m_l.rho - q_l->rho) < FLT_EPSILON)
    {
        printf("numer %f, denom %f\n", (q_m_l.rho * q_m_l.u - q_l->rho * q_l->u),(q_m_l.rho - q_l->rho));
        rs->speeds[0] = 0.0f;
    }
    else
        rs->speeds[0] = (q_m_l.rho * q_m_l.u - q_l->rho * q_l->u)/(q_m_l.rho - q_l->rho);

    printf("s = %f\n", rs->speeds[0]);

    rs->speeds[1] = q_r->u;

    rs->fluct_l.rho = q_m_l.rho*q_m_l.u - q_l->rho*q_l->u;
    rs->fluct_l.y   = q_m_l.y  *q_m_l.u - q_l->y  *q_l->u;
    rs->fluct_r.rho = q_r->rho*q_r->u   - q_m_r.rho*q_m_r.u;
    rs->fluct_r.y   = q_r->y  *q_r->u   - q_m_r.y  *q_m_r.u;
};

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
