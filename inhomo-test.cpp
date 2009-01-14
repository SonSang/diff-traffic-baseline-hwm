#include "arz.hpp"

template <class f>
static inline float secant(float x0, float x1, float bottom, float top, float tol, int maxiter, f & fnc)
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

float fundamental_diagram(float rho, float relv, float u_max, float gamma)
{
    return rho*(eq_u(rho, u_max, gamma) + relv);
}

float critical_density(float relv, float u_max, float gamma)
{
    return std::pow((u_max + relv)/(u_max*(1.0f+gamma)), 1.0f/gamma);
}

float max_flow(float relv, float u_max, float gamma)
{
    return fundamental_diagram(critical_density(relv, u_max, gamma), relv, u_max, gamma);
}

float demand(float rho, float relv, float u_max, float gamma)
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

float inv_demand(float flow, float relv, float u_max, float gamma)
{
    float crit = critical_density(relv, u_max, gamma);
    float max_flow = fundamental_diagram(crit, relv, u_max, gamma);

    if(flow >= max_flow)
        return crit;

    inv_fd solver(flow, relv, u_max, gamma);
    return secant<inv_fd>(crit*0.3f, crit, crit*0.05f, crit, 5e-8, 500, solver);
}

float inv_supply(float flow, float relv, float u_max, float gamma)
{
    float crit = critical_density(relv, u_max, gamma);
    float max_flow = fundamental_diagram(crit, relv, u_max, gamma);

    if(flow >= max_flow)
        return crit;

    inv_fd solver(flow, relv, u_max, gamma);
    return secant<inv_fd>(crit*1.3f, 1.0, crit, 1.0, 5e-8, 500, solver);
}

float supply(float rho, float relv, float u_max, float gamma)
{
    float crit = critical_density(relv, u_max, gamma);
    if(rho <= crit)
        rho = crit;
    return fundamental_diagram(rho, relv, u_max, gamma);
}

int main()
{
    const float gamma_c = 0.5f;

    float speed_l = 4.0f;
    float speed_r = 2.0f;

    q q_l;
    q_l.rho =  0.2f;
    q_l.y   = -0.1f;

    q q_r;
    q_r.rho =  0.5f;
    q_r.y   = -0.1f;

    full_q fq_l;
    fq_l.from_q(&q_l, speed_l, gamma_c);
    full_q fq_r;
    fq_r.from_q(&q_r, speed_r, gamma_c);

    full_q fq_l_o;
    fq_l_o.from_q(&q_l, speed_r, gamma_c);
    full_q fq_r_o;
    fq_r_o.from_q(&q_r, speed_l, gamma_c);


    float rho_ml, rho_mr;

    rho_ml = inv_eq_u(fq_r.u - fq_l.u + fq_l.u_eq, 1.0f/speed_l, 1.0f/gamma_c);
    rho_mr = inv_eq_u(fq_r.u - fq_l.u + fq_l.u_eq, 1.0f/speed_r, 1.0f/gamma_c);
    // rho_mr = inv_eq_u(fq_r.u - fq_l.u + eq_u(fq_l.rho, speed_r, gamma_c), 1.0f/speed_r, 1.0f/gamma_c);

    printf("        left:               right:\n");
    printf("mspeed: %8.5f           %8.5f\n", speed_l, speed_r);
    printf("rho:    %8.5f           %8.5f\n", q_l.rho, q_r.rho);
    printf("y:      %8.5f           %8.5f\n", q_l.y, q_r.y);
    printf("u_eq:   %8.5f           %8.5f\n", fq_l.u_eq, fq_r.u_eq);
    printf("u:      %8.5f           %8.5f\n", fq_l.u, fq_r.u);
    printf("u_eq(o):%8.5f           %8.5f\n", fq_l_o.u_eq, fq_r_o.u_eq);
    printf("u(o):   %8.5f           %8.5f\n", fq_l_o.u,    fq_r_o.u);
    printf("rho_m:  %8.5f           %8.5f\n", rho_ml, rho_mr);
    printf("u_m:    %8.5f           %8.5f\n", fq_r.u, fq_r.u);

    printf("fluct0:  %8.5f           %8.5f\n", rho_ml*fq_r.u - fq_l.rho*fq_l.u, fq_r.rho*fq_r.u - rho_ml*fq_r.u);
    printf("fluct1:  %8.5f           %8.5f\n", to_y(rho_ml, fq_r.u, speed_l, gamma_c)*fq_r.u - fq_l.y*fq_l.u,
           fq_r.y*fq_r.u - to_y(rho_ml, fq_r.u, speed_r, gamma_c)*fq_r.u);

    float demand_l = demand(fq_l.rho, fq_l.u - fq_l.u_eq, speed_l, gamma_c);
    float supply_r = supply(rho_mr, fq_l.u - fq_l.u_eq, speed_r, gamma_c);
    printf("demand (left): %f\n", demand(fq_l.rho, fq_l.u - fq_l.u_eq, speed_l, gamma_c));
    printf("critical (left): %f\n", critical_density(fq_l.u - fq_l.u_eq, speed_l, gamma_c));
    printf("max_flow (left): %f\n", max_flow(fq_l.u - fq_l.u_eq, speed_l, gamma_c));
    printf("supply (right): %f\n", supply(rho_mr, fq_l.u - fq_l.u_eq, speed_r, gamma_c));
    printf("inv_demand(left): %f\n", inv_demand(demand_l, fq_l.u - fq_l.u_eq, speed_l, gamma_c));
    printf("inv_supply(right): %f\n", inv_supply(supply_r, fq_l.u - fq_l.u_eq, speed_r, gamma_c));

    float rho_0l, rho_0r;
    if(demand_l <= supply_r)
    {
        rho_0l = fq_l.rho;
        rho_0r = inv_demand(demand_l, fq_l.u - fq_l.u_eq, speed_r, gamma_c);
    }
    else
    {
        rho_0l = inv_supply(supply_r, fq_l.u - fq_l.u_eq, speed_l, gamma_c);
        rho_0r = rho_mr;
    }

    printf("rho_0l = %f, rho_0r = %f\n", rho_0l, rho_0r);

    printf("u_eq_0l = %f, u_eq_0r = %f\n", eq_u(rho_0l, speed_l, gamma_c),
           eq_u(rho_0r, speed_r, gamma_c));

    printf("u_0l = %f, u_0r = %f\n", fq_l.u - fq_l.u_eq + eq_u(rho_0l, speed_l, gamma_c),
           fq_l.u - fq_l.u_eq + eq_u(rho_0r, speed_r, gamma_c));

    float u_0l = fq_l.u - fq_l.u_eq + eq_u(rho_0l, speed_l, gamma_c);
    float u_0r = fq_l.u - fq_l.u_eq + eq_u(rho_0r, speed_r, gamma_c);

    float y_0l = to_y(rho_0l, u_0l, speed_l, gamma_c);
    float y_0r = to_y(rho_0r, u_0r, speed_r, gamma_c);

    printf("f0 = %f\n", rho_0r*u_0r - rho_0l*u_0l);
    printf("f1 = %f\n", y_0r*u_0r - y_0l*u_0l);

    return 0;
}
