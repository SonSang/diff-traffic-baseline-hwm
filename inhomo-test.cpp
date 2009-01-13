#include "arz.hpp"

int main()
{
    const float gamma_c = 0.5f;

    float speed_l = 4.0f;
    float speed_r = 0.5f;

    q q_l;
    q_l.rho =  0.4f;
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

    return 0;
}
