#include <gsl/gsl_integration.h>
#include <cmath>
#include <cstdio>
#include <algorithm>

struct kinematics
{
    kinematics(double in_phi_max, double in_l, double in_v_max, double in_v_min, double in_m) :
        kappa_max(std::tan(in_phi_max)/in_l), L(in_l), m(in_m), inv_m(1.0/in_m)
    {
        float d1 = -0.5;
        v_coeff[2] = in_v_max;
        v_coeff[1] = 2*(in_v_min - in_v_max) - d1;
        v_coeff[0] = d1 - in_v_min + v_coeff[2];
    }

    double kappa_max;
    double L;

    double v_coeff[3];


    double m;
    double inv_m;
};

double ke_v(double t, void *n)
{
    const kinematics *ke = reinterpret_cast<const kinematics*>(n);

    t *= ke->inv_m;

    return ke->v_coeff[0]*t*t + ke->v_coeff[1]*t + ke->v_coeff[2];
}

double ke_theta(double t, void *n)
{
    const kinematics *ke = reinterpret_cast<const kinematics*>(n);

    if(t <= ke->m/2)
        return ke->kappa_max*t*t*ke->inv_m;
    else if(t <= ke->m)
        return -ke->kappa_max*(t*t*ke->inv_m - 2*t + ke->m/2);
    else
        return ke->kappa_max*ke->m/2;
};

double ke_x(double t, void *n)
{
    return ke_v(t, n)*std::cos(ke_theta(t, n));
}

double ke_y(double t, void *n)
{
    return ke_v(t, n)*std::sin(ke_theta(t, n));
}

int main(int argc, char **argv)
{
    if(argc < 6)
    {
        fprintf(stderr, "Usage: %s <start speed (m/s)> <end speed (m/s)> <final orientation (rad)> <car length (m)> <max wheel deflection (rad)> [nsamp]\n", argv[0]);
        exit(1);
    }

    double v0;
    sscanf(argv[1], "%lf", &v0);

    double v1;
    sscanf(argv[2], "%lf", &v1);

    double orientation;
    sscanf(argv[3], "%lf", &orientation);

    double car_length;
    sscanf(argv[4], "%lf", &car_length);

    double phi_max;
    sscanf(argv[5], "%lf", &phi_max);

    kinematics ke(phi_max, car_length, v0, v1, 0.0);

    gsl_function f_x = {ke_x, &ke};
    gsl_function f_y = {ke_y, &ke};

    ke.m = 2.0*orientation/ke.kappa_max;
    ke.inv_m = 1.0/ke.m;

    if(argc == 6)
    {
        double final_x;
        {
            double abserr;
            size_t neval;
            gsl_integration_qng(&f_x, 0, ke.m, 1e-4, 1e-3, &final_x, &abserr, &neval);
        }

        fprintf(stderr, "t_end x_end\n");
        fprintf(stderr, "%lf %lf\n", ke.m, final_x);
    }
    if(argc == 7)
    {
        size_t n;
        sscanf(argv[6], "%zu", &n);

        printf("t x y theta v\n");

        double interval[2] = {0.0, ke.m};

        for(size_t i = 0; i < n; ++i)
        {
            double t = (interval[1]-interval[0])*i/(n-1);

            double x, y, theta, v;
            double abserr;
            size_t neval;
            gsl_integration_qng(&f_x, 0, t, 1e-4, 1e-3, &x, &abserr, &neval);
            gsl_integration_qng(&f_y, 0, t, 1e-4, 1e-3, &y, &abserr, &neval);

            theta = ke_theta(t, &ke);
            v     = ke_v(t, &ke);

            printf("%lf %lf %lf %lf %lf \n", t, x, y, theta, v);
        }
    }

    return 0;
}

