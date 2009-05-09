#include <gsl/gsl_integration.h>
#include <cmath>
#include <cstdio>
#include <algorithm>

struct kinematics
{
    kinematics(double in_phi_max, double in_l, double in_v, double in_m) :
        kappa_max(std::tan(in_phi_max)/in_l), L(in_l), v(in_v), m(in_m), inv_m(1.0/in_m)
    {}

    double kappa_max;
    double L;
    double v;

    double m;
    double inv_m;
};

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
    const kinematics *ke = reinterpret_cast<const kinematics*>(n);

    return ke->v*std::cos(ke_theta(t, n));
}

double ke_y(double t, void *n)
{
    const kinematics *ke = reinterpret_cast<const kinematics*>(n);

    return ke->v*std::sin(ke_theta(t, n));
}

int main(int argc, char **argv)
{
    if(argc < 5)
    {
        fprintf(stderr, "Usage: %s <speed (m/s)> <final orientation (rad)> <car length (m)> <max wheel deflection (rad)> [nsamp]\n", argv[0]);
        exit(1);
    }

    double velocity;
    sscanf(argv[1], "%lf", &velocity);

    double orientation;
    sscanf(argv[2], "%lf", &orientation);

    double car_length;
    sscanf(argv[3], "%lf", &car_length);

    double phi_max;
    sscanf(argv[4], "%lf", &phi_max);

    kinematics ke(phi_max, car_length, velocity, 0.0);

    gsl_function f_x = {ke_x, &ke};
    gsl_function f_y = {ke_y, &ke};

    ke.m = 2.0*orientation/ke.kappa_max;
    ke.inv_m = 1.0/ke.m;

    if(argc == 5)
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
    if(argc == 6)
    {
        size_t n;
        sscanf(argv[5], "%zu", &n);

        printf("t x y theta\n");

        double interval[2] = {0.0, ke.m};

        for(size_t i = 0; i < n; ++i)
        {
            double t = (interval[1]-interval[0])*i/(n-1);

            double x, y, theta;
            double abserr;
            size_t neval;
            gsl_integration_qng(&f_x, 0, t, 1e-4, 1e-3, &x, &abserr, &neval);
            gsl_integration_qng(&f_y, 0, t, 1e-4, 1e-3, &y, &abserr, &neval);

            theta = ke_theta(t, &ke);

            printf("%lf %lf %lf %lf\n", t, x, y, theta);
        }
    }

    return 0;
}

