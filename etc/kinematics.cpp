#include <gsl/gsl_integration.h>
#include <cmath>
#include <cstdio>
#include <algorithm>

struct kinematics
{
    kinematics(double in_phi_max, double in_l, double in_v, double in_m) :
        phi_max(in_phi_max), L(in_l), v(in_v), m(in_m), inv_m(1.0/in_m)
    {}

    double phi_max;
    double L;
    double v;

    double m;
    double inv_m;
};

double ke_theta(double t, void *n)
{
    const kinematics *ke = reinterpret_cast<const kinematics*>(n);

    if(t <= ke->m/4)
        return 2*ke->phi_max*t*t*ke->inv_m;
    else if(t <= 3*ke->m/4)
        return -ke->phi_max*(2*t*t*ke->inv_m - 2*t + ke->m/4);
    else if(t <= ke->m)
        return ke->phi_max*(2*t*t*ke->inv_m - 4*t + 2*ke->m);
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

template <class F>
static inline double secant(F &f, double x0, double x1, double bottom, double top, double tol, int maxiter)
{
    double xn_1 = x0;
    double xn   = x1;
    double fn_1 = f(xn_1);
    double fn   = f(xn);
    double denom = fn-fn_1;
    for(int i = 0; std::abs(fn) > tol && std::abs(denom) > tol && i < maxiter; ++i)
    {
        double newxn = xn - (xn - xn_1)/denom * fn;
        while(newxn <= bottom)
            newxn = (newxn + xn)*0.5f;
        while(newxn >= top)
            newxn = (newxn + xn)*0.5f;
        xn_1 = xn;
        xn = newxn;
        fn_1 = fn;
        fn = f(xn);
        denom = (fn-fn_1);
    }
    return xn;
}

struct y_integrator
{
    y_integrator(kinematics &in_ke, gsl_function &in_f_y, double in_width)
        : ke(in_ke), f_y(in_f_y), width(in_width)
    {}

    inline double operator()(float m)
    {
        double x, y;
        double abserr;
        size_t neval;

        gsl_integration_qng(&f_y, 0, m, 1e-4, 1e-3, &y, &abserr, &neval);
        return y - width;
    }

    kinematics   &ke;
    gsl_function &f_y;
    double width;
};


int main(int argc, char **argv)
{
    double interval[2] = {0.0, M_PI};

    kinematics ke(M_PI/4.0, 1.0, 16.666667, M_PI);

    gsl_function f_x = {ke_x, &ke};
    gsl_function f_y = {ke_y, &ke};

    // size_t n = 200;

    // for(size_t i = 0; i < n; ++i)
    // {
    //     double t = (interval[1]-interval[0])*i/(n-1);

    //     double x, y;
    //     double abserr;
    //     size_t neval;
    //     gsl_integration_qng(&f_x, 0, t, 1e-4, 1e-3, &x, &abserr, &neval);
    //     gsl_integration_qng(&f_y, 0, t, 1e-4, 1e-3, &y, &abserr, &neval);

    //     printf("%lf %lf\n", x, y);
    // }
    y_integrator yi(ke, f_y, 4.0);

    double res = secant<y_integrator>(yi, 0.5, 1.0, 0.0, 20.0, 1e-4, 100);
    printf("res: %lf\n", res);

    return 0;
}

