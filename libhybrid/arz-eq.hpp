#ifndef __ARZ_EQ_HPP__
#define __ARZ_EQ_HPP__

#include <cmath>

template <typename T>
inline T arz<T>::eq::y(const T rho, const T u,
                       const T u_max,
                       const T gamma)
{
    return rho*(u - u_eq(rho, u_max, gamma));
}

template <typename T>
inline T arz<T>::eq::u(const T rho, const T y,
                       const T u_max,
                       const T gamma)
{
    if(rho < epsilon())
        return u_max;
    return std::max(y/rho + u_eq(rho, u_max, gamma), static_cast<T>(0));
}

template <typename T>
inline T arz<T>::eq::u_eq(const T rho,
                          const T u_max,
                          const T gamma)
{
    return u_max*(1.0 - std::pow(rho, gamma));
}

template <typename T>
inline T arz<T>::eq::inv_u_eq(const T u_eq,
                              const T inv_u_max,
                              const T inv_gamma)
{
    return std::pow(1.0 - u_eq*inv_u_max, inv_gamma);
}

template <typename T>
inline T arz<T>::eq::u_eq_prime(const T rho,
                                const T u_max,
                                const T gamma)
{
    return -u_max*gamma*std::pow(rho, gamma - 1);
}


template <typename T, class f>
inline T secant(const T x0, const T x1, const T bottom, const T top, const T tol, const int maxiter, const f &fnc)
{
    T xn_1 = x0;
    T xn   = x1;
    T fn_1 = fnc(xn_1);
    T fn   = fnc(xn);
    T denom = fn-fn_1;
    for(int i = 0; std::abs(fn) > tol && std::abs(denom) > tol && i < maxiter; ++i)
    {
        T newxn = xn - (xn - xn_1)/denom * fn;
        while(newxn <= bottom)
            newxn = (newxn + xn)*0.5;
        while(newxn >= top)
            newxn = (newxn + xn)*0.5;
        xn_1 = xn;
        xn = newxn;
        fn_1 = fn;
        fn = fnc(xn);
        denom = (fn-fn_1);
    }
    return xn;
}

template <typename T>
struct inv_rho_m_l
{
    inv_rho_m_l(const T flow_m_r, const T relv, const T u_max_l, const T gamma) :
        flow_m_r_(flow_m_r), relv_(relv), u_max_l_(u_max_l), gamma_(gamma)
    {}

    T operator()(const T rho_m_l) const
    {
        return flow_m_r_/rho_m_l - u_max_l_*(1.0 - std::pow(rho_m_l, gamma_)) - relv_;
    }

    T flow_m_r_;
    T relv_;
    T u_max_l_;
    T gamma_;
};

template <typename T>
inline T arz<T>::eq::rho_m_l_solve(const T flow_m_r, const T relv, const T u_max_l, const T gamma)
{
    const inv_rho_m_l<T> solver(flow_m_r, relv, u_max_l, gamma);

    return secant<T, inv_rho_m_l<T> >(0.3, 0.7, 1e-4, 1.0, 5e-6, 100, solver);
}

#endif
