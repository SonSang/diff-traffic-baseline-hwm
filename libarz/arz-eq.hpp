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
    return y/rho + u_eq(rho, u_max, gamma);
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

#endif
