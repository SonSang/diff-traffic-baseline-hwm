#ifndef __ARZ_IMPL_HPP__
#define __ARZ_IMPL_HPP__

#include <limits>
#include <cstring>
#include <cstdio>

// arz implementation
template <typename T>
inline T arz<T>::epsilon()
{
    return 1e-3;
}

// q implemenation
template <typename T>
arz<T>::q::q()
    : base()
{}

template <typename T>
arz<T>::q::q(const q &restrict o)
    : base(o)
{}

template <typename T>
arz<T>::q::q(const base &restrict o)
    : base(o)
{}

template <typename T>
template <class E>
arz<T>::q::q(const tvmet::XprVector< E, 2 > &restrict e)
    : base(e)
{}

template <typename T>
arz<T>::q::q(const T in_rho, const T in_y)
    : base(in_rho, in_y)
{}

template <typename T>
arz<T>::q::q(const T in_rho, const T in_u,
             const T u_max)
    : base(in_rho, std::min(eq::y(in_rho, in_u, u_max), 0.0f))
{}

template <typename T>
inline const T arz<T>::q::rho() const
{
    return (*this)[0];
}

template <typename T>
inline T& arz<T>::q::rho()
{
    return (*this)[0];
}

template <typename T>
inline const T arz<T>::q::y() const
{
    return (*this)[1];
}

template <typename T>
inline T& arz<T>::q::y()
{
    return (*this)[1];
}

template <typename T>
inline void arz<T>::q::fix()
{
    if(rho() <= epsilon())
    {
        rho() = 0.0;
        y()   = 0.0;
    }
    else if(rho() > 1.0-epsilon())
    {
        rho() = 1.0-epsilon();
    }
    if(y() > -epsilon())
        y() = 0.0;

    assert(check());
}

template <typename T>
inline bool arz<T>::q::check() const
{
    return xisfinite(rho()) && rho() >= 0 && rho() <= 1.0f
    && xisfinite(y()) && y() <= 0;
}

// full_q implemenation
template <typename T>
arz<T>::full_q::full_q()
    : base()
{}

template <typename T>
arz<T>::full_q::full_q(const full_q &restrict o)
    : base(o),
      u_eq_(o.u_eq_),
      u_(o.u_)
{
    assert(check());
}

template <typename T>
arz<T>::full_q::full_q(const q &restrict o,
                       const T               u_max)
    : base(o)
{
    u_eq_ = eq::u_eq(base::rho(), u_max);
    u_    = base::rho() <= std::numeric_limits<T>::epsilon() ? u_max :
            std::max(base::y()/base::rho() + u_eq_,
                     static_cast<T>(0));
    assert(check());
}

template <typename T>
arz<T>::full_q::full_q(const T in_rho, const T in_u,
                       const T u_max)
{
    base::rho() = in_rho;
    u_eq_       = eq::u_eq(base::rho(), u_max);
    u_          = std::min(u_eq_, in_u);
    base::y()   = base::rho()*(u() - u_eq_);
    assert(check());
}

template <typename T>
inline void arz<T>::full_q::clear()
{
    memset(this, 0, sizeof(*this));
}

template <typename T>
inline const T& arz<T>::full_q::u() const
{
    return u_;
}

template <typename T>
inline T& arz<T>::full_q::u()
{
    return u_;
}

template <typename T>
inline const T& arz<T>::full_q::u_eq() const
{
    return u_eq_;
}

template <typename T>
inline T& arz<T>::full_q::u_eq()
{
    return u_eq_;
}

template <typename T>
inline tvmet::XprVector<
    tvmet::XprBinOp<
        tvmet::Fcnl_mul<T, T>,
        tvmet::VectorConstReference<T, 2>,
        tvmet::XprLiteral<T>
        >,
    2 > arz<T>::full_q::flux() const
{
    return (*this)*u();
}

template <typename T>
inline T arz<T>::full_q::flux_0() const
{
    return base::rho()*u();
}

template <typename T>
inline T arz<T>::full_q::flux_1() const
{
    return base::y()*u();
}

template <typename T>
inline T arz<T>::full_q::lambda_0(const T u_max) const
{
    return u() + base::rho()*eq::u_eq_prime(base::rho(), u_max);
}

template <typename T>
inline T arz<T>::full_q::lambda_1(const T u_max) const
{
    return u();
}

template <typename T>
inline typename arz<T>::full_q arz<T>::centered_rarefaction(const full_q &restrict q_l,
                                                            const T                    u_max)
{
    full_q res;

    res.u()    = GAMMA*(q_l.u() + u_max - q_l.u_eq())/(GAMMA+1);
    res.u_eq() = u_max - res.u()/(u_max*GAMMA);
    res.rho()  = (u_max - res.u_eq())*(u_max - res.u_eq());
    res.y()    = res.rho()*(res.u() - res.u_eq());

    return res;
}

template <typename T>
inline typename arz<T>::full_q arz<T>::rho_middle(const full_q &restrict q_l,
                                                  const full_q &restrict q_r,
                                                  const T                    inv_u_max)
{
    full_q res;

    res.u_eq() = q_r.u() - q_l.u() + q_l.u_eq();
    res.rho()  = (1 - res.u_eq()*inv_u_max)*(1 - res.u_eq()*inv_u_max);
    res.u()    = q_r.u();
    res.y()    = res.rho()*(res.u() - res.u_eq());

    return res;
}

template <typename T>
inline bool arz<T>::full_q::check() const
{
    return q::check() && xisfinite(u()) && u() >= 0.0f
    && xisfinite(u_eq()) && u_eq() >= 0.0f;
}

template <typename T>
inline void arz<T>::riemann_solution::riemann(const full_q &restrict q_l,
                                              const full_q &restrict q_r,
                                              const T                    u_max,
                                              const T                    inv_u_max)
{
    const full_q *fq_0;
    full_q        q_m;

    if(q_l.rho() < VACUUM_EPS)
    {   // case 4
        speeds[0] = 0.0;
        waves [0] = q(0.0, 0.0);

        speeds[1] = q_l.u();
        waves [1] = q_l;

        q_m.clear();
        fq_0 = &q_m;
    }
    else if(q_r.rho() < VACUUM_EPS)
    {   // case 5
        q_m = full_q(0.0, q_l.u() + (u_max - q_l.u_eq()),
                     u_max);

        const T lambda_0_l = q_l.lambda_0(u_max);
        const T lambda_0_m = q_m.u();

        speeds[0] = (lambda_0_l + lambda_0_m)/2;
        waves [0] = q_m - q_l;

        speeds[1] = speeds[0];
        waves [1] = q(0.0, 0.0);

        if(lambda_0_l > 0.0)
            fq_0 = &q_l;
        else
        {
            q_m = centered_rarefaction(q_l, u_max);
            fq_0 = &q_m;
        }
    }
    else if(std::abs(q_l.u() - q_r.u()) < epsilon())
    {   // case 0
        speeds[0] = 0.0;
        waves [0] = q(0.0, 0.0);

        speeds[1] = q_r.u();
        waves [1] = q_r - q_l;

        fq_0 = &q_l;
    }
    else if(q_l.u() > q_r.u())
    {   // case 1
        q_m = rho_middle(q_l, q_r, inv_u_max);

        const T flux_0_diff = q_m.flux_0() - q_l.flux_0();
        speeds[0] = std::abs(flux_0_diff) < epsilon() ? 0.0 : flux_0_diff/(q_m.rho() - q_l.rho());
        waves [0] = q_m - q_l;

        speeds[1] = q_r.u();
        waves [1] = q_r - q_m;

        fq_0 = (speeds[0] >= 0.0) ? &q_l : &q_m;
    }
    else if(u_max + q_l.u() - q_l.u_eq() > q_r.u())
    {   // case 2
        q_m = rho_middle(q_l, q_r, inv_u_max);

        const T lambda0_l = q_l.lambda_0(u_max);
        const T lambda0_m = q_m.lambda_0(u_max);

        speeds[0] = (lambda0_l + lambda0_m)/2;
        waves [0] = q_m - q_l;

        speeds[1] = q_r.u();
        waves [1] = q_r - q_m;

        if(lambda0_l >= 0.0)
            fq_0 = &q_l;
        else if(lambda0_m < 0.0)
            fq_0 = &q_m;
        else
        {
            q_m = centered_rarefaction(q_l, u_max);
            fq_0 = &q_m;
        }
    }
    else
    {   // case 3
        q_m.rho()  = 0.0;
        q_m.y()    = 0.0;
        q_m.u_eq() = u_max;
        q_m.u()    = u_max + q_l.u() - q_l.u_eq();

        const T lambda0_l = q_l.lambda_0(u_max);
        const T lambda0_m = q_m.u();

        speeds[0] = (lambda0_l + lambda0_m)/2;
        waves [0] = q_m - q_l;

        speeds[1] = q_r.u();
        waves [1] = q_r - q_m;

        if(lambda0_l >= 0.0)
            fq_0 = &q_l;
        else
        {
            q_m = centered_rarefaction(q_l, u_max);
            fq_0 = &q_m;
        }
    }

    left_fluctuation  = fq_0->flux() -  q_l. flux();
    right_fluctuation =  q_r. flux() - fq_0->flux();
    q_0               = arz<float>::q(fq_0->rho(), fq_0->y());
}

//     const full_q q_m_r(eq::inv_u_eq(q_r.u() - q_l.u() + q_l.u_eq(), 1.0f/u_max_r, inv_gamma),
//                        q_r.u(),
//                        u_max_r,
//                        gamma);

//     const float rho_m_l = eq::rho_m_l_solve(q_m_r.rho()*q_r.u(), q_l.u() - q_l.u_eq(), u_max_l, gamma);
//     const full_q q_m_l(rho_m_l,
//                        q_m_r.rho()*q_r.u()/rho_m_l,
//                        u_max_l,
//                        gamma);

//     const T flux_0_diff = q_m_l.flux_0() - q_l.flux_0();
//     speeds[0]           = std::abs(flux_0_diff) < epsilon() ? 0.0 : flux_0_diff/(q_m_l.rho() - q_l.rho());
//     waves [0]           = q_m_l - q_l;

//     speeds[1]           = q_r.u();
//     waves [1]           = q_r - q_m_r;

//     left_fluctuation  = q_m_l.flux() - q_l.flux();
//     right_fluctuation = q_r.flux()   - q_m_r.flux();
//     q_0               = arz<float>::q(q_m_r.rho(), q_m_r.y()); // choice of right value is (fairly) arbitrary
// };

template <typename T>
inline void arz<T>::riemann_solution::lebaque_inhomogeneous_riemann(const full_q &restrict q_l,
                                                                    const full_q &restrict q_r,
                                                                    const float u_max_l,
                                                                    const float u_max_r)
{
    const T rho_m = std::min(static_cast<float>(1.0), eq::inv_u_eq(q_r.u() - q_l.u() + q_l.u_eq(), 1.0f/u_max_r));

    //    printf("rho_m:     %8.5f\n", rho_m);

    const T demand_l = std::max(0.0f, demand(q_l.rho(),  q_l.u() - q_l.u_eq(), u_max_l));
    const T supply_r = std::max(0.0f, supply(rho_m,      q_l.u() - q_l.u_eq(), u_max_r));

    float m_l_rho, m_r_rho;
    if(demand_l <= supply_r)
    {
        m_l_rho = q_l.rho();
        m_r_rho = inv_demand(demand_l, q_l.u() - q_l.u_eq(), u_max_r);
    }
    else
    {
        m_l_rho = inv_supply(supply_r, q_l.u() - q_l.u_eq(), u_max_l);
        m_r_rho = rho_m;
    }
    assert(m_l_rho >= 0.0 && m_l_rho <= 1.0);
    assert(m_r_rho >= 0.0 && m_r_rho <= 1.0);

    //    const float ueq = eq::u_eq(m_l_rho, u_max_l, gamma);

    const full_q q_m_r(m_r_rho,
                       std::max(static_cast<T>(0.0), q_l.u() - q_l.u_eq() + eq::u_eq(m_r_rho, u_max_r)),
                       u_max_r);

    const full_q q_m_l(m_l_rho,
                       std::max(static_cast<T>(0.0), q_l.u() - q_l.u_eq() + eq::u_eq(m_l_rho, u_max_l)),
                       u_max_l);


//     printf("demand_l:   %8.5f       supply_r:     %8.5f\n", demand_l, supply_r);

//     printf("          q_l:        q_m_l:      q_m_r:      q_r\n");
//     printf("rho:      %8.5f       %8.5f       %8.5f       %8.5f\n", q_l.rho(),q_m_l.rho(), q_m_r.rho(),q_r.rho());
//     printf("y:        %8.5f       %8.5f       %8.5f       %8.5f\n", q_l.y(),  q_m_l.y(), q_m_r.y(),    q_r.y()  )
// ;    printf("u_eq:    %8.5f       %8.5f       %8.5f       %8.5f\n", q_l.u_eq(),q_m_l.u_eq(), q_m_r.u_eq(),q_r.u_eq());
//     printf("u(real):  %8.5f       %8.5f       %8.5f       %8.5f\n", eq::u(q_l.rho(),q_l.y(), u_max_l, gamma), eq::u(q_m_l.rho(),q_m_l.y(), u_max_l, gamma), eq::u(q_m_r.rho(),q_m_r.y(), u_max_r, gamma), eq::u(q_r.rho(),q_r.y(), u_max_r, gamma));
//     printf("u:        %8.5f       %8.5f       %8.5f       %8.5f\n", q_l.u(),  q_m_l.u(), q_m_r.u(),    q_r.u()  );
//     printf("lambda_0: %8.5f       %8.5f       %8.5f       %8.5f\n",
//            q_l.lambda_0(u_max_l, gamma),
//            q_l.lambda_0(u_max_l, gamma),
//            q_l.lambda_0(u_max_r, gamma),
//            q_l.lambda_0(u_max_r, gamma));

//     printf("flux_0:                %8.5f       %8.5f\n",
//            q_m_l.flux_0(),
//            q_m_r.flux_0());

//     printf("flux_1:                %8.5f       %8.5f\n",
//            q_m_l.flux_1(),
//            q_m_r.flux_1());

    clear();

    const T rho_diff = q_m_l.rho() - q_l.rho();
    speeds[0]        = std::abs(rho_diff) < 100*epsilon() ? 0.0 : (q_m_l.flux_0() - q_l.flux_0())/rho_diff;
    waves [0]        = q_m_l - q_l;

    speeds[1]           = q_r.u();
    waves [1]           = q_r - q_m_r;

    left_fluctuation  = q_m_l.flux() - q_l.flux();
    right_fluctuation = q_r.flux()   - q_m_r.flux();
    q_0               = arz<float>::q(q_m_r.rho(), q_m_r.y()); // choice of right value is (fairly) arbitrary
};

template <typename T>
inline void arz<T>::riemann_solution::starvation_riemann(const full_q &restrict q_r,
                                                         const T                    u_max,
                                                         const T                    inv_u_max)
{
    if(q_r.rho() < VACUUM_EPS)
    {
        clear();
        return;
    }

    speeds[0]         = 0.0;
    waves [0]         = q(0.0, 0.0);
    speeds[1]         = q_r.u();
    waves [1]         = q_r;
    left_fluctuation  = q(0.0, 0.0);
    right_fluctuation = q_r.flux();
    q_0               = arz<float>::q(q_r.rho(), q_r.y());
}

template <typename T>
inline void arz<T>::riemann_solution::stop_riemann(const full_q &restrict q_l,
                                                   const T                    u_max,
                                                   const T                    inv_u_max)
{
    full_q q_m;
    q_m.u_eq() = q_l.u_eq() - q_l.u();
    q_m.u()    = 0.0;
    q_m.rho()  = eq::inv_u_eq(q_m.u_eq(), inv_u_max);
    q_m.y()    = -q_m.rho()*q_m.u_eq();

    const T rho_diff  = q_m.rho() - q_l.rho();
    speeds[0]         = rho_diff < VACUUM_EPS ? 0.0 : -q_l.flux_0()/rho_diff;
    waves [0]         = -q_l;
    speeds[1]         = 0.0;
    waves [1]         = q(0.0, 0.0);
    left_fluctuation  = -q_l.flux();
    right_fluctuation = q(0.0, 0.0);
    q_0               = arz<float>::q(0, 0);
}

template <typename T>
inline void arz<T>::riemann_solution::clear()
{
    memset(this, 0, sizeof(*this));
}

template <typename T>
inline bool arz<T>::riemann_solution::check() const
{
    return xisfinite(waves[0][0])
    && xisfinite(waves[0][1])
    && xisfinite(waves[1][0])
    && xisfinite(waves[1][1])
    && xisfinite(left_fluctuation[0])
    && xisfinite(left_fluctuation[1])
    && xisfinite(right_fluctuation[0])
    && xisfinite(right_fluctuation[1])
    && xisfinite(speeds[0]) && xisfinite(speeds[1]);
}

#endif
