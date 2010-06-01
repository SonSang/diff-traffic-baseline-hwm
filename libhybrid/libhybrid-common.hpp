#ifndef _LIBHYBRID_COMMON_HPP_
#define _LIBHYBRID_COMMON_HPP_

const char *libhybrid_package_string();

#ifdef _MSC_VER
#define restrict
#define xisfinite _finite
#else
#define restrict __restrict__
#define xisfinite std::isfinite
#endif

#endif
