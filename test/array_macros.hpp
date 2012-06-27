#ifndef _ARRAY_MACROS_HPP__
#define _ARRAY_MACROS_HPP__

#include <algorithm>
#include <cstring>

#define DECLARE_ARRAY_ALL(type, name)           \
    int name##_n;                               \
    int name##_n_allocd;                        \
    type* name

#define DECLARE_ARRAY(type, name)               \
    int name##_n;                               \
    type* name

#define INIT_ARRAY(name, size, n_allocd)        \
    name##_n = 0;                               \
    n_allocd = size;                            \
    name     = (typeof(name)) malloc(sizeof(name[0])*n_allocd);

#define EXTEND_ARRAY(name, num, n_allocd)       \
    if(name##_n + num >= n_allocd)              \
    {                                           \
        n_allocd = (name##_n + num)*2;            \
        void *m  = realloc(name, sizeof(name[0])*n_allocd); \
        name     = (typeof(name)) m;            \
    }

#define SIZE_ARRAY(name, size, n_allocd)       \
    if(size >= n_allocd)              \
    {                                           \
        n_allocd = (size)*2;            \
        void *m  = realloc(name, sizeof(name[0])*n_allocd); \
        name     = (typeof(name)) m;            \
    }

#define RESERVE_ARRAY(name, size, n_allocd)     \
    if(size >= n_allocd)                        \
    {                                           \
        n_allocd = size;                        \
        void *m  = realloc(name, sizeof(name[0])*n_allocd); \
        name     = (typeof(name)) m;            \
    }

#define RESIZE_ARRAY(name, size, n_allocd)      \
    name##_n    = size;                         \
    if(name##_n >= n_allocd)                    \
    {                                           \
        n_allocd  = name##_n*2;                 \
        void *m = realloc(name, sizeof(name[0])*n_allocd); \
        name    = (typeof(name)) m;             \
    }

#define FIT_ARRAY(name, n_allocd)               \
    if(name##_n < n_allocd)                     \
    {                                           \
        n_allocd = name##_n;                    \
        void *m  = realloc(name, sizeof(name[0])*n_allocd); \
        name     = (typeof(name)) m;            \
    }

#define COPY_ARRAY(dest, dest_n_allocd, src, src_n_allocd) \
    {                                           \
        dest##_n      = src##_n;                \
        dest_n_allocd = src_n_allocd;           \
        void *m       = realloc(dest, sizeof(dest[0])*dest_n_allocd); \
        dest          = (typeof(dest)) m;       \
        memcpy(dest, src, sizeof(dest[0])*dest##_n); \
    }

#define FREE_ARRAY_ALL(name)                    \
    name##_n        = 0;                        \
    name##_n_allocd = 0;                        \
    free(name);                                 \
    name            = 0;

#define FREE_ARRAY(name)                        \
    name##_n = 0;                               \
    free(name);                                 \
    name     = 0;
#endif
