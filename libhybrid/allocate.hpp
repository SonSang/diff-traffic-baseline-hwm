#ifndef _ALLOCATE_HPP_
#define _ALLOCATE_HPP_

#include <cstdlib>
#include <cstdio>
#include <cassert>

#define CACHE_LINE 64
#define FLOATS_PER_CACHE_LINE (CACHE_LINE/sizeof(float))
#define MAXES_STRIDE FLOATS_PER_CACHE_LINE

#ifdef _MSC_VER
inline void *aligned_malloc(size_t size, size_t align_size)
{
    char *ptr,*ptr2,*aligned_ptr;
    int align_mask = align_size - 1;

    ptr=(char *)malloc(size + align_size + sizeof(int));
    if(ptr==NULL) return(NULL);

    ptr2 = ptr + sizeof(int);
    aligned_ptr = ptr2 + (align_size - ((size_t)ptr2 & align_mask));

    ptr2 = aligned_ptr - sizeof(int);
    *((int *)ptr2)=(int)(aligned_ptr - ptr);

    return(aligned_ptr);
}

inline void *xmalloc(size_t bytes)
{
    void *ptr = aligned_malloc(bytes, CACHE_LINE);
    if(ptr == NULL)
    {
        fprintf(stderr, "memalign failed\n!");
        exit(1);
    }
    return ptr;
}
#else
inline void *xmalloc(size_t bytes)
{
    size_t mod = bytes % CACHE_LINE;
    if(mod != 0)
        bytes += CACHE_LINE-mod;
    assert((bytes % CACHE_LINE) == 0);
    void *ptr;
    int res = posix_memalign(&ptr, CACHE_LINE, bytes);
    if(res)
    {
        fprintf(stderr, "memalign failed\n!");
        exit(1);
    }

    return ptr;
}
#endif
#endif
