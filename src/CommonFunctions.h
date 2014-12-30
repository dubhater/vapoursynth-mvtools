#ifndef __COMMON_F__
#define __COMMON_F__

// returns a > 0 ? a : 0
inline static int satz(int a)
{
    return ~(a >> (sizeof(int)*8 - 1)) & a;
}

// returns maximum(a, b)
inline static int imax(int a, int b)
{
    return a + satz(b - a);
}

// returns minimum(a, b)
inline static int imin(int a, int b)
{
    return a - satz(a - b);
}

/* returns the biggest integer x such as 2^x <= i */
inline static int ilog2(int i)
{
    int result = 0;
    while ( i > 1 ) { i /= 2; result++; }
    return result;
}

/* computes 2^i */
inline static int iexp2(int i)
{
    return 1 << satz(i);
    //     int result = 1;
    //     while ( i > 0 ) { result *= 2; i--; }
    //     return result;
}

// general common divisor (from wikipedia)
inline static int64_t gcd(int64_t u, int64_t v)
{
    int shift;

    /* GCD(0,x) := x */
    if (u == 0 || v == 0)
        return u | v;

    /* Let shift := lg K, where K is the greatest power of 2
       dividing both u and v. */
    for (shift = 0; ((u | v) & 1) == 0; ++shift) {
        u >>= 1;
        v >>= 1;
    }

    while ((u & 1) == 0)
        u >>= 1;

    /* From here on, u is always odd. */
    do {
        while ((v & 1) == 0)  /* Loop X */
            v >>= 1;

        /* Now u and v are both odd, so diff(u, v) is even.
           Let u = min(u, v), v = diff(u, v)/2. */
        if (u < v) {
            v -= u;
        } else {
            int64_t diff = u - v;
            u = v;
            v = diff;
        }
        v >>= 1;
    } while (v != 0);

    return u << shift;
}

#endif
