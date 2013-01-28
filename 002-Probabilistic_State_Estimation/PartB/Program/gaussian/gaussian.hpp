#ifndef GAUSSIAN_HPP
#define GAUSSIAN_HPP

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <iostream>
#include <strings.h>
#include <unistd.h>
#include <pwd.h>
#include <sys/types.h>
#include <cmath>

using namespace std;

typedef struct
{
    double gauss_random;
    int    available_gauss;

    double prev_mean;
    double prev_variance;
} gaussian_struct_t;
typedef gaussian_struct_t* gaussian_t;

extern gaussian_t gaussian;

gaussian_t alloc_gaussian();

// return a number drawn from the gaussian distribution with 
// mean value "m" and variance "v"
double gaussian_random( gaussian_t g, double m, double v );

/**************************************
** AUXILIARY FUNCTIONS
**************************************/

#define use_seed( seed )\
{\
    srand(seed);\
}

static inline
int read_seed()
{
    int seed;
    FILE *fp = fopen("/dev/random", "r");
    fread(&seed, sizeof(int), 1, fp);
    fclose(fp);
    return seed;
}

static inline
int generate_seed()
{
    int seed = read_seed();
    use_seed( seed );
    return seed;
}

static inline
int generate_seed_verbose()
{
    int seed = generate_seed();
    fprintf(stderr, "seed: %x\n", seed);
    return seed;
}

static inline
double uniform_random()
{
    return ( (double)rand() / (double)RAND_MAX );
}

static inline
double uniform_random( double min, double max )
{
    return ( uniform_random() * ( max - min ) ) + min;
}

static inline
int uniform_int_random( int min, int max )
{
    return (int)( round( uniform_random( (double)min - 0.499999, (double)max + 0.499999 ) ));
}

#endif
