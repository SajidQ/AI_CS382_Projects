#include "gaussian.hpp"

gaussian_t gaussian;

gaussian_t alloc_gaussian()
{
    gaussian_t g = (gaussian_t)malloc(sizeof(gaussian_struct_t));
    g->gauss_random = 0;
    g->available_gauss = 0;

    g->prev_mean = -100000;
    g->prev_variance = -10000;
    return g;
}

double gaussian_random( gaussian_t g, double m, double v )
{
    if( g->available_gauss == false || m != g->prev_mean || v != g->prev_variance )
    {
      double r1, r2, rsq, fac;
      
      do
      {
          r1 = uniform_random( -1, 1);
          r2 = uniform_random( -1, 1);
          rsq = r1*r1 + r2*r2;
      }
      while( rsq >= 1.0 || rsq == 0.0 );

      fac = sqrt ((-2.0 * log (rsq))/rsq);
      
      g->gauss_random = (r2*fac)*sqrt(v) + m;

      g->available_gauss = true;
      g->prev_mean = m;
      g->prev_variance = v;
      
      return (r1*fac)*sqrt(v) + m;
    }
    else
    {
         g->available_gauss = false;
        return g->gauss_random;
    }
}



int main( int* argc, char** argv )
{

// if you are coding on a linux-based system
// and you have access to /dev/random then use the
// following lines to set the seed for the random
// number generator
    generate_seed();

// otherwise set manually a seed or use your own preferred 
// way of initializing the random number generator
//    use_seed( 555 );

    gaussian = alloc_gaussian();

    int N = 100;
    double x[N];
    double sum = 0;

    for( int i=0; i<N; i++ )
    {
	x[i] = gaussian_random( gaussian, 0.0, 4.0 );
	sum += x[i];
    }

    printf("Random number drawn from a gaussian distribution with mean value 0 and standard deviation equal to 2.0\n");
    for( int i=0; i<N; i++ )
	printf("%3.12f\n",x[i]);

    printf("The mean of the sampled population is: %f\n",sum/(double)N);
    
    free( gaussian );

    return 0;
}
