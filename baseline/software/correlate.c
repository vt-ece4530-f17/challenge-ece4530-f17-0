#include <stdio.h>
#include <sys/stat.h> 
#include <fcntl.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include <assert.h>
#include <string.h>

#ifndef NODE1SOC
#define _GNU_SOURCE
#include <sys/types.h>
#include <sys/syscall.h>
#include <sys/mman.h>
#include <linux/perf_event.h>
#include "hps_0.h"
#include "socal/hps.h"

static int fddev = -1;
__attribute__((constructor)) static void init(void) {
        static struct perf_event_attr attr;
        attr.type = PERF_TYPE_HARDWARE;
        attr.config = PERF_COUNT_HW_CPU_CYCLES;
        fddev = syscall(__NR_perf_event_open, &attr, 0, -1, -1, 0);
}

__attribute__((destructor)) static void fini(void) {
  close(fddev);
}

static inline long long cpucycles(void) {
  long long result = 0;
  if (read(fddev, &result, sizeof(result)) < sizeof(result)) return 0;
  return result;
}
#endif

typedef signed char           sample_t;
typedef volatile signed char *sample_hptr;
typedef unsigned              index_t;

// reseeds the PRNG with a true random number
void initrng() {
  unsigned s;
  int f = open("/dev/random", O_RDONLY);
  read(f, &s, sizeof(s));
  srand(s);
  close(f);
}

// generates a sine wave v[] of n samples with double precision
void gensin(double *v, index_t n) {
  index_t i;
  assert(n > 0);
  for (i=0; i<n; i++) 
    v[i] = sin(2.0 * M_PI * i/n);
}

// generates a noise form v[] of n samples,
// amplitude -level to +level
void gennoise(double *v,
	      index_t n,
	      double level) {
  index_t i;
  for (i=0; i<n; i++)
    v[i] = level * (2.0 * rand() /  INT32_MAX - 1.0);
}

// creates a search waveform of s samples in sig
//   - noise level level
//   - template inserted at pos
void buildsignal(double *sig, index_t s,        // output buffer
		 double level,                  // noise level
		 double *template, index_t t,   // template to insert
		 index_t pos                    // position of template insertion
		 ) {
  index_t i;
  gennoise(sig, s, level);                 // build noisy signal
  for (i=0; i<t; i++)
    sig[(i + pos) % s] += template[i];     // insert template
}

// finds max in sig[] of s samples
index_t findmax(double *sig, index_t s) {
  index_t i, max = 0;
  for (i=0; i<s; i++) {
    if (fabs(sig[i]) > fabs(sig[max]))
      max = i;
  }
  return max;
}

// scales sig[] so that +max or -max of sig equals +(1 << precision)-1
// or -(1 << precision) and stores the result in v[]
void scalesig(double *sig, index_t s,
	      unsigned precision,
	      sample_hptr v) {
  
  index_t m = findmax(sig, s);
  unsigned i;
  double scale;

  if (sig[m] > 0)
    scale = ((1 << precision) - 1) * 1.0 / sig[m];
  else
    scale = -(1 << precision) * 1.0 / sig[m];
  
  for (i=0; i<s; i++)
    v[i] = floor(sig[i] * scale);  
}

// saves an int signal of s samples in file name
void samplesave(sample_hptr sig, index_t s,
		char *name) {
  index_t i;
  FILE *f = fopen(name, "w");
  for (i=0; i<s; i++)
    fprintf(f,"%d\n", sig[i]);
  fclose(f);
}

// saves an index

//================================================================================================
//================================================================================================
//
// main computation: this is reference functionality
//
void  bulkcorrelate(sample_hptr sig,
		    index_t     waveforms,        // total number of waveforms
		    index_t     samples,          // samples per waveform
		    sample_hptr template,         // search template
		    index_t     templatesamples,  // samples in search template
		    index_t    *c                 // store max index of correlation of each waveform
		    ) {
  index_t i, j, k;
  int     acc;
  int     max;
  index_t maxindex;

  for (i=0; i < waveforms; i++) {
    max      = 0;
    maxindex = 0;
    for (j=0; j < samples; j++) {
      acc = 0;
      for (k=0; k < templatesamples; k++) {
	acc += sig[i * samples + (j + k) % samples] * template[k];
      }      
      if (acc > max) {
	max = acc;
	maxindex = j;
      }
    }
    c[i] = maxindex;  // max correlation index
  }
  
#ifdef NODE1SOC
  {
    FILE *f = fopen("wave_corr.txt", "w");
    for (i=0; i<waveforms; i++)
      fprintf(f, "%d\n", c[i]);
    fclose(f);
  }
#endif      

}

//================================================================================================
//================================================================================================
//
// accelerated computation: this is the function that needs to be accelerated using hardware
//
// You are allowed to transform and change the function body of
// hw_bulkcorrelate as you desire, but the API MUST remain unchanged.
// Also, during development, you are allowed to make arbitrary changes
// to the main program (such as changing the T1, S1, Q1, N1 macro
// parameters below).  However, the final version MUST be tested
// against the original main program.
//
void  hw_bulkcorrelate(sample_hptr sig,
		       index_t     waveforms,       // total number of waveforms
		       index_t     samples,         // samples per waveform
		       sample_hptr template,        // search template
		       index_t     templatesamples, // samples in search template
		       index_t     *c               // store max index of correlation of each waveform
		       ) {
  bulkcorrelate(sig,
		waveforms,
		samples,
		template,
		templatesamples,
		c);
}  
//================================================================================================
//================================================================================================

#ifndef NODE1SOC

#define TIMINGMEASUREMENTS 5

int compare_unsigned(const void *a, const void *b) {
  const unsigned long long *da = (const unsigned long long *) a;
  const unsigned long long *db = (const unsigned long long *) b;
  return (*da > *db) - (*da < *db);
}

unsigned long long median(unsigned long long *thist) {
  qsort(thist, TIMINGMEASUREMENTS, sizeof(unsigned long long), compare_unsigned);
  return thist[TIMINGMEASUREMENTS >> 1];
}

#endif

//-----------------------------------------------------------------
// T1 - reference search template is 16 samples
// S1 - each search waveform holds 128 samples
// Q1 - processing batch size is 128 waveforms per noise level
// N1 - number of noise levels to test is 8
#define T1 16
#define S1 128
#define Q1 128
#define N1 8
#define NOISELEVEL(A) (0.125 + 0.125 * A)

#ifdef NODE1SOC
// bulk storage in main memory
sample_t    bulk[N1*Q1*S1];
#else
// bulk storage in FPGA on AXI bus
sample_hptr bulk; // [N1*Q1*S1];
#endif

int main() {
  index_t  i, j;
  double   wave       [T1];
  double   sig        [S1];
  sample_t isig       [S1];

  sample_t intwave    [T1];  
  index_t  ref_intmax [N1*Q1];
  index_t  acc_intmax [N1*Q1];

#ifndef NODE1SOC  
  int memfd;  
  if( ( memfd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) {
    printf( "ERROR: could not open \"/dev/mem\"...\n" );
    return( 1 );
  }
  
  bulk = mmap( NULL, 
	       ONCHIP_MEMORY2_0_SPAN, 
	       ( PROT_READ | PROT_WRITE ), 
	       MAP_SHARED, 
	       memfd, 
	       0xC0000000 + ONCHIP_MEMORY2_0_BASE );
  
  if( bulk == MAP_FAILED ) {
    printf( "ERROR: mmap() failed...\n" );
    close( memfd );
    return(1);
  }
#endif
  
  unsigned TARGET = 60;
  
  // Prepare the signal
  
  // - reseed the random number generator
  initrng();
  
  // - generate a template waveform
  gensin (wave, T1);
  
  // - scale it to 8 bit (7 bit 2-c)  
  scalesig    (wave, T1, 7, intwave);

#ifdef NODE1SOC
  samplesave(intwave, T1, "wave_ref.txt");
#endif
  
  // - build signal
  for (i=0; i<N1; i++) {   // for every noise level
    for (j=0; j<Q1; j++) {     // for all waveforms at this noise level
      
      buildsignal (sig,           // waveform
		   S1,            // number of samples in waveform
		   NOISELEVEL(i), // noise level
		   wave,          // target waveform to hide
		   T1,            // number of samples in target waveform
		   TARGET         // target insertion index
		   );
      
      scalesig    (sig,
		   S1,            // number of samples in waveform
		   7,             // 8 bit = 7 bit 2-c
		   isig
		   );
      
      memcpy((void *) (bulk + (j + i * Q1) * S1),   // store it in bulk memory
	     isig,
	     sizeof(char) * S1);
#ifdef NODE1SOC
  // on x86, save sample waveforms
      {
	char buf[64];
	if (j == 0) {
	  snprintf(buf,64,"wave_level_%d.txt", i);
	  samplesave(isig, S1, buf);
	}
      }
#endif      
    }
  }
  
  //------------------------------ reference computation
  
#ifndef NODE1SOC
  unsigned long long ref_cycles[TIMINGMEASUREMENTS];
  unsigned timingloop;

  for (timingloop=0; timingloop<TIMINGMEASUREMENTS; timingloop++) {
    ref_cycles[timingloop] = cpucycles();
#endif
  
    // reference computation
    bulkcorrelate(bulk,        // waveform[num][samples]
		  N1*Q1,       // N1*Q1 waveforms
		  S1,          // S1 samples per waveform
		  intwave,     // search template
		  T1,          // of T1 samples
		  ref_intmax   // max index found 
		  );
    
#ifndef NODE1SOC
    ref_cycles[timingloop] = cpucycles() - ref_cycles[timingloop];
  }
#endif
  
  // error analysis
  printf("REFERENCE\n");
  for (i=0; i<N1; i++) {
    unsigned errorlevel = 0;
    for (j=0; j<Q1; j++)
      errorlevel += abs(ref_intmax[i*Q1 + j] - TARGET);
    printf("Noise %4.3f error %4d\n", NOISELEVEL(i), errorlevel);
  }

  //------------------------------ accelerated computation
  
#ifndef NODE1SOC
  unsigned long long acc_cycles[TIMINGMEASUREMENTS];
  
  for (timingloop=0; timingloop<TIMINGMEASUREMENTS; timingloop++) {
    acc_cycles[timingloop] = cpucycles();
#endif
    
    // accelerated computation
    hw_bulkcorrelate(bulk,        // waveform[num][samples]
		     N1*Q1,       // N1*Q1 waveforms
		     S1,          // S1 samples per waveform
		     intwave,     // search template
		     T1,          // of T1 samples
		     acc_intmax   // max index found 
		     );
    
#ifndef NODE1SOC
    acc_cycles[timingloop] = cpucycles() - acc_cycles[timingloop];
  }
#endif

  unsigned totalerror = 0;
  
  // error analysis  
  printf("ACCELERATED\n");
  for (i=0; i<N1; i++) {
    unsigned errorlevel_ref = 0;
    unsigned errorlevel_acc = 0;
    for (j=0; j<Q1; j++) {
      errorlevel_ref += abs(ref_intmax[i*Q1 + j] - TARGET);
      errorlevel_acc += abs(acc_intmax[i*Q1 + j] - TARGET);
    }
    printf("Noise %4.3f error %4d delta %4d\n",
	   NOISELEVEL(i),
	   errorlevel_acc,
	   abs(errorlevel_ref - errorlevel_acc));
    totalerror +=  abs(errorlevel_ref - errorlevel_acc);
  }
  
  printf("Total Error: %d\n", totalerror);
  if (totalerror <1024) 
    printf("Testbench passes!\n");    
  else
    printf("Testbench fails!\n");    
    
#ifndef NODE1SOC
  printf("Reference Execution time %lld\n", median(ref_cycles));
  printf("Accelerated Execution time %lld\n", median(acc_cycles));

  if ( munmap( (void *) bulk, ONCHIP_MEMORY2_0_SPAN ) != 0 ) {
    printf( "ERROR: munmap() failed...\n" );
    close( memfd );
    return( 1 );
  }
  close( memfd );
#endif
  
  return 0;
}
