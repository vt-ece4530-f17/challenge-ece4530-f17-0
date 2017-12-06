#include <stdio.h>
#include <sys/stat.h> 
#include <fcntl.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include <assert.h>
#include <string.h>

#define _GNU_SOURCE
#include <sys/types.h>
#include <sys/syscall.h>
#include <sys/mman.h>
#include <linux/perf_event.h>
#include "hps_0.h"
#include "socal/hps.h"

static void hwcpucyclesinit(volatile unsigned *timerbase) {
  timerbase[5] = 0x7FFF;
  timerbase[4] = 0xFFFF;
  timerbase[3] = 0xFFFF;
  timerbase[2] = 0xFFFF;
  timerbase[1] = timerbase[1] | 0x4;  // start timer
}

static long long hwcpucycles(volatile unsigned *timerbase) {
  unsigned long long snap;
  timerbase[9] = 0; // take snapshot
  snap = timerbase[9];
  snap = (snap << 16) | timerbase[8];
  snap = (snap << 16) | timerbase[7];
  snap = (snap << 16) | timerbase[6];
  return (0x7FFFFFFFFFFFFFFFLL - snap);
}

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

typedef signed char           sample_t;
typedef volatile signed char *sample_hptr;
typedef unsigned              index_t;

sample_hptr bulk; // [N1*Q1*S1];

#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )

//----- sample function to be timed
void testaccess() {
  unsigned i;
  for (i = 0; i<128*1024; i++)
    bulk[i] = bulk[i] + 1;
}

int main() {

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

  void *peripherals;
  volatile unsigned  *timerbase;
  peripherals = mmap( NULL,
		      HW_REGS_SPAN,
		      ( PROT_READ | PROT_WRITE ),
		      MAP_SHARED,
		      memfd,
		      HW_REGS_BASE );
  if (peripherals == MAP_FAILED) {
    printf("ERROR: mmap() failed ...\n");
    return 1;
  }

  // timer intialization
  timerbase = peripherals +
    ( ( unsigned )( ALT_LWFPGASLVS_OFST + TIMER_0_BASE) &
      ( unsigned )( HW_REGS_MASK ) );
  hwcpucyclesinit(timerbase);
  
  // timer usage
  unsigned long long ref_cycles[TIMINGMEASUREMENTS];
  unsigned timingloop;
  for (timingloop=0; timingloop<TIMINGMEASUREMENTS; timingloop++) {
    ref_cycles[timingloop] = hwcpucycles(timerbase);
    testaccess();    
    ref_cycles[timingloop] = hwcpucycles(timerbase) - ref_cycles[timingloop];
    printf("Iteration %d: %lld cycles\n", timingloop, ref_cycles[timingloop]);
  }
    
  printf("Reference Execution time %lld\n", median(ref_cycles));

  if ( munmap( (void *) bulk, ONCHIP_MEMORY2_0_SPAN ) != 0 ) {
    printf( "ERROR: munmap() failed...\n" );
    close( memfd );
    return( 1 );
  }
  close( memfd );
  
  return 0;
}
