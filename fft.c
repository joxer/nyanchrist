#include <fftw3.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <pulse/simple.h>
#include <pulse/error.h>
#include <math.h>
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#define BUFSIZE 8192
int main(int argc, char ** argv)
{
  fftw_complex *in, *out;
  static const pa_sample_spec ss = {
    .format = PA_SAMPLE_U8,
    .rate = 44100,
        .channels = 1
  };
  int error;
  in = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * BUFSIZE);
  out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * BUFSIZE);
  pa_simple * s = pa_simple_new(NULL, argv[0], PA_STREAM_RECORD, NULL, "record", &ss, NULL, NULL, &error);
  unsigned char buf[BUFSIZE];
  int fd_serial = open(argv[1], O_RDWR | O_NOCTTY | O_NDELAY );
  fftw_plan p = fftw_plan_dft_1d(BUFSIZE, in, out, FFTW_FORWARD, FFTW_ESTIMATE);
  struct termios settings;
  tcgetattr(fd_serial,&settings);
  cfsetospeed(&settings,9600);
  tcsetattr(fd_serial,TCSANOW,&settings);
  tcflush(fd_serial,TCOFLUSH);
    
  while ( 1 )
    {
      pa_simple_read(s,buf,BUFSIZE,&error);
      int i = 0;
      for ( i = 0; i < BUFSIZE; i++ )
	{
	  in[i][0] = buf[i];
	  in[i][1] = 0.0;
      
	}
      fftw_execute(p);
      char n1,n2,n3;
    
      float avg1 = 0.0;
   
      for ( i = 0; i < BUFSIZE/40; i++ )
	{
	  if ( i < BUFSIZE/80 )
	    continue;
	  avg1 += pow((out[i][0]*out[i][0]+out[i][1]*out[i][1]),2);
	}
      avg1 /= (float)(BUFSIZE/40);
      float avg2 = 0.0;
      for ( i = BUFSIZE/40; i < BUFSIZE/40+BUFSIZE/60; i++ )
	{
	  avg2 += pow((out[i][0]*out[i][0]+out[i][1]*out[i][1]),2);
	}
      avg2 /= (float)(BUFSIZE/60);
      float avg3 = 0.0;
      for ( i =  BUFSIZE/40+BUFSIZE/60; i <  BUFSIZE/40+BUFSIZE/60+BUFSIZE/60; i++ )
	{
	  avg3 +=     pow((out[i][0]*out[i][0]+out[i][1]*out[i][1]),2);
	}
      avg3 /= (float)(BUFSIZE/60);
      float avgmax = avg1+avg2+avg3;
      avg1 /= avgmax;avg2 /= avgmax;avg3 /= avgmax;

      printf("%f %f %f\n",avg1,avg2,avg3);
      char byte = avg1*254;
      write(fd_serial,&byte,1);
      byte = avg2*254;
      write(fd_serial,&byte,1);
      byte = avg3*254;
      write(fd_serial,&byte,1);
    
    }
  return 0;
}
