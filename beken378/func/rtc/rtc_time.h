#ifndef __TIME_H__
#define __TIME_H__

//#define RT_TICK_PER_SECOND		1000

typedef long long time_t_at;

/*
 * Structure returned by gettimeofday(2) system call,
 * and used in other calls.
 */
struct timeval_at {
  long	tv_sec;		/* seconds */
  long	tv_usec;	/* and microseconds */
};

/*
 * Structure defined by POSIX.1b to be like a timeval.
 */
struct timespec_at {
  long	tv_sec;		/* seconds */
  long	tv_nsec;	/* and nanoseconds */
};

struct timezone_at {
  int tz_minuteswest;	/* minutes west of Greenwich */
  int tz_dsttime;	/* type of dst correction */
};

struct tm_at {
  int tm_sec;			/* Seconds.	[0-60] (1 leap second) */
  int tm_min;			/* Minutes.	[0-59] */
  int tm_hour;			/* Hours.	[0-23] */
  int tm_mday;			/* Day.		[1-31] */
  int tm_mon;			/* Month.	[0-11] */
  int tm_year;			/* Year - 1900. */
  int tm_wday;			/* Day of week.	[0-6] */
  int tm_yday;			/* Days in year.[0-365]	*/
  int tm_isdst;			/* DST.		[-1/0/1]*/

  long int tm_gmtoff;		/* Seconds east of UTC.  */
  const char *tm_zone;		/* Timezone abbreviation.  */
};

int gettimeofday_at(struct timeval_at *tp, void *ignore);
int gettimeofday_ms_at(void);

time_t_at mktime_at(struct tm_at * const t);

char *asctime_at(const struct tm_at *timeptr);
char *ctime_at(const time_t_at *timep);
struct tm_at *localtime_at(const time_t_at* t);

char *asctime_r_at(const struct tm_at *t, char *buf);
struct tm_at* gmtime_r_at(const time_t_at *timep, struct tm_at *r);
struct tm_at* localtime_r_at(const time_t_at* t, struct tm_at* r);

time_t_at time_at(time_t_at *t);

#endif
