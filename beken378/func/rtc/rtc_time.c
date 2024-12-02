#include "rtc_time.h"
#include "rtc.h"
#include <string.h>
#include "drv_model_pub.h"

/* days per month -- nonleap! */
const short __spm[13] =
  { 0,
    (31),
    (31+28),
    (31+28+31),
    (31+28+31+30),
    (31+28+31+30+31),
    (31+28+31+30+31+30),
    (31+28+31+30+31+30+31),
    (31+28+31+30+31+30+31+31),
    (31+28+31+30+31+30+31+31+30),
    (31+28+31+30+31+30+31+31+30+31),
    (31+28+31+30+31+30+31+31+30+31+30),
    (31+28+31+30+31+30+31+31+30+31+30+31),
  };
static long int timezone;
static const char days[] = "Sun Mon Tue Wed Thu Fri Sat ";
static const char months[] = "Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec ";

/* seconds per day */
#define SPD 24*60*60

int __isleap(int year)
{
	/* every fourth year is a leap year except for century years that are
	 * not divisible by 400. */
	/*  return (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)); */
	return (!(year % 4) && ((year % 100) || !(year % 400)));
}

struct tm_at* gmtime_r_at(const time_t_at *timep, struct tm_at *r)
{
	time_t_at i;
	register time_t_at work = *timep % (SPD);
	r->tm_sec = work % 60;
	work /= 60;
	r->tm_min = work % 60;
	r->tm_hour = work / 60;
	work = *timep / (SPD);
	r->tm_wday = (4 + work) % 7;
	for (i = 1970;; ++i)
	{
		register time_t_at k = __isleap(i) ? 366 : 365;
		if (work >= k)
			work -= k;
		else
			break;
	}
	r->tm_year = i - 1900;
	r->tm_yday = work;

	r->tm_mday = 1;
	if (__isleap(i) && (work > 58))
	{
		if (work == 59)
			r->tm_mday = 2; /* 29.2. */
		work -= 1;
	}

	for (i = 11; i && (__spm[i] > work); --i)
		;
	r->tm_mon = i;
	r->tm_mday += work - __spm[i];
	return r;
}

struct tm_at* localtime_r_at(const time_t_at* t, struct tm_at* r)
{
	time_t_at tmp;
	struct timezone_at tz = {0};
	gettimeofday_at(0, &tz);
	timezone = tz.tz_minuteswest * 60L;
	tmp = *t + timezone;
	return gmtime_r_at(&tmp, r);
}

struct tm_at* localtime_at(const time_t_at* t)
{
	static struct tm_at tmp;
	return localtime_r_at(t, &tmp);
}

time_t_at mktime_at(struct tm_at * const t)
{
	register time_t_at day;
	register time_t_at i;
	register time_t_at years = t->tm_year - 70;

	if (t->tm_sec > 60)
	{
		t->tm_min += t->tm_sec / 60;
		t->tm_sec %= 60;
	}
	if (t->tm_min > 60)
	{
		t->tm_hour += t->tm_min / 60;
		t->tm_min %= 60;
	}
	if (t->tm_hour > 24)
	{
		t->tm_mday += t->tm_hour / 24;
		t->tm_hour %= 24;
	}
	if (t->tm_mon > 12)
	{
		t->tm_year += t->tm_mon / 12;
		t->tm_mon %= 12;
	}
	while (t->tm_mday > __spm[1 + t->tm_mon])
	{
		if (t->tm_mon == 1 && __isleap(t->tm_year + 1900))
		{
			--t->tm_mday;
		}
		t->tm_mday -= __spm[t->tm_mon];
		++t->tm_mon;
		if (t->tm_mon > 11)
		{
			t->tm_mon = 0;
			++t->tm_year;
		}
	}

	if (t->tm_year < 70)
		return (time_t_at) -1;

	/* Days since 1970 is 365 * number of years + number of leap years since 1970 */
	day = years * 365 + (years + 1) / 4;

	/* After 2100 we have to substract 3 leap years for every 400 years
	 This is not intuitive. Most mktime implementations do not support
	 dates after 2059, anyway, so we might leave this out for it's
	 bloat. */
	if (years >= 131)
	{
		years -= 131;
		years /= 100;
		day -= (years >> 2) * 3 + 1;
		if ((years &= 3) == 3)
			years--;
		day -= years;
	}

	day += t->tm_yday = __spm[t->tm_mon] + t->tm_mday - 1 +
			(__isleap(t->tm_year + 1900) & (t->tm_mon > 1));

	/* day is now the number of days since 'Jan 1 1970' */
	i = 7;
	t->tm_wday = (day + 4) % i; /* Sunday=0, Monday=1, ..., Saturday=6 */

	i = 24;
	day *= i;
	i = 60;
	return ((day + t->tm_hour) * i + t->tm_min) * i + t->tm_sec;
}

static void num2str(char *c, int i)
{
	c[0] = i / 10 + '0';
	c[1] = i % 10 + '0';
}

char *asctime_r_at(const struct tm_at *t, char *buf)
{
	/* "Wed Jun 30 21:49:08 1993\n" */
	*(int*) buf = *(int*) (days + (t->tm_wday << 2));
	*(int*) (buf + 4) = *(int*) (months + (t->tm_mon << 2));
	num2str(buf + 8, t->tm_mday);
	if (buf[8] == '0')
		buf[8] = ' ';
	buf[10] = ' ';
	num2str(buf + 11, t->tm_hour);
	buf[13] = ':';
	num2str(buf + 14, t->tm_min);
	buf[16] = ':';
	num2str(buf + 17, t->tm_sec);
	buf[19] = ' ';
	num2str(buf + 20, (t->tm_year + 1900) / 100);
	num2str(buf + 22, (t->tm_year + 1900) % 100);
	buf[24] = '\n';
	return buf;
}

char *asctime_at(const struct tm_at *timeptr)
{
	static char buf[25];
	return asctime_r_at(timeptr, buf);
}

char *ctime_at(const time_t_at *timep)
{
	return asctime_at(localtime_at(timep));
}

int gettimeofday_ms_at(void)
{
	time_t_at time = 0;
	sddev_control(SOFT_RTC_DEVICE_NAME,RT_DEVICE_CTRL_RTC_GET_MS,&time);
	return time;
}

int gettimeofday_at(struct timeval_at *tp, void *ignore)
{
	time_t_at time;
	UINT32 status = 0;
	status = sddev_control(SOFT_RTC_DEVICE_NAME, RT_DEVICE_CTRL_RTC_GET_TIME, &time);
	if(status == 0 && tp != NULL){
		tp->tv_sec = time;
		tp->tv_usec = 0;
		return time;
	}
	return 0;
}


#ifndef _gettimeofday
/* Dummy function when hardware do not have RTC */
int _gettimeofday( struct timeval_at *tv, void *ignore)
{
	tv->tv_sec = 0;  // convert to seconds
	tv->tv_usec = 0;  // get remaining microseconds
	return 0;  // return non-zero for error
}
#endif

/**
 * Returns the current time.
 *
 * @param time_t_at * t the timestamp pointer, if not used, keep NULL.
 *
 * @return time_t_at return timestamp current.
 *
 */
/* for IAR 6.2 later Compiler */
time_t_at time_at(time_t_at *t)
{
	time_t_at time_now = 0;

	sddev_control(SOFT_RTC_DEVICE_NAME, RT_DEVICE_CTRL_RTC_GET_TIME, &time_now);
	return time_now;
}

