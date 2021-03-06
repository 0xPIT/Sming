/* $Id: Print.cpp 1156 2011-06-07 04:01:16Z bhagman $
||
|| @author         Hernando Barragan <b@wiring.org.co>
|| @url            http://wiring.org.co/
|| @contribution   Nicholas Zambetti
|| @contribution   Brett Hagman <bhagman@wiring.org.co>
|| @contribution   Alexander Brevig <abrevig@wiring.org.co>
||
|| @description
|| | Print library.
|| |
|| | Wiring Common API
|| #
||
|| @license Please see cores/Common/License.txt.
||
*/
#undef __STRICT_ANSI__
#include "Print.h"
#include "WiringFrameworkIncludes.h"
#include <stdio.h>


extern "C" {
  /*
 * Copyright Patrick Powell 1995
 * This code is based on code written by Patrick Powell (papowell@astart.com)
 * It may be used for any purpose as long as this notice remains intact
 * on all source code distributions
 */

/**************************************************************
 * Original:
 * Patrick Powell Tue Apr 11 09:48:21 PDT 1995
 * A bombproof version of doprnt (dopr) included.
 * Sigh.  This sort of thing is always nasty do deal with.  Note that
 * the version here does not include floating point...
 *
 * snprintf() is used instead of sprintf() as it does limit checks
 * for string length.  This covers a nasty loophole.
 *
 * The other functions are there to prevent NULL pointers from
 * causing nast effects.
 *
 * More Recently:
 *  Brandon Long <blong@fiction.net> 9/15/96 for mutt 0.43
 *  This was ugly.  It is still ugly.  I opted out of floating point
 *  numbers, but the formatter understands just about everything
 *  from the normal C string format, at least as far as I can tell from
 *  the Solaris 2.5 printf(3S) man page.
 *
 *  Brandon Long <blong@fiction.net> 10/22/97 for mutt 0.87.1
 *    Ok, added some minimal floating point support, which means this
 *    probably requires libm on most operating systems.  Don't yet
 *    support the exponent (e,E) and sigfig (g,G).  Also, fmtint()
 *    was pretty badly broken, it just wasn't being exercised in ways
 *    which showed it, so that's been fixed.  Also, formated the code
 *    to mutt conventions, and removed dead code left over from the
 *    original.  Also, there is now a builtin-test, just compile with:
 *           gcc -DTEST_SNPRINTF -o snprintf snprintf.c -lm
 *    and run snprintf for results.
 * 
 *  Thomas Roessler <roessler@guug.de> 01/27/98 for mutt 0.89i
 *    The PGP code was using unsigned hexadecimal formats. 
 *    Unfortunately, unsigned formats simply didn't work.
 *
 *  Michael Elkins <me@cs.hmc.edu> 03/05/98 for mutt 0.90.8
 *    The original code assumed that both snprintf() and vsnprintf() were
 *    missing.  Some systems only have snprintf() but not vsnprintf(), so
 *    the code is now broken down under HAVE_SNPRINTF and HAVE_VSNPRINTF.
 *
 *  Andrew Tridgell (tridge@samba.org) Oct 1998
 *    fixed handling of %.0f
 *    added test for HAVE_LONG_DOUBLE
 *
 **************************************************************/

#include <string.h>
#include <ctype.h>
//#include <sys/types.h>

/* Define this as a fall through, HAVE_STDARG_H is probably already set */
#define HAVE_VARARGS_H


/* varargs declarations: */

# include <stdarg.h>
# define HAVE_STDARGS    /* let's hope that works everywhere (mj) */
# define VA_LOCAL_DECL   va_list ap
# define VA_START(f)     va_start(ap, f)
# define VA_SHIFT(v,t)  ;   /* no-op for ANSI */
# define VA_END          va_end(ap)

// #  include <varargs.h>
// #  undef HAVE_STDARGS
// #  define VA_LOCAL_DECL   va_list ap
// #  define VA_START(f)     va_start(ap)      /* f is ignored! */
// #  define VA_SHIFT(v,t) v = va_arg(ap,t)
// #  define VA_END        va_end(ap)


//#ifdef HAVE_LONG_DOUBLE
//#define LDOUBLE long double
//#else
#define LDOUBLE double
//#endif

int snprintf (char *str, size_t count, const char *fmt, ...);
int vsnprintf (char *str, size_t count, const char *fmt, va_list arg);

static void dopr (char *buffer, size_t maxlen, const char *format, 
                  va_list args);
static void fmtstr (char *buffer, size_t *currlen, size_t maxlen,
        char *value, int flags, int min, int max);
static void fmtint (char *buffer, size_t *currlen, size_t maxlen,
        long value, int base, int min, int max, int flags);
static void fmtfp (char *buffer, size_t *currlen, size_t maxlen,
       LDOUBLE fvalue, int min, int max, int flags);
static void dopr_outch (char *buffer, size_t *currlen, size_t maxlen, char c );

/*
 * dopr(): poor man's version of doprintf
 */

/* format read states */
#define DP_S_DEFAULT 0
#define DP_S_FLAGS   1
#define DP_S_MIN     2
#define DP_S_DOT     3
#define DP_S_MAX     4
#define DP_S_MOD     5
#define DP_S_CONV    6
#define DP_S_DONE    7

/* format flags - Bits */
#define DP_F_MINUS  (1 << 0)
#define DP_F_PLUS   (1 << 1)
#define DP_F_SPACE  (1 << 2)
#define DP_F_NUM    (1 << 3)
#define DP_F_ZERO   (1 << 4)
#define DP_F_UP     (1 << 5)
#define DP_F_UNSIGNED   (1 << 6)

/* Conversion Flags */
#define DP_C_SHORT   1
#define DP_C_LONG    2
#define DP_C_LDOUBLE 3

#define char_to_int(p) (p - '0')
#define MAX(p,q) ((p >= q) ? p : q)

static void dopr (char *buffer, size_t maxlen, const char *format, va_list args)
{
  char ch;
  long value;
  LDOUBLE fvalue;
  char *strvalue;
  int min;
  int max;
  int state;
  int flags;
  int cflags;
  size_t currlen;
  
  state = DP_S_DEFAULT;
  currlen = flags = cflags = min = 0;
  max = -1;
  ch = *format++;

  while (state != DP_S_DONE)
  {
    if ((ch == '\0') || (currlen >= maxlen)) 
      state = DP_S_DONE;

    switch(state) 
    {
    case DP_S_DEFAULT:
      if (ch == '%') 
  state = DP_S_FLAGS;
      else 
  dopr_outch (buffer, &currlen, maxlen, ch);
      ch = *format++;
      break;
    case DP_S_FLAGS:
      switch (ch) 
      {
      case '-':
  flags |= DP_F_MINUS;
        ch = *format++;
  break;
      case '+':
  flags |= DP_F_PLUS;
        ch = *format++;
  break;
      case ' ':
  flags |= DP_F_SPACE;
        ch = *format++;
  break;
      case '#':
  flags |= DP_F_NUM;
        ch = *format++;
  break;
      case '0':
  flags |= DP_F_ZERO;
        ch = *format++;
  break;
      default:
  state = DP_S_MIN;
  break;
      }
      break;
    case DP_S_MIN:
      if (isdigit(ch)) 
      {
  min = 10*min + char_to_int (ch);
  ch = *format++;
      } 
      else if (ch == '*') 
      {
  min = va_arg (args, int);
  ch = *format++;
  state = DP_S_DOT;
      } 
      else 
  state = DP_S_DOT;
      break;
    case DP_S_DOT:
      if (ch == '.') 
      {
  state = DP_S_MAX;
  ch = *format++;
      } 
      else 
  state = DP_S_MOD;
      break;
    case DP_S_MAX:
      if (isdigit(ch)) 
      {
  if (max < 0)
    max = 0;
  max = 10*max + char_to_int (ch);
  ch = *format++;
      } 
      else if (ch == '*') 
      {
  max = va_arg (args, int);
  ch = *format++;
  state = DP_S_MOD;
      } 
      else 
  state = DP_S_MOD;
      break;
    case DP_S_MOD:
      /* Currently, we don't support Long Long, bummer */
      switch (ch) 
      {
      case 'h':
  cflags = DP_C_SHORT;
  ch = *format++;
  break;
      case 'l':
  cflags = DP_C_LONG;
  ch = *format++;
  break;
      case 'L':
  cflags = DP_C_LDOUBLE;
  ch = *format++;
  break;
      default:
  break;
      }
      state = DP_S_CONV;
      break;
    case DP_S_CONV:
      switch (ch) 
      {
      case 'd':
      case 'i':
  if (cflags == DP_C_SHORT) 
    value = va_arg (args, /*short*/ int);
  else if (cflags == DP_C_LONG)
    value = va_arg (args, long int);
  else
    value = va_arg (args, int);
  fmtint (buffer, &currlen, maxlen, value, 10, min, max, flags);
  break;
  //     case 'o':
  // flags |= DP_F_UNSIGNED;
  // if (cflags == DP_C_SHORT)
  //   value = va_arg (args, unsigned short int);
  // else if (cflags == DP_C_LONG)
  //   value = va_arg (args, unsigned long int);
  // else
  //   value = va_arg (args, unsigned int);
  // fmtint (buffer, &currlen, maxlen, value, 8, min, max, flags);
  // break;
      case 'u':
        flags |= DP_F_UNSIGNED;
        if (cflags == DP_C_SHORT)
          value = va_arg (args, /*unsigned short*/ int);
        else if (cflags == DP_C_LONG)
          value = va_arg (args, unsigned long int);
        else
          value = va_arg (args, unsigned int);
        fmtint (buffer, &currlen, maxlen, value, 10, min, max, flags);
        break;
      case 'X':
  flags |= DP_F_UP;
      case 'x':
  flags |= DP_F_UNSIGNED;
  if (cflags == DP_C_SHORT)
    value = va_arg (args, /*unsigned short*/ int);
  else if (cflags == DP_C_LONG)
    value = va_arg (args, unsigned long int);
  else
    value = va_arg (args, unsigned int);
  fmtint (buffer, &currlen, maxlen, value, 16, min, max, flags);
  break;
      case 'f':
  if (cflags == DP_C_LDOUBLE)
    fvalue = va_arg (args, LDOUBLE);
  else
    fvalue = va_arg (args, double);
  /* um, floating point? */
  fmtfp (buffer, &currlen, maxlen, fvalue, min, max, flags);
  break;
      case 'E':
  flags |= DP_F_UP;
      case 'e':
  if (cflags == DP_C_LDOUBLE)
    fvalue = va_arg (args, LDOUBLE);
  else
    fvalue = va_arg (args, double);
  break;
      case 'G':
  flags |= DP_F_UP;
      case 'g':
  if (cflags == DP_C_LDOUBLE)
    fvalue = va_arg (args, LDOUBLE);
  else
    fvalue = va_arg (args, double);
  break;
      case 'c':
  dopr_outch (buffer, &currlen, maxlen, va_arg (args, int));
  break;
      case 's':
  strvalue = va_arg (args, char *);
  if (max < 0) 
    max = maxlen; /* ie, no max */
  fmtstr (buffer, &currlen, maxlen, strvalue, flags, min, max);
  break;
  //     case 'p':
  // strvalue = va_arg (args, void *);
  // fmtint (buffer, &currlen, maxlen, (long) strvalue, 16, min, max, flags);
  // break;
      case 'n':
  if (cflags == DP_C_SHORT) 
  {
    short int *num;
    num = va_arg (args, short int *);
    *num = currlen;
        } 
  else if (cflags == DP_C_LONG) 
  {
    long int *num;
    num = va_arg (args, long int *);
    *num = currlen;
        } 
  else 
  {
    int *num;
    num = va_arg (args, int *);
    *num = currlen;
        }
  break;
      case '%':
  dopr_outch (buffer, &currlen, maxlen, ch);
  break;
      case 'w':
  /* not supported yet, treat as next char */
  ch = *format++;
  break;
      default:
  /* Unknown, skip */
  break;
      }
      ch = *format++;
      state = DP_S_DEFAULT;
      flags = cflags = min = 0;
      max = -1;
      break;
    case DP_S_DONE:
      break;
    default:
      /* hmm? */
      break; /* some picky compilers need this */
    }
  }
  if (currlen < maxlen - 1) 
    buffer[currlen] = '\0';
  else 
    buffer[maxlen - 1] = '\0';
}

static void fmtstr (char *buffer, size_t *currlen, size_t maxlen,
        char *value, int flags, int min, int max)
{
  int padlen, strln;     /* amount to pad */
  int cnt = 0;
  
  if (value == 0)
  {
    value = strdup("<NULL>");
  }

  for (strln = 0; value[strln]; ++strln); /* strlen */
  padlen = min - strln;
  if (padlen < 0) 
    padlen = 0;
  if (flags & DP_F_MINUS) 
    padlen = -padlen; /* Left Justify */

  while ((padlen > 0) && (cnt < max)) 
  {
    dopr_outch (buffer, currlen, maxlen, ' ');
    --padlen;
    ++cnt;
  }
  while (*value && (cnt < max)) 
  {
    dopr_outch (buffer, currlen, maxlen, *value++);
    ++cnt;
  }
  while ((padlen < 0) && (cnt < max)) 
  {
    dopr_outch (buffer, currlen, maxlen, ' ');
    ++padlen;
    ++cnt;
  }
}

/* Have to handle DP_F_NUM (ie 0x and 0 alternates) */

static void fmtint (char *buffer, size_t *currlen, size_t maxlen,
        long value, int base, int min, int max, int flags)
{
  int signvalue = 0;
  unsigned long uvalue;
  char convert[20];
  int place = 0;
  int spadlen = 0; /* amount to space pad */
  int zpadlen = 0; /* amount to zero pad */
  int caps = 0;
  
  if (max < 0)
    max = 0;

  uvalue = value;

  if(!(flags & DP_F_UNSIGNED))
  {
    if( value < 0 ) {
      signvalue = '-';
      uvalue = -value;
    }
    else
      if (flags & DP_F_PLUS)  /* Do a sign (+/i) */
  signvalue = '+';
    else
      if (flags & DP_F_SPACE)
  signvalue = ' ';
  }
  
  if (flags & DP_F_UP) caps = 1; /* Should characters be upper case? */

  do {
    convert[place++] =
      (caps? "0123456789ABCDEF":"0123456789abcdef")
      [uvalue % (unsigned)base  ];
    uvalue = (uvalue / (unsigned)base );
  } while(uvalue && (place < 20));
  if (place == 20) place--;
  convert[place] = 0;

  zpadlen = max - place;
  spadlen = min - MAX (max, place) - (signvalue ? 1 : 0);
  if (zpadlen < 0) zpadlen = 0;
  if (spadlen < 0) spadlen = 0;
  if (flags & DP_F_ZERO)
  {
    zpadlen = MAX(zpadlen, spadlen);
    spadlen = 0;
  }
  if (flags & DP_F_MINUS) 
    spadlen = -spadlen; /* Left Justifty */

#ifdef DEBUG_SNPRINTF
  dprint (1, (debugfile, "zpad: %d, spad: %d, min: %d, max: %d, place: %d\n",
      zpadlen, spadlen, min, max, place));
#endif

  /* Spaces */
  while (spadlen > 0) 
  {
    dopr_outch (buffer, currlen, maxlen, ' ');
    --spadlen;
  }

  /* Sign */
  if (signvalue) 
    dopr_outch (buffer, currlen, maxlen, signvalue);

  /* Zeros */
  if (zpadlen > 0) 
  {
    while (zpadlen > 0)
    {
      dopr_outch (buffer, currlen, maxlen, '0');
      --zpadlen;
    }
  }

  /* Digits */
  while (place > 0) 
    dopr_outch (buffer, currlen, maxlen, convert[--place]);
  
  /* Left Justified spaces */
  while (spadlen < 0) {
    dopr_outch (buffer, currlen, maxlen, ' ');
    ++spadlen;
  }
}

static LDOUBLE abs_val (LDOUBLE value)
{
  LDOUBLE result = value;

  if (value < 0)
    result = -value;

  return result;
}

static LDOUBLE pow10 (int exp)
{
  LDOUBLE result = 1;

  while (exp)
  {
    result *= 10;
    exp--;
  }
  
  return result;
}

static long print_round (LDOUBLE value)
{
  long intpart;

  intpart = value;
  value = value - intpart;
  if (value >= 0.5)
    intpart++;

  return intpart;
}

static void fmtfp (char *buffer, size_t *currlen, size_t maxlen,
       LDOUBLE fvalue, int min, int max, int flags)
{
  int signvalue = 0;
  LDOUBLE ufvalue;
  char iconvert[20];
  char fconvert[20];
  int iplace = 0;
  int fplace = 0;
  int padlen = 0; /* amount to pad */
  int zpadlen = 0; 
  int caps = 0;
  long intpart;
  long fracpart;
  
  /* 
   * AIX manpage says the default is 0, but Solaris says the default
   * is 6, and sprintf on AIX defaults to 6
   */
  if (max < 0)
    max = 6;

  ufvalue = abs_val (fvalue);

  if (fvalue < 0)
    signvalue = '-';
  else
    if (flags & DP_F_PLUS)  /* Do a sign (+/i) */
      signvalue = '+';
    else
      if (flags & DP_F_SPACE)
  signvalue = ' ';

#if 0
  if (flags & DP_F_UP) caps = 1; /* Should characters be upper case? */
#endif

  intpart = ufvalue;

  /* 
   * Sorry, we only support 9 digits past the decimal because of our 
   * conversion method
   */
  if (max > 9)
    max = 9;

  /* We "cheat" by converting the fractional part to integer by
   * multiplying by a factor of 10
   */
  fracpart = print_round ((pow10 (max)) * (ufvalue - intpart));

  if (fracpart >= pow10 (max))
  {
    intpart++;
    fracpart -= pow10 (max);
  }

#ifdef DEBUG_SNPRINTF
  dprint (1, (debugfile, "fmtfp: %f =? %d.%d\n", fvalue, intpart, fracpart));
#endif

  /* Convert integer part */
  do {
    iconvert[iplace++] =
      (caps? "0123456789ABCDEF":"0123456789abcdef")[intpart % 10];
    intpart = (intpart / 10);
  } while(intpart && (iplace < 20));
  if (iplace == 20) iplace--;
  iconvert[iplace] = 0;

  /* Convert fractional part */
  do {
    fconvert[fplace++] =
      (caps? "0123456789ABCDEF":"0123456789abcdef")[fracpart % 10];
    fracpart = (fracpart / 10);
  } while(fracpart && (fplace < 20));
  if (fplace == 20) fplace--;
  fconvert[fplace] = 0;

  /* -1 for decimal point, another -1 if we are printing a sign */
  padlen = min - iplace - max - 1 - ((signvalue) ? 1 : 0); 
  zpadlen = max - fplace;
  if (zpadlen < 0)
    zpadlen = 0;
  if (padlen < 0) 
    padlen = 0;
  if (flags & DP_F_MINUS) 
    padlen = -padlen; /* Left Justifty */

  if ((flags & DP_F_ZERO) && (padlen > 0)) 
  {
    if (signvalue) 
    {
      dopr_outch (buffer, currlen, maxlen, signvalue);
      --padlen;
      signvalue = 0;
    }
    while (padlen > 0)
    {
      dopr_outch (buffer, currlen, maxlen, '0');
      --padlen;
    }
  }
  while (padlen > 0)
  {
    dopr_outch (buffer, currlen, maxlen, ' ');
    --padlen;
  }
  if (signvalue) 
    dopr_outch (buffer, currlen, maxlen, signvalue);

  while (iplace > 0) 
    dopr_outch (buffer, currlen, maxlen, iconvert[--iplace]);

  /*
   * Decimal point.  This should probably use locale to find the correct
   * char to print out.
   */
  if (max > 0)
  {
    dopr_outch (buffer, currlen, maxlen, '.');

    while (fplace > 0) 
      dopr_outch (buffer, currlen, maxlen, fconvert[--fplace]);
  }

  while (zpadlen > 0)
  {
    dopr_outch (buffer, currlen, maxlen, '0');
    --zpadlen;
  }

  while (padlen < 0) 
  {
    dopr_outch (buffer, currlen, maxlen, ' ');
    ++padlen;
  }
}

static void dopr_outch (char *buffer, size_t *currlen, size_t maxlen, char c)
{
  if (*currlen < maxlen)
    buffer[(*currlen)++] = c;
}

#ifndef HAVE_VSNPRINTF
int vsnprintf (char *str, size_t count, const char *fmt, va_list args)
{
  str[0] = 0;
  dopr(str, count, fmt, args);
  return(strlen(str));
}
#endif /* !HAVE_VSNPRINTF */

// #ifndef HAVE_SNPRINTF
// /* VARARGS3 */
// #ifdef HAVE_STDARGS
// int snprintf (char *str,size_t count,const char *fmt,...)
// #else
// int snprintf (va_alist) va_dcl
// #endif
// {
// #ifndef HAVE_STDARGS
//   char *str;
//   size_t count;
//   char *fmt;
// #endif
//   VA_LOCAL_DECL;
    
//   VA_START (fmt);
//   VA_SHIFT (str, char *);
//   VA_SHIFT (count, size_t );
//   VA_SHIFT (fmt, char *);
//   (void) vsnprintf(str, count, fmt, ap);
//   VA_END;
//   return(strlen(str));
// }
// #endif /* !HAVE_SNPRINTF */

// #endif /* !HAVE_SNPRINTF */
//  Shut up the compaq compiler which hates empty files. This will
//    never be linked anyway. 
// static void do_nothing() { return; }

}

/*
|| @description
|| | Virtual method - may be redefined in derived class (polymorphic)
|| | write()s a specific length string.
|| #
*/

size_t Print::write(const uint8_t *buffer, size_t size)
{
  size_t n = 0;
  while (size--) {
    n += write(*buffer++);
  }
  return n;
}


// Base method (character)
size_t Print::print(char c)
{
  return write(c);
}

// Base method (string)
size_t Print::print(const char c[])
{
  return write(c);
}


// Base method (unsigned)
size_t Print::print(unsigned long n, int base)
{
  if (base == 0) return write(n);
  else return printNumber(n, base);
}

// Base method (signed)
size_t Print::print(long n, int base)
{
  if (base == 0)
  {
    return write(n);
  }
  else if (base == 10)
  {
    // why must this only be in base 10?
    if (n < 0)
    {
      int t = print('-');
      n = -n;
      return printNumber(n, 10) + t;
    }
    return printNumber(n, 10);
  }
  else
  {
    return printNumber(n, base);
  }
}


// Overload (unsigned)
size_t Print::print(unsigned int n, int base)
{
  return print((unsigned long)n, base);
}

// Overload (unsigned)
size_t Print::print(unsigned char n, int base)
{
  return print((unsigned long) n, base);
}

// Overload (signed)
size_t Print::print(int n, int base)
{
  return print((long)n, base);
}


size_t Print::print(double n, int digits)
{
  return printFloat(n, digits);
}


size_t Print::print(const Printable &p)
{
  return p.printTo(*this);
}

size_t Print::print(const String &s)
{
  return write(s.c_str(), s.length());
}


size_t Print::println(void)
{
  size_t n = print('\r');
  n += print('\n');
  return n;
}


size_t Print::println(const String &s)
{
  size_t n = print(s);
  n += println();
  return n;
}


size_t Print::println(char c)
{
  size_t n = print(c);
  n += println();
  return n;
}

size_t Print::println(const char c[])
{
  size_t n = print(c);
  n += println();
  return n;
}


size_t Print::println(unsigned long num, int base)
{
  size_t n = print(num, base);
  n += println();
  return n;
}

size_t Print::println(unsigned int num, int base)
{
  size_t n = print(num, base);
  n += println();
  return n;
}

size_t Print::println(unsigned char b, int base)
{
  size_t n = print(b, base);
  n += println();
  return n;
}

size_t Print::println(long num, int base)
{
  size_t n = print(num, base);
  n += println();
  return n;
}

size_t Print::println(int num, int base)
{
  size_t n = print(num, base);
  n += println();
  return n;
}

size_t Print::println(double num, int digits)
{
  size_t n = print(num, digits);
  n += println();
  return n;
}


size_t Print::println(const Printable &p)
{
  size_t n = print(p);
  n += println();
  return n;
}

size_t Print::printf(const char *fmt, ...)
{
	size_t sz = 0;
	size_t buffSize = INITIAL_PRINTF_BUFFSIZE;
	bool retry = false;
	do {
		char tempBuff[buffSize];
		va_list va;
		va_start(va, fmt);
		sz = vsnprintf(tempBuff,buffSize, fmt, va);
		va_end(va);
		if (sz > (buffSize -1))
		{
			buffSize = sz + 1; // Leave room for terminating null char
			retry = true;
		}
		else
		{
			if (sz > 0)
			{
				write(tempBuff,sz);
			}
			return sz;
		}
	} while (retry);
}

// private methods

size_t Print::printNumber(unsigned long n, uint8_t base)
{
  /* BH: new version to be implemented
    uint8_t buf[sizeof(char) * sizeof(int32_t)];
    uint32_t i = 0;

    if (n == 0)
    {
      write('0');
      return;
    }

    while (n > 0)
    {
      buf[i++] = n % base;
      n /= base;
    }

    for (; i > 0; i--)
      write((buf[i - 1] < 10 ?
            '0' + buf[i - 1] :
            'A' + buf[i - 1] - 10));
  */

  char buf[8 * sizeof(long) + 1]; // Assumes 8-bit chars plus zero byte.
  char *str = &buf[sizeof(buf) - 1];
  
  *str = '\0';
  
  // prevent crash if called with base == 1
  if (base < 2) base = 10;
  
  do {
    unsigned long m = n;
    n /= base;
    char c = m - base * n;
    *--str = c < 10 ? c + '0' : c + 'A' - 10;
  } while(n);
  
  return write(str);
}

size_t Print::printFloat(double number, uint8_t digits)
{
  size_t n = 0;
  
  if (isnan(number)) return print("nan");
  if (isinf(number)) return print("inf");
  if (number > 4294967040.0) return print ("ovf");  // constant determined empirically
  if (number <-4294967040.0) return print ("ovf");  // constant determined empirically
  
  // Handle negative numbers
  if (number < 0.0)
  {
    n += print('-');
    number = -number;
  }
  
  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
  rounding /= 10.0;
  
  number += rounding;
  
  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  n += print(int_part);
  
  // Print the decimal point, but only if there are digits beyond
  if (digits > 0) {
    n += print(".");
  }
  
  // Extract digits from the remainder one at a time
  while (digits-- > 0)
  {
    remainder *= 10.0;
    int toPrint = int(remainder);
    n += print(toPrint);
    remainder -= toPrint;
  }
  
  return n;
}

