#include <stdio.h>
#include <stdint.h>
#include <ctype.h>

int view_buffer2 ( const char *str, void const * pv, uint32_t len, int (*pf)( const char *, ... ) )
{
uint8_t *p = (uint8_t *)pv;
uint32_t  i = 0 , x = 0;
char  buf[20], *pb = &buf[0];
char  tb[46], *tpb = &tb[0];

if( len > 256 && !(len & 0x8000) )   // don't go silly with the length eh!
   len = 256;
len &= ~0x8000;
if( len > 1824 )
{
   /* likely to be an error. Limit to something sensible */
   len = 256;
}

(*pf) ( "\r\n%s (len: %d, @: %p)\r\n     ", str, len, p );
while ( len-- )
   {
   (*pf) ( "%02X ", *p );
   if ( isalnum(*p) )
      *pb++ = *p;
   else
      *pb++ = '.';
   p++;
   if ( ++i == 16 )
      {
      *pb = 0;
      (*pf) ( "  %s\r\n     ", buf );
      i = 0;
      pb = &buf[0];
      }
   else if ( i == 8 )
      *pb++ = ' ';
   }
if ( i ) //ran out of bytes. finish off buffer
   {

   x = 3 * (16-i);
   while (x-- )
   {
      *tpb++ = ' ';
   }
   *tpb = 0;
   pf("%s",tb);

   *pb = 0;
   (*pf) ( "  %s\r\n", buf );
   }
   return 0;
}

int view_buffer ( const char *str, void const * pv, uint32_t len )
{
	return view_buffer2( str, pv, len, printf );
}

