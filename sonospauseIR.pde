/*======================================================================
 SonosIR - Arduino sketch for infrared control of Sonos ZonePlayer
 (c) 2010 Simon Long, KuDaTa Software

 Original Sonos control Arduino sketch by Jan-Piet Mens.
 Original Arduino IRremote library by Ken Shirriff.

 Use is at the user's own risk - no warranty is expressed or implied.
 ======================================================================*/

#include <Client.h>
#include <Ethernet.h>

/*----------------------------------------------------------------------*/
/* Macros and constants */
/*----------------------------------------------------------------------*/

/* Sonos SOAP command packet header and footer */
#define SONOS_CMDH "<s:Envelope xmlns:s=\"http://schemas.xmlsoap.org/soap/envelope/\" s:encodingStyle=\"http://schemas.xmlsoap.org/soap/encoding/\"><s:Body>"
#define SONOS_CMDF "</s:Body></s:Envelope>"

/* Sonos SOAP command packet enumeration */
#define SONOS_PAUSE  0
#define SONOS_PLAY   1
#define SONOS_PREV   2
#define SONOS_NEXT   3
#define SONOS_SEEK   4
#define SONOS_NORMAL 5
#define SONOS_REPEAT 6
#define SONOS_SHUFF  7
#define SONOS_SHUREP 8
#define SONOS_MODE   9
#define SONOS_POSIT  10
#define SONOS_GETVOL 11
#define SONOS_SETVOL 12
#define SONOS_TRACK  13
#define SONOS_CLEAR  14
#define SONOS_SETURI 15

/* State machine for A-B repeat and intro scan functions */
#define MODE_NORMAL 0
#define MODE_SCAN   1
#define MODE_A      2
#define MODE_AB     3

/* Sums of ASCII values for play modes */
#define SUM_NORMAL            457
#define SUM_SHUFFLE           525
#define SUM_REPEAT_ALL        761
#define SUM_SHUFFLE_NOREPEAT  1226

/* Forward / rewind interval in seconds */
#define JUMP_SECS   10

/* Intro scan period in seconds */
#define INTRO_SECS 10

/* Volume increment / decrement */
#define VOL_DELTA   5

/* Minimum time between IR commands */
#define IR_DELAY 400

/* Polling interval in A-B and scan modes */
#define POLL_WAIT 500

/* Time during which number keys will be combined */
#define NUM_WAIT 1500

/* Time at start of track for which Prev jumps rather than seeks to 0 */
#define SKIP_WAIT 2000

/* Preset radio station URIs */
#define PRESET1 "mms://62.27.44.14/radiobielefeld$livestream.wma"
#define PRESET2 "x-rincon-mp3radio://gffstream.ic.llnwd.net/stream/gffstream_mp3_w98b"
#define PRESET3 "mms://mercury.sharp-stream.com/planetrock"
#define PRESET4 "mms://wmlive-acl.bbc.co.uk/wms/bbc_ami/radio2/radio2_bb_live_eq1_sl0"
#define PRESET5 ""
#define PRESET6 ""
#define PRESET7 ""
#define PRESET8 ""
#define PRESET9 ""
#define PRESET0 ""

/* Arduino pin to which IR receiver is connected */
#define IR_PIN 8
#define LED_PIN 7


/* IRremote hex codes for received remote commands */
#define REMOTE_PLAY      0x0C
#define REMOTE_PAUSE     0x0D
#define REMOTE_PREV      0x21
#define REMOTE_NEXT      0x20
#define REMOTE_SHUFFLE   0x1C
#define REMOTE_REPEAT    0x1D
#define REMOTE_AB        0x3B
#define REMOTE_SCAN      0x2B
#define REMOTE_REV       0x32
#define REMOTE_FWD       0x0A
#define REMOTE_VOLU      0x10
#define REMOTE_VOLD      0x11
#define REMOTE_KEY0      0x00
#define REMOTE_KEY1      0x01
#define REMOTE_KEY2      0x02
#define REMOTE_KEY3      0x03
#define REMOTE_KEY4      0x04
#define REMOTE_KEY5      0x05
#define REMOTE_KEY6      0x06
#define REMOTE_KEY7      0x07
#define REMOTE_KEY8      0x08
#define REMOTE_KEY9      0x09
#define REMOTE_CLEAR     0x31
#define REMOTE_RECALL    0x0F

/* IP addresses of Arduino and ZonePlayer*/
#define IP1     192
#define IP2     168
#define IP3     1
#define IP4ZP   132 // Office
//#define IP4ZP   11 // Bedroom
#define IP4ARD  141

// RC == 0088

/* Enable DEBUG for serial debug output */
// #define DEBUG

/*----------------------------------------------------------------------*/
/* Global variables */
/*----------------------------------------------------------------------*/

/* IP address of ZonePlayer to control */
byte sonosip[] = { IP1, IP2, IP3, IP4ZP };

/* Millisecond timer values */
unsigned long lastcmd = 0;
unsigned long lastrew = 0;
unsigned long lastpoll = 0;
unsigned long lastnum = 0;
unsigned long lastpreset = 0;

/* A-B repeat and intro scan state */
int mode;

/* Second timer values used for A-B repeat */
int posa, posb;

/* Global used to store number of seconds to seek to */
int desttime;

/* Global used for volume setting */
int newvol;

/* Global used for direct track number selection */
int desttrack;

/* Buffers used for Sonos data reception */
char data1[20];
char data2[20];

/* Global null buffer used to disable data reception */
char nullbuf[1] = { 0 };

/* Ethernet control */
Client client (sonosip, 1400);

/*----------------------------------------------------------------------*/
/* Cut-down IR library - only reads RC-5 */
/*----------------------------------------------------------------------*/

#include <avr/interrupt.h>

/* IR timer properties */
#define INIT_TIMER_COUNT2 161 // to achieve 50us interrupts, timer 2 overflows after (256-161) clock ticks
#define GAP_TICKS 100         // 50us * 100 = 5ms between data pulse trains

/* RC-5 pulse time windows */
#define T1L  11
#define T1H  20
#define T2L  25
#define T2H  42
#define TLC  3
#define THC  5

/* IR detector status */
#define MARK  0
#define SPACE 1

/* IR function return values */
#define ERR 0
#define DECODED 1

/* IR receiver states */
#define STATE_IDLE     0
#define STATE_MARK     1
#define STATE_SPACE    2
#define STATE_STOP     3

/* IR receiver control data */
#define RAWBUF 30
uint8_t recvpin;             // pin for IR data from detector
uint8_t rcvstate;            // state machine
unsigned int timer;          // state timer, counts 50uS ticks
unsigned int rawbuf[RAWBUF]; // raw data
uint8_t rawlen;              // counter of entries in rawbuf

/*----------------------------------------------------------------------*/
/* IRenable - configures IR reception */

void IRenable (int pin)
{
 /* set up the timer to generate interrupts */
 /* system clock = 16MHz, prescale set to /8 so ticks every 0.5us */
 TCCR2A = 0;  // normal mode
 _SFR_BYTE(TCCR2B) &= ~_BV(CS22);
 _SFR_BYTE(TCCR2B) |= _BV(CS21);
 _SFR_BYTE(TCCR2B) &= ~_BV(CS20);
 _SFR_BYTE(TIMSK2) |= _BV(TOIE2);

 /* approx 100 ticks per interrupt, so interrupts every 50us */
 TCNT2 = INIT_TIMER_COUNT2;

 /* enable interrupts */
 sei ();

 /* initialise state machine */
 rcvstate = STATE_IDLE;
 rawlen = 0;

 /* setup input pin */
 recvpin = pin;
 pinMode (recvpin, INPUT);
}

/*----------------------------------------------------------------------*/
/* IRdecode - reads last received code from buffer */

int IRdecode (unsigned long *value)
{
 if (rcvstate != STATE_STOP) return ERR;
 if (decodeRC5 (value)) return DECODED;
 IRresume ();
 return ERR;
}

/*----------------------------------------------------------------------*/
/* IRresume - resets IR reception */

void IRresume()
{
 rcvstate = STATE_IDLE;
 rawlen = 0;
}

/*----------------------------------------------------------------------*/
/* ISR - interrupt service routine to monitor state of IR receiver */

ISR(TIMER2_OVF_vect)
{
 uint8_t irdata;

 /* reset the timer to time the next call to the ISR */
 TCNT2 = INIT_TIMER_COUNT2;

 /* read the input pin */
 irdata = (uint8_t) digitalRead (recvpin);

 /* increment the tick counter */
 timer++;

 /* overflow of stored data buffer - give up */
 if (rawlen >= RAWBUF) rcvstate = STATE_STOP;

 switch (rcvstate)
 {
   case STATE_IDLE :
     if (irdata == MARK)
     {
       if (timer >= GAP_TICKS)
       {
         rawlen = 0;
         rawbuf[rawlen++] = timer;
         rcvstate = STATE_MARK;
       }
       timer = 0;
     }
     break;
   case STATE_MARK :
     if (irdata == SPACE)
     {
       rawbuf[rawlen++] = timer;
       timer = 0;
       rcvstate = STATE_SPACE;
     }
     break;
   case STATE_SPACE :
     if (irdata == MARK)
     {
       rawbuf[rawlen++] = timer;
       timer = 0;
       rcvstate = STATE_MARK;
     }
     else if (timer > GAP_TICKS) rcvstate = STATE_STOP;
     break;
   case STATE_STOP :
     if (irdata == MARK) timer = 0;
     break;
 }
}

/*----------------------------------------------------------------------*/
/* getRClevel - decodes next bit from RC-5 timing information */

int getRClevel (int *offset, int *used)
{
 int width, val, avail;

 /* end of buffer - assume SPACE */
 if (*offset >= rawlen) return SPACE;

 /* get the next width - is it a high or low? */
 width = rawbuf[*offset];
 val = ((*offset) % 2) ? MARK : SPACE;

 /* is the width 1 or 2 times the pulse width? */
 if (width >= T1L + (val * TLC) && width <= T1H + (val * THC)) avail = 1;
 else if (width >= T2L + (val * TLC) && width <= T2H + (val * THC)) avail = 2;
 else return -1;

 /* update bit state */
 (*used)++;
 if (*used >= avail)
 {
   *used = 0;
   (*offset)++;
 }
 return val;
}

/*----------------------------------------------------------------------*/
/* decodeRC5 - decodes a complete RC-5 command */

int decodeRC5 (unsigned long *value)
{
 int offset = 1;
 long data = 0;
 int used = 0;
 int nbits;

 if (rawlen < 13) return ERR;

 /* read start bits */
 if (getRClevel (&offset, &used) != MARK) return ERR;
 if (getRClevel (&offset, &used) != SPACE) return ERR;
 if (getRClevel (&offset, &used) != MARK) return ERR;

 for (nbits = 0; offset < rawlen; nbits++)
 {
   /* read bit pairs for data bits */
   int levelA = getRClevel (&offset, &used);
   int levelB = getRClevel (&offset, &used);

   if (levelA == SPACE && levelB == MARK) data = (data << 1) | 1;
   else if (levelA == MARK && levelB == SPACE) data <<= 1;
   else return ERR;
 }

 *value = data;
 return DECODED;
}

/*----------------------------------------------------------------------*/
/* Function defintions */
/*----------------------------------------------------------------------*/

/*----------------------------------------------------------------------*/
/* setup - configures Arduino */

void setup ()
{
 uint8_t mac[6] = { 0xBE, 0xEF, 0xEE, 0x00, 0x20, 0x09 };
 uint8_t ip[4] = { IP1, IP2, IP3, IP4ARD };

 pinMode(LED_PIN, OUTPUT);
 digitalWrite(LED_PIN, LOW);
 delay (3000);

 /* initialise Ethernet connection */
 Ethernet.begin (mac, ip);

 /* initialise IR receiver */
 IRenable (IR_PIN);

 /* disable intro scan and A-B repeat */
 mode = MODE_NORMAL;

#ifdef DEBUG
 Serial.begin (9600);
#endif
}

/*----------------------------------------------------------------------*/
/* loop - called repeatedly by Arduino */

void loop ()
{
 int res;
 unsigned long value;

 /* look to see if a packet has been received from the IR receiver */
 if (IRdecode (&value))
 {
   digitalWrite(LED_PIN, HIGH);

   /* only process if this is the first packet for 200ms - prevents multiple operations */
   if (millis () > lastcmd)
   {
#ifdef DEBUG
     Serial.println (value, HEX);
#endif
     /* store time for which to ignore further IR commands */
     lastcmd = millis () + IR_DELAY;

     /* compare received IR against known commands */
     switch (value & 0xFF)
     {
       case REMOTE_SCAN :
                    if (mode == MODE_SCAN)
                      mode = MODE_NORMAL;
                    else
                    {
                      mode = MODE_SCAN;
                      sonos (SONOS_POSIT, data1, nullbuf);
                      if (seconds (data1 + 1) > 10) sonos (SONOS_NEXT, nullbuf, nullbuf);
                    }
                    break;

       case REMOTE_AB :
                    if (mode == MODE_AB)
                      mode = MODE_NORMAL;
                    else if (mode == MODE_A)
                    {
                      mode = MODE_AB;
                      sonos (SONOS_POSIT, data1, nullbuf);
                      posb = seconds (data1 + 1);
                    }
                    else
                    {
                      mode = MODE_A;
                      sonos (SONOS_POSIT, data1, nullbuf);
                      posa = seconds (data1 + 1);
                    }
                    break;

       case REMOTE_PLAY :
                    mode = MODE_NORMAL;
                    sonos (SONOS_PLAY, nullbuf, nullbuf);
                    break;

       case REMOTE_PAUSE :
                    mode = MODE_NORMAL;
                    sonos (SONOS_PAUSE, nullbuf, nullbuf);
                    break;

       case REMOTE_NEXT :
                    mode = MODE_NORMAL;
                    sonos (SONOS_NEXT, nullbuf, nullbuf);
                    break;

       case REMOTE_PREV :
                    mode = MODE_NORMAL;
                    if (millis () > lastrew)
                    {
                      desttime = 0;
                      sonos (SONOS_SEEK, nullbuf, nullbuf);
                    }
                    else
                      sonos (SONOS_PREV, nullbuf, nullbuf);
                    lastrew = millis () + SKIP_WAIT;
                    break;

       case REMOTE_SHUFFLE :
                    mode = MODE_NORMAL;
                    sonos (SONOS_MODE, data1, nullbuf);
                    res = sum_letters (data1 + 1);
                    if (res == SUM_NORMAL) sonos (SONOS_SHUFF, nullbuf, nullbuf);
                    if (res == SUM_SHUFFLE) sonos (SONOS_REPEAT, nullbuf, nullbuf);
                    if (res == SUM_REPEAT_ALL) sonos (SONOS_SHUREP, nullbuf, nullbuf);
                    if (res == SUM_SHUFFLE_NOREPEAT) sonos (SONOS_NORMAL, nullbuf, nullbuf);
                    break;

       case REMOTE_REPEAT :
                    mode = MODE_NORMAL;
                    sonos (SONOS_MODE, data1, nullbuf);
                    res = sum_letters (data1 + 1);
                    if (res == SUM_NORMAL) sonos (SONOS_REPEAT, nullbuf, nullbuf);
                    if (res == SUM_SHUFFLE) sonos (SONOS_SHUFF, nullbuf, nullbuf);
                    if (res == SUM_REPEAT_ALL) sonos (SONOS_NORMAL, nullbuf, nullbuf);
                    if (res == SUM_SHUFFLE_NOREPEAT) sonos (SONOS_SHUREP, nullbuf, nullbuf);
                    break;

       case REMOTE_REV :
                    mode = MODE_NORMAL;
                    sonos (SONOS_POSIT, data1, nullbuf);
                    res = seconds (data1 + 1);
                    res -= JUMP_SECS;
                    if (res > 0)
                    {
                      desttime = res;
                      sonos (SONOS_SEEK, nullbuf, nullbuf);
                    }
                    break;

       case REMOTE_FWD :
                    mode = MODE_NORMAL;
                    strcpy (data2, "TrackDuration");
                    sonos (SONOS_POSIT, data1, data2);
                    res = seconds (data1 + 1);
                    res += JUMP_SECS;
                    if (res < seconds (data2 + 1))
                    {
                      desttime = res;
                      sonos (SONOS_SEEK, nullbuf, nullbuf);
                    }
                    break;

       case REMOTE_VOLU :
                    sonos (SONOS_GETVOL, data1, nullbuf);
                    sscanf (data1 + 1, "%d", &newvol);
                    newvol += VOL_DELTA;
                    if (newvol > 100) newvol = 100;
                    sonos (SONOS_SETVOL, nullbuf, nullbuf);
                    break;

       case REMOTE_VOLD :
                    sonos (SONOS_GETVOL, data1, nullbuf);
                    sscanf (data1 + 1, "%d", &newvol);
                    newvol -= VOL_DELTA;
                    if (newvol < 0) newvol = 0;
                    sonos (SONOS_SETVOL, nullbuf, nullbuf);
                    break;

       case REMOTE_KEY0 :
       case REMOTE_KEY1 :
       case REMOTE_KEY2 :
       case REMOTE_KEY3 :
       case REMOTE_KEY4 :
       case REMOTE_KEY5 :
       case REMOTE_KEY6 :
       case REMOTE_KEY7 :
       case REMOTE_KEY8 :
       case REMOTE_KEY9 :
                    mode = MODE_NORMAL;
                    if (lastpreset)
                    {
                      desttrack = (value & 0xFF);
                      sonos (SONOS_SETURI, nullbuf, nullbuf);
                      sonos (SONOS_PLAY, nullbuf, nullbuf);
                      lastpreset = 0;
                    }
                    else
                    {
                      if (lastnum)
                      {
                        desttrack *= 10;
                        desttrack += (value & 0xFF);
                      }
                      else
                        desttrack = (value & 0xFF);
                      lastnum = millis () + NUM_WAIT;
                    }
                    break;

       case REMOTE_CLEAR :
                    mode = MODE_NORMAL;
                    sonos (SONOS_CLEAR, nullbuf, nullbuf);
                    break;

       case REMOTE_RECALL :
                    mode = MODE_NORMAL;
                    lastpreset = millis () + NUM_WAIT;
                    break;
      }
   }

   digitalWrite(LED_PIN, LOW);
   /* get ready to receive next IR command */
   IRresume ();
 }

 /* processing for intro scan and A-B repeat modes */
 if (mode != MODE_NORMAL)
 {
   /* if in intro scan or A-B, poll the current playback position every 0.5s */
   if (millis () > lastpoll)
   {
     sonos (SONOS_POSIT, data1, nullbuf);
     res = seconds (data1 + 1);
     if (mode == MODE_SCAN)
     {
       /* intro scan - if current time is greater than 10 seconds, skip forward */
       if (res >= INTRO_SECS)
       {
         strcpy (data1, "fault");
         sonos (SONOS_NEXT, data1, nullbuf);

         /* if skip returned an error, at end of playlist - cancel scan */
         if (data1[0] != 'f') mode = MODE_NORMAL;
       }
     }
     if (mode == MODE_AB)
     {
       /* A-B repeat - if current time is greater than end time, skip to start time */
       if (res >= posb)
       {
         desttime = posa;
         sonos (SONOS_SEEK, nullbuf, nullbuf);
       }
     }
     if (mode == MODE_A)
     {
       /* A pressed - waiting for B - abort if track end reached */
       if (res < posa) mode = MODE_NORMAL;
     }

     /* update playback position timer */
     lastpoll = millis () + POLL_WAIT;
   }
 }

 if (lastnum)
 {
   if (millis () > lastnum)
   {
     sonos (SONOS_TRACK, nullbuf, nullbuf);
     lastnum = 0;
   }
 }

 if (lastpreset)
 {
   if (millis () > lastpreset) lastpreset = 0;
 }
}

/*----------------------------------------------------------------------*/
/* seconds - converts supplied string in format hh:mm:ss to seconds */

int seconds (char *str)
{
 int hrs, mins, secs;
 sscanf (str, "%d:%d:%d", &hrs, &mins, &secs);
 hrs *= 60;
 hrs += mins;
 hrs *= 60;
 hrs += secs;
 return hrs;
}

/*----------------------------------------------------------------------*/
/* sum_letters - adds ASCII codes for all letters in supplied string */

int sum_letters (char *str)
{
 int tot = 0;
 char *ptr = str;
 while (*ptr)
 {
   tot += *ptr;
   ptr++;
 }
 return tot;
}

/*----------------------------------------------------------------------*/
/* out - outputs supplied string to Ethernet client */

void out (const char *s, int nl)
{
 if (nl)
 {
   client.println (s);
#ifdef DEBUG
   Serial.println (s);
#endif
 }
 else
 {
   client.print (s);
#ifdef DEBUG
   Serial.print (s);
#endif
 }
}

/*----------------------------------------------------------------------*/
/* tag - wraps supplied data in tags and appends it to buf */

void tag (char *buf, char *tag, char *data)
{
  char *ptr = buf;
  while (*ptr) ptr++;
  sprintf (ptr, "<%s>%s</%s>", tag, data, tag);
}

/*----------------------------------------------------------------------*/
/* set_service - writes service name to buffer */

void set_service (char *buf, int command)
{
  if (command == SONOS_SETVOL || command == SONOS_GETVOL) sprintf (buf, "RenderingControl");
  else sprintf (buf, "AVTransport");
}

/*----------------------------------------------------------------------*/
/* set_command - writes command name to buffer */

void set_command (char *buf, int command)
{
 switch (command)
 {
   case SONOS_PAUSE:   sprintf (buf, "Pause");
                       break;
   case SONOS_PLAY:    sprintf (buf, "Play");
                       break;
   case SONOS_PREV:    sprintf (buf, "Previous");
                       break;
   case SONOS_NEXT:    sprintf (buf, "Next");
                       break;
   case SONOS_SEEK:
   case SONOS_TRACK:   sprintf (buf, "Seek");
                       break;
   case SONOS_NORMAL:
   case SONOS_REPEAT:
   case SONOS_SHUFF:
   case SONOS_SHUREP:  sprintf (buf, "SetPlayMode");
                       break;
   case SONOS_MODE:    sprintf (buf, "GetTransportSettings");
                       break;
   case SONOS_POSIT:   sprintf (buf, "GetPositionInfo");
                       break;
   case SONOS_GETVOL:  sprintf (buf, "GetVolume");
                       break;
   case SONOS_SETVOL:  sprintf (buf, "SetVolume");
                       break;
   case SONOS_CLEAR:   sprintf (buf, "RemoveAllTracksFromQueue");
                       break;
   case SONOS_SETURI:  sprintf (buf, "SetAVTransportURI");
                       break;
 }
}

/*----------------------------------------------------------------------*/
/* sonos - sends a command packet to the ZonePlayer */

void sonos (int cmd, char *resp1, char *resp2)
{
 char buf[100];
 char extra[200];
 char cmdbuf[32];
 char service[20];
 char *ptr1;
 char *ptr2;
 char *optr;
 char copying;
 unsigned long timeout;

 if (client.connect ())
 {
#ifdef DEBUG
   Serial.println ("connected");
#endif

   /* prepare the extra data strings to go into the desired command packet */
   extra[0] = 0;
   switch (cmd)
   {
       case SONOS_PLAY :
           tag (extra, "Speed", "1");
           break;

       case SONOS_SEEK :
           sprintf (cmdbuf, "%02d:%02d:%02d", desttime / 3600, (desttime / 60) % 60, desttime % 60);
           tag (extra, "Unit", "REL_TIME");
           tag (extra, "Target", cmdbuf);
           break;

       case SONOS_NORMAL :
       case SONOS_REPEAT :
       case SONOS_SHUFF :
       case SONOS_SHUREP :
           if (cmd == SONOS_NORMAL) strcpy (cmdbuf, "NORMAL");
           if (cmd == SONOS_REPEAT) strcpy (cmdbuf, "REPEAT_ALL");
           if (cmd == SONOS_SHUFF) strcpy (cmdbuf, "SHUFFLE_NOREPEAT");
           if (cmd == SONOS_SHUREP) strcpy (cmdbuf, "SHUFFLE");
           tag (extra, "NewPlayMode", cmdbuf);
           break;

       case SONOS_MODE :
           strcpy (resp1, "PlayMode");
           break;

       case SONOS_POSIT :
           strcpy (resp1, "RelTime");
           break;

       case SONOS_GETVOL :
           tag (extra, "Channel", "Master");
           strcpy (resp1, "CurrentVolume");
           break;

       case SONOS_SETVOL :
           sprintf (cmdbuf, "%d", newvol);
           tag (extra, "Channel", "Master");
           tag (extra, "DesiredVolume", cmdbuf);
           break;

       case SONOS_TRACK :
           sprintf (cmdbuf, "%d", desttrack);
           tag (extra, "Unit", "TRACK_NR");
           tag (extra, "Target", cmdbuf);
           break;

       case SONOS_SETURI :
           switch (desttrack)
           {
             case 1 :  tag (extra, "CurrentURI", PRESET1);
                       break;
             case 2 :  tag (extra, "CurrentURI", PRESET2);
                       break;
             case 3 :  tag (extra, "CurrentURI", PRESET3);
                       break;
             case 4 :  tag (extra, "CurrentURI", PRESET4);
                       break;
             case 5 :  tag (extra, "CurrentURI", PRESET5);
                       break;
             case 6 :  tag (extra, "CurrentURI", PRESET6);
                       break;
             case 7 :  tag (extra, "CurrentURI", PRESET7);
                       break;
             case 8 :  tag (extra, "CurrentURI", PRESET8);
                       break;
             case 9 :  tag (extra, "CurrentURI", PRESET9);
                       break;
             case 0 :  tag (extra, "CurrentURI", PRESET0);
                       break;
           }
           tag (extra, "CurrentURIMetaData", nullbuf);
           break;
   }

   /* set the command and service names */
   set_command (cmdbuf, cmd);
   set_service (service, cmd);

   /* output the command packet */
   sprintf (buf, "POST /MediaRenderer/%s/Control HTTP/1.1", service);
   out (buf, 1);
   out ("Connection: close", 1);
   sprintf (buf, "Host: %d.%d.%d.%d:1400", sonosip[0], sonosip[1], sonosip[2], sonosip[3]);
   out (buf, 1);
   sprintf (buf, "Content-Length: %d", 231 + 2 * strlen (cmdbuf) + strlen (extra) + strlen (service));
   out (buf, 1);
   out ("Content-Type: text/xml; charset=\"utf-8\"", 1);
   sprintf (buf, "Soapaction: \"urn:schemas-upnp-org:service:%s:1#%s\"", service, cmdbuf);
   out (buf, 1);
   out ("", 1);
   out (SONOS_CMDH, 0);
   sprintf (buf, "<u:%s xmlns:u=\"urn:schemas-upnp-org:service:%s:1\">", cmdbuf, service);
   out (buf, 0);
   out ("<InstanceID>0</InstanceID>", 0);
   out (extra, 0);
   sprintf (buf, "</u:%s>", cmdbuf);
   out (buf, 0);
   out (SONOS_CMDF, 1);

   /* wait for a response packet */
   timeout = millis ();
   timeout += 1000;
   while ((!client.available ()) && (millis () < timeout));

   /* parse the response looking for the strings in resp1 and resp2 */
   ptr1 = resp1;
   ptr2 = resp2;
   copying = 0;
   while (client.available ())
   {
     char c = client.read ();

     /* if response buffers start with nulls, either no response required, or already received */
     if (resp1[0] || resp2[0])
     {
       /* if a response has been identified, copy the data */
       if (copying)
       {
         /* look for the < character that indicates the end of the data */
         if (c == '<')
         {
           /* stop receiving data, and null the first character in the response buffer */
           copying = 0;
           *optr = 0;
           if (copying == 1) resp1[0] = 0;
           else resp2[0] = 0;
         }
         else
         {
           /* copy the next byte to the response buffer */
           *optr = c;
           optr++;
         }
       }
       else
       {
         /* look for input characters that match the response buffers */
         if (c == *ptr1)
         {
           /* character matched - advance to next character to match */
           ptr1++;

           /* is this the end of the response buffer */
           if (*ptr1 == 0)
           {
             /* string matched - start copying from next character received */
             copying = 1;
             optr = resp1;
             ptr1 = resp1;
           }
         }
         else ptr1 = resp1;

         /* as above for second response buffer */
         if (c == *ptr2)
         {
           ptr2++;

           if (*ptr2 == 0)
           {
             copying = 2;
             optr = resp2;
             ptr2 = resp2;
           }
         }
         else ptr2 = resp2;
       }
     }

#ifdef DEBUG
     Serial.print (c);
#endif
   }
 }
 else
 {
#ifdef DEBUG
   Serial.println ("connection failed");
#endif
 }
 client.stop ();
}

/* End of file */
/*======================================================================*/
