/*
 * ======================================================================
 * SonosIR - Arduino sketch for infrared control of Sonos ZonePlayer (c) 2010
 * Simon Long, KuDaTa Software
 * 
 * Original Sonos control Arduino sketch by Jan-Piet Mens. Uses the Arduino
 * IRremote library by Ken Shirriff.
 * 
 * Use is at the user's own risk - no warranty is expressed or implied.
 * ======================================================================
 *
 * Changes:
 *	2010-04-24 simon added volume control
 */

#include <Client.h>
#include <Ethernet.h>
#include <IRremote.h>

/*----------------------------------------------------------------------*/
/* Macros and constants */
/*----------------------------------------------------------------------*/

/* Sonos SOAP command packet skeleton */
#define SONOS_CMDH "<s:Envelope xmlns:s=\"http://schemas.xmlsoap.org/soap/envelope/\" s:encodingStyle=\"http://schemas.xmlsoap.org/soap/encoding/\"><s:Body>"
#define SONOS_CMDP " xmlns:u=\"urn:schemas-upnp-org:service:"
#define SONOS_CMDQ ":1\"><InstanceID>0</InstanceID>"
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

/* State machine for A-B repeat and intro scan functions */
#define MODE_NORMAL 0
#define MODE_SCAN   1
#define MODE_A      2
#define MODE_AB     3

/* Arduino pin to which IR receiver is connected */
#define IR_PIN 8

/* IRremote hex codes for received remote commands */
//#define REMOTE_PLAY      0x35
//#define REMOTE_PAUSE     0x30
//#define REMOTE_PREV      0x21
//#define REMOTE_NEXT      0x20
#define REMOTE_SHUFFLE   0x1C
#define REMOTE_REPEAT    0x1D
#define REMOTE_AB        0x3B
#define REMOTE_SCAN      0x2B
#define REMOTE_REV       0x32
#define REMOTE_FWD       0x34
//#define REMOTE_VOLU      0x10
//#define REMOTE_VOLD      0x11

//Thomson universal set to Philips code = 0200
// 0C 0D
// 20 10
// 21 11

#define REMOTE_PLAY     0x0C
#define REMOTE_PAUSE    0x0D
#define REMOTE_NEXT     0x20
#define REMOTE_VOLU     0x10
#define REMOTE_PREV     0x21
#define REMOTE_VOLD     0x11


/* IP addresses of Arduino and ZonePlayer */
#define IP1     192
#define IP2     168
#define IP3     1
#define IP4ZP   132 /* Office */
#define IP4ARD  141 /* Arduino */

/* Enable DEBUG for serial debug output */
//
//#define DEBUG

/*----------------------------------------------------------------------*/
/* Global variables */
/*----------------------------------------------------------------------*/

/* IP address of ZonePlayer to control */
byte sonosip[] = {
	IP1, IP2, IP3, IP4ZP
};

/* Millisecond timer values */
unsigned long   lastcmd = 0;
unsigned long   lastrew = 0;
unsigned long   lastpoll = 0;

/* A-B repeat and intro scan state */
int             mode;

/* Second timer values used for A-B repeat */
int             posa, posb;

/* Global used to store number of seconds to seek to in a Sonos command */
int             desttime;

/* Global used for volume setting */
int             newvol;

/* Buffers used for Sonos data reception */
char            data1[20];
char            data2[20];

/* Global null buffer used to disable data reception */
char            nullbuf[1] = {0};

/* IR receiver control */
IRrecv          irrecv(IR_PIN);
decode_results  results;

/* Ethernet control */
Client          client(sonosip, 1400);

/*----------------------------------------------------------------------*/
/* Function defintions */
/*----------------------------------------------------------------------*/

/*----------------------------------------------------------------------*/
/* setup - configures Arduino */

void 
setup()
{
	uint8_t         mac[6] = {0xBE, 0xEF, 0xEE, 0x00, 0x20, 0x09};
	uint8_t         ip[4] = {IP1, IP2, IP3, IP4ARD};

	delay(3000);

	/* initialise Ethernet connection */
	Ethernet.begin(mac, ip);

	/* initialise IR receiver */
	irrecv.enableIRIn();

	/* disable intro scan and A-B repeat */
	mode = MODE_NORMAL;

#ifdef DEBUG
	Serial.begin(9600);
#endif
}

/*----------------------------------------------------------------------*/
/* loop - called repeatedly by Arduino */

void 
loop()
{
	int             res;

	/* look to see if a packet has been received from the IR receiver */
	if (irrecv.decode(&results)) {
#ifdef DEBUG
		Serial.println(results.value, HEX);
#endif

		/*
		 * only process if this is the first packet for 200ms -
		 * prevents multiple operations
		 */
		if (millis() > (lastcmd + 200)) {
			/* compare received IR against known commands */
			switch (results.value & 0xFF) {
			case REMOTE_SCAN:
				if (mode == MODE_SCAN)
					mode = MODE_NORMAL;
				else {
					mode = MODE_SCAN;
					sonos(SONOS_POSIT, data1, nullbuf);
					if (seconds(data1 + 1) > 10)
						sonos(SONOS_NEXT, nullbuf, nullbuf);
				}
				break;

			case REMOTE_AB:
				if (mode == MODE_AB)
					mode = MODE_NORMAL;
				else if (mode == MODE_A) {
					mode = MODE_AB;
					sonos(SONOS_POSIT, data1, nullbuf);
					posb = seconds(data1 + 1);
				} else {
					mode = MODE_A;
					sonos(SONOS_POSIT, data1, nullbuf);
					posa = seconds(data1 + 1);
				}
				break;

			case REMOTE_PLAY:
				mode = MODE_NORMAL;
				sonos(SONOS_PLAY, nullbuf, nullbuf);
				break;

			case REMOTE_PAUSE:
				mode = MODE_NORMAL;
				sonos(SONOS_PAUSE, nullbuf, nullbuf);
				break;

			case REMOTE_NEXT:
				mode = MODE_NORMAL;
				sonos(SONOS_NEXT, nullbuf, nullbuf);
				break;

			case REMOTE_PREV:
				mode = MODE_NORMAL;
				if (millis() > lastrew + 2000) {
					desttime = 0;
					sonos(SONOS_SEEK, nullbuf, nullbuf);
				} else
					sonos(SONOS_PREV, nullbuf, nullbuf);
				lastrew = millis();
				break;

			case REMOTE_SHUFFLE:
				mode = MODE_NORMAL;
				sonos(SONOS_MODE, data1, nullbuf);
				res = sum_letters(data1 + 1);
				if (res == 457)
					sonos(SONOS_SHUFF, nullbuf, nullbuf);
				//NORMAL
					if (res == 525)
					sonos(SONOS_REPEAT, nullbuf, nullbuf);
				//SHUFFLE
					if (res == 761)
					sonos(SONOS_SHUREP, nullbuf, nullbuf);
				//REPEAT_ALL
					if (res == 1226)
					sonos(SONOS_NORMAL, nullbuf, nullbuf);
				//SHUFFLE_NOREPEAT
					break;

			case REMOTE_REPEAT:
				mode = MODE_NORMAL;
				sonos(SONOS_MODE, data1, nullbuf);
				res = sum_letters(data1 + 1);
				if (res == 457)
					sonos(SONOS_REPEAT, nullbuf, nullbuf);
				//NORMAL
					if (res == 525)
					sonos(SONOS_SHUFF, nullbuf, nullbuf);
				//SHUFFLE
					if (res == 761)
					sonos(SONOS_NORMAL, nullbuf, nullbuf);
				//REPEAT_ALL
					if (res == 1226)
					sonos(SONOS_SHUREP, nullbuf, nullbuf);
				//SHUFFLE_NOREPEAT
					break;

			case REMOTE_REV:
				mode = MODE_NORMAL;
				sonos(SONOS_POSIT, data1, nullbuf);
				res = seconds(data1 + 1);
				res -= 10;
				if (res > 0) {
					desttime = res;
					sonos(SONOS_SEEK, nullbuf, nullbuf);
				}
				break;

			case REMOTE_FWD:
				mode = MODE_NORMAL;
				strcpy(data2, "TrackDuration");
				sonos(SONOS_POSIT, data1, data2);
				res = seconds(data1 + 1);
				res += 10;
				if (res < seconds(data2 + 1)) {
					desttime = res;
					sonos(SONOS_SEEK, nullbuf, nullbuf);
				}
				break;

			case REMOTE_VOLU:
				sonos(SONOS_GETVOL, data1, nullbuf);
				sscanf(data1 + 1, "%d", &newvol);
				newvol += 5;
				if (newvol > 100)
					newvol = 100;
				sonos(SONOS_SETVOL, nullbuf, nullbuf);
				break;

			case REMOTE_VOLD:
				sonos(SONOS_GETVOL, data1, nullbuf);
				sscanf(data1 + 1, "%d", &newvol);
				newvol -= 5;
				if (newvol < 0)
					newvol = 0;
				sonos(SONOS_SETVOL, nullbuf, nullbuf);
				break;
			}

			/* store time at which last IR command was processed */
			lastcmd = millis();
		}
		/* get ready to receive next IR command */
		irrecv.resume();
	}
	/* processing for intro scan and A-B repeat modes */
	if (mode != MODE_NORMAL) {
		/*
		 * if in intro scan or A-B, poll the current playback
		 * position every 0.5s
		 */
		if (millis() > lastpoll + 500) {
			sonos(SONOS_POSIT, data1, nullbuf);
			res = seconds(data1 + 1);
			if (mode == MODE_SCAN) {
				/*
				 * intro scan - if current time is greater
				 * than 10 seconds, skip forward
				 */
				if (res >= 10) {
					strcpy(data1, "fault");
					sonos(SONOS_NEXT, data1, nullbuf);

					/*
					 * if skip returned an error, at end
					 * of playlist - cancel scan
					 */
					if (data1[0] != 'f')
						mode = MODE_NORMAL;
				}
			}
			if (mode == MODE_AB) {
				/*
				 * A-B repeat - if current time is greater
				 * than end time, skip to start time
				 */
				if (res >= posb) {
					desttime = posa;
					sonos(SONOS_SEEK, nullbuf, nullbuf);
				}
			}
			if (mode == MODE_A) {
				/*
				 * A pressed - waiting for B - abort if track
				 * end reached
				 */
				if (res < posa)
					mode = MODE_NORMAL;
			}
			/* update playback position timer */
			lastpoll = millis();
		}
	}
}

/*----------------------------------------------------------------------*/
/* seconds - converts supplied string in format hh:mm:ss to seconds */

int 
seconds(char *str)
{
	int             hrs, mins, secs;
	sscanf(str, "%d:%d:%d", &hrs, &mins, &secs);
	hrs *= 60;
	hrs += mins;
	hrs *= 60;
	hrs += secs;
	return hrs;
}

/*----------------------------------------------------------------------*/
/* sum_letters - adds ASCII codes for all letters in supplied string */

int 
sum_letters(char *str)
{
	int             tot = 0;
	char           *ptr = str;
	while (*ptr) {
		tot += *ptr;
		ptr++;
	}
	return tot;
}

/*----------------------------------------------------------------------*/
/* out - outputs supplied string to Ethernet client */

void 
out(const char *s)
{
	client.println(s);
#ifdef DEBUG
	Serial.println(s);
#endif
}

/*----------------------------------------------------------------------*/
/* sonos - sends a command packet to the ZonePlayer */

void 
sonos(int cmd, char *resp1, char *resp2)
{
	//char          buf[512];
	char            buf[350];
	char            cmdbuf[32];
	char            extra[64];
	char            service[20];
	char           *ptr1;
	char           *ptr2;
	char           *optr;
	char            copying;
	unsigned long   timeout;

	extra[0] = 0;
	strcpy(service, "AVTransport");

	if (client.connect()) {
#ifdef DEBUG
		Serial.println("connected");
#endif

		/*
		 * prepare the data strings to go into the desired command
		 * packet
		 */
		switch (cmd) {
		case SONOS_PLAY:
			strcpy(cmdbuf, "Play");
			strcpy(extra, "<Speed>1</Speed>");
			break;

		case SONOS_PAUSE:
			strcpy(cmdbuf, "Pause");
			break;

		case SONOS_PREV:
			strcpy(cmdbuf, "Previous");
			break;

		case SONOS_NEXT:
			strcpy(cmdbuf, "Next");
			break;

		case SONOS_SEEK:
			strcpy(cmdbuf, "Seek");
			sprintf(extra, "<Unit>REL_TIME</Unit><Target>%02d:%02d:%02d</Target>", desttime / 3600, (desttime / 60) % 60, desttime % 60);
			break;

		case SONOS_NORMAL:
		case SONOS_REPEAT:
		case SONOS_SHUFF:
		case SONOS_SHUREP:
			if (cmd == SONOS_NORMAL)
				strcpy(cmdbuf, "NORMAL");
			if (cmd == SONOS_REPEAT)
				strcpy(cmdbuf, "REPEAT_ALL");
			if (cmd == SONOS_SHUFF)
				strcpy(cmdbuf, "SHUFFLE_NOREPEAT");
			if (cmd == SONOS_SHUREP)
				strcpy(cmdbuf, "SHUFFLE");
			sprintf(extra, "<NewPlayMode>%s</NewPlayMode>", cmdbuf);
			strcpy(cmdbuf, "SetPlayMode");
			break;

		case SONOS_MODE:
			strcpy(cmdbuf, "GetTransportSettings");
			strcpy(resp1, "PlayMode");
			break;

		case SONOS_POSIT:
			strcpy(cmdbuf, "GetPositionInfo");
			strcpy(resp1, "RelTime");
			break;

		case SONOS_GETVOL:
			strcpy(cmdbuf, "GetVolume");
			strcpy(extra, "<Channel>Master</Channel>");
			strcpy(service, "RenderingControl");
			strcpy(resp1, "CurrentVolume");
			break;

		case SONOS_SETVOL:
			strcpy(cmdbuf, "SetVolume");
			sprintf(extra, "<Channel>Master</Channel><DesiredVolume>%d</DesiredVolume>", newvol);
			strcpy(service, "RenderingControl");
			break;
		}

		/* output the command packet */
		sprintf(buf, "POST /MediaRenderer/%s/Control HTTP/1.1", service);
		out(buf);
		out("Connection: close");
		sprintf(buf, "Host: %d.%d.%d.%d:1400", sonosip[0], sonosip[1], sonosip[2], sonosip[3]);
		out(buf);
		sprintf(buf, "Content-Length: %d", 231 + 2 * strlen(cmdbuf) + strlen(extra) + strlen(service));
		out(buf);
		out("Content-Type: text/xml; charset=\"utf-8\"");
		sprintf(buf, "Soapaction: \"urn:schemas-upnp-org:service:%s:1#%s\"", service, cmdbuf);
		out(buf);
		out("");
		sprintf(buf, "%s<u:%s%s%s%s%s</u:%s>%s", SONOS_CMDH, cmdbuf, SONOS_CMDP, service, SONOS_CMDQ, extra, cmdbuf, SONOS_CMDF);
		out(buf);

		/* wait for a response packet */
		timeout = millis();
		while ((!client.available()) && ((millis() - timeout) < 1000));

		/*
		 * parse the response looking for the strings in resp1 and
		 * resp2
		 */
		ptr1 = resp1;
		ptr2 = resp2;
		copying = 0;
		while (client.available()) {
			char            c = client.read();

			/*
			 * if response buffers start with nulls, either no
			 * response required, or already received
			 */
			if (resp1[0] || resp2[0]) {
				/*
				 * if a response has been identified, copy
				 * the data
				 */
				if (copying) {
					/*
					 * look for the < character that
					 * indicates the end of the data
					 */
					if (c == '<') {
						/*
						 * stop receiving data, and
						 * null the first character
						 * in the response buffer
						 */
						copying = 0;
						*optr = 0;
						if (copying == 1)
							resp1[0] = 0;
						else
							resp2[0] = 0;
					} else {
						/*
						 * copy the next byte to the
						 * response buffer
						 */
						*optr = c;
						optr++;
					}
				} else {
					/*
					 * look for input characters that
					 * match the response buffers
					 */
					if (c == *ptr1) {
						/*
						 * character matched -
						 * advance to next character
						 * to match
						 */
						ptr1++;

						/*
						 * is this the end of the
						 * response buffer
						 */
						if (*ptr1 == 0) {
							/*
							 * string matched -
							 * start copying from
							 * next character
							 * received
							 */
							copying = 1;
							optr = resp1;
							ptr1 = resp1;
						}
					} else
						ptr1 = resp1;

					/*
					 * as above for second response
					 * buffer
					 */
					if (c == *ptr2) {
						ptr2++;

						if (*ptr2 == 0) {
							copying = 2;
							optr = resp2;
							ptr2 = resp2;
						}
					} else
						ptr2 = resp2;
				}
			}
#ifdef DEBUG
			Serial.print(c);
#endif
		}
	} else {
#ifdef DEBUG
		Serial.println("connection failed");
#endif
	}
	client.stop();
}

/* End of file */
/* ====================================================================== */
