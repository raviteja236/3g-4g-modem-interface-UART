#include<stdio.h>
#include<unistd.h>
#include<sys/mman.h>
#include<sys/select.h>
#include<string.h>
#include<fcntl.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#define AT                       "AT\r\n"
#define AT_NETOPEN		"AT+NETOPEN\r\n"
#define AT_CMQTTSTART		"AT+CMQTTSTART\r\n"
#define AT_CMQTTACCQ		"AT+CMQTTACCQ=0,\"CLIENT TEST0\"\r\n"
#define AT_CMQTTWLTOPIC		"AT+CMQTTWILLTOPIC=0,7\r\n"
#define AT_WLTOPIC		"srtopic"
#define AT_CMQTTWLMSG		"AT+CMQTTWILLMSG=0,6,1\r\n"
#define WLMSG			"qwerty"
#define AT_CMQTTCON		"AT+CMQTTCONNECT=0,\"tcp:\/\/test.mosquitto.org:1883\",60,1\r\n"
#define AT_CMQTTSUB		"AT+CMQTTSUB=0,7,1\r\n"
#define SUB			"srtopic"
#define AT_CMQTTTOPIC		"AT+CMQTTTOPIC=0,7\r\n"
#define TOPIC			"srtopic"
#define AT_CMQTTPAYLOAD		"AT+CMQTTPAYLOAD=0,sizeof(buf1)\r\n"
//#define PAYLOAD			"techsource"
#define AT_CMQTTPUB		"AT+CMQTTPUB=0,1,60\r\n"


/* response codes */
#define AT_OK		1
#define AT_ERR		2

#define DISPLAY_STRING 1
unsigned char result_buf[6500];
unsigned char temp[10];
unsigned int rdIdx;
unsigned short final_result[1254];
unsigned char pcnt=0;

int set_interface_attribs(int fd, int speed)
{
	struct termios tty;

	if (tcgetattr(fd, &tty) < 0) {
		printf("Error from tcgetattr: %s\n", strerror(errno));
		return -1;
	}

	cfsetospeed(&tty, (speed_t)speed);
	cfsetispeed(&tty, (speed_t)speed);

	tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;         /* 8-bit characters */
	tty.c_cflag &= ~PARENB;     /* no parity bit */
	tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
	tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

	/* setup for non-canonical mode */
	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	tty.c_oflag &= ~OPOST;

	/* fetch bytes as they become available */
	tty.c_cc[VMIN] = 1;
	tty.c_cc[VTIME] = 1;

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		printf("Error from tcsetattr: %s\n", strerror(errno));
		return -1;
	}
	return 0;
}

void set_mincount(int fd, int mcount)
{
	struct termios tty;

	if (tcgetattr(fd, &tty) < 0) {
		printf("Error tcgetattr: %s\n", strerror(errno));
		return;
	}

	tty.c_cc[VMIN] = mcount ? 1 : 0;
	tty.c_cc[VTIME] = 5;        /* half second timer */

	if (tcsetattr(fd, TCSANOW, &tty) < 0)
		printf("Error tcsetattr: %s\n", strerror(errno));
}

int Transcive(int sinkfd,int modfd)
{
	char buf[256],modbuf[256];
	fd_set rfds;
	struct timeval tv;
	int ret;
	int timeout_ms = 1000;

	tv.tv_sec = timeout_ms/1000;
	tv.tv_usec = (timeout_ms/1000)*1000;

	FD_ZERO(&rfds);
	//FD_SET(fd,&rfds);
        FD_SET(sinkfd,&rfds);

	ret = select(sinkfd+1,&rfds,0,0,&tv);

	if(ret > 0)
	{
		if(FD_ISSET(sinkfd,&rfds))
		{
			int i;
			i = read(sinkfd,buf,256);
	        	printf("%s\n",buf);
                       	write(modfd,AT_CMQTTTOPIC,strlen(AT_CMQTTTOPIC));
                        sleep(2);
                        write(modfd,TOPIC,strlen(TOPIC));
                         sleep(2);		
			/* send payload command */
			sprintf(modbuf,"AT+CMQTTPAYLOAD=0,%d\r\n",i);
			write(modfd,modbuf,strlen(modbuf));
                        sleep(2);
			/* send payload command */
			write(modfd,buf,i);
                        sleep(2);
                        write(modfd,AT_CMQTTPUB,strlen(AT_CMQTTPUB));
                        sleep(2);
		}
	}
}


void main()
{
	//uart initialization
    char *portname = "/dev/ttyUSB0";
    char *portname1 = "/dev/ttyUSB1";
	int sinkfd,modfd;
	int wlen;
	int rdlen;
	char ctr;
	unsigned int dptr = 0;
	int resp = 0;

	sinkfd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (sinkfd < 0) {
		printf("Error opening %s: %s\n", portname, strerror(errno));
		return -1;
	}
        modfd = open(portname1, O_RDWR | O_NOCTTY | O_SYNC);
	if (modfd < 0) {
		printf("Error opening %s: %s\n", portname, strerror(errno));
		return -1;
	}
	/*baudrate 115200, 8 bits, no parity, 1 stop bit */
	set_interface_attribs(sinkfd, B9600);
	set_interface_attribs(modfd, B115200); 
	
	/* connect data network */
	write(modfd,AT_NETOPEN,strlen(AT_NETOPEN));
        sleep(2);
	write(modfd,AT_CMQTTSTART,strlen(AT_CMQTTSTART));
	sleep(2);
	write(modfd,AT_CMQTTACCQ,strlen(AT_CMQTTACCQ));
	sleep(2);
	write(modfd,AT_CMQTTWLTOPIC,strlen(AT_CMQTTWLTOPIC));
	sleep(2);
	write(modfd,AT_WLTOPIC,strlen(AT_WLTOPIC));
	sleep(2);
	write(modfd,AT_CMQTTWLMSG,strlen(AT_CMQTTWLMSG));
	sleep(2);
	write(modfd,WLMSG,strlen(WLMSG));
	sleep(2);
	write(modfd,AT_CMQTTCON,strlen(AT_CMQTTCON));
	sleep(5);
	write(modfd,AT_CMQTTSUB,strlen(AT_CMQTTSUB));
	sleep(2);
	write(modfd,SUB,strlen(SUB));
	sleep(2);
	printf("read the sink data");
	sleep(1);
	while(1)
    {
		Transcive(sinkfd,modfd);
    }

}
