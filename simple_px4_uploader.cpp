#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <signal.h>
#include <stdint.h>

#include <cstdlib>
#include <fstream>
#include <iostream>

#define DEBUG 1
#define tcdrain(fd) ioctl(fd, TCSBRK, 1); 
#define TIME 0.2

using namespace std;
// protocol bytes
#define  INSYNC            '\x12'
#define  EOC               '\x20'

// reply bytes
#define  OK                '\x10'
#define  FAILED            '\x11'
#define  INVALID   	   '\x13' // rev3+

// command bytes
#define  NOP               '\x00' // guaranteed to be discarded by the bootloader
#define  GET_SYNC          '\x21'
#define  GET_DEVICE        '\x22'
#define  CHIP_ERASE        '\x23'
#define  CHIP_VERIFY       '\x24' // rev2 only
#define  PROG_MULTI        '\x27'
#define  READ_MULTI        '\x28' // rev2 only
#define   GET_CRC          '\x29' // rev3+
#define   GET_OTP          '\x2a' // rev4+  , get a word from OTP area
#define   GET_SN           '\x2b' // rev4+  , get a word from SN area
#define   GET_CHIP         '\x2c' // rev5+  , get chip version
#define   SET_BOOT_DELAY   '\x2d' // rev5+  , set boot delay
#define   REBOOT           '\x30'

#define   INFO_BL_REV      '\x01' //bootloader protocol revision
#define   BL_REV_MIN        2      // minimum supported bootloader protocol 
#define   BL_REV_MAX        4      // maximum supported bootloader protocol 
#define   INFO_BOARD_ID    '\x02' // board type
#define   INFO_BOARD_REV   '\x03' // board revision
#define   INFO_FLASH_SIZE  '\x04' // max firmware size in bytes

#define  PROG_MULTI_MAX   252  // protocol max is 255, must be multiple of 4
#define  READ_MULTI_MAX   252  // protocol max is 255
#define  BOOT_DELAY_TIME  '\x04'

uint8_t NSH_INIT[3] =  {'\x0d', '\x0d', '\x0d'};
uint8_t GET_SYNC_EOC[2] = {'\x21', '\x20'};
uint8_t NSH_REBOOT_BL[10] = {'r', 'e', 'b', 'o', 'o', 't', ' ', '-','b', '\n'};
uint8_t NSH_REBOOT[7] = {'r', 'e', 'b', 'o', 'o', 't','\n'};

#if 1
uint8_t MAVLINK_REBOOT_ID1[41] = {'\xfe', '\x21', '\x72', '\xff', '\x00', '\x4c', '\x00', '\x00', '\x80', '\x3f', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00','\x00', '\x00','\x00', '\x00','\x00', '\x00','\x00', '\x00','\x00', '\x00','\x00', '\xf6', '\x00', '\x01', '\x00', '\x00', '\x48', '\xf0'};

uint8_t MAVLINK_REBOOT_ID0[41] = {'\xfe', '\x21', '\x45', '\xff', '\x00', '\x4c', '\x00', '\x00', '\x80', '\x3f', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\xf6', '\x00', '\x00', '\x00', '\x00', '\xd7', '\xac'};
#endif

int baudrate_arr[] = {B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300, B38400, B19200, B9600, B4800, B2400, B1200, B300, };
int name_arr[] = {115200,38400,  19200,  9600,  4800,  2400,  1200,  300, 38400, 19200,  9600, 4800, 2400, 1200,  300, };

class serial{
	public:
		serial(const char* portname, int baudrate);
		~serial();
		bool _setup_port(int baud, int data_bits, int stop_bits, bool parity, bool hardware_control);

		int OpenDev();
		int getfd(){
			return fd;
		};
	private:
		char* _portname;
		int baudrate;
		int fd;
		int databits; 
		int stopbits;
		int parity;
};

serial::serial(const char* portname, int baudrate){
	if(NULL == portname){
		_portname = new char[1];
		*_portname = '\0';
	}
	else{
		_portname = new char[strlen(portname) + 1];
		memcpy(_portname, portname, strlen(portname)+1);
	}
};

serial::~serial(){
	#if DEBUG
	printf("serial deconstructor\n");	
	#endif 
	delete[] _portname;
};

bool
serial::
_setup_port(int baud, int data_bits, int stop_bits, bool parity, bool hardware_control)
{
	// Check file descriptor
	if(!isatty(fd))
	{
		fprintf(stderr, "\nERROR: file descriptor %d is NOT a serial port\n", fd);
		return false;
	}

	// Read file descritor configuration
	struct termios  config;
	bzero(&config, sizeof(struct termios));
	if(tcgetattr(fd, &config) < 0)
	{
		fprintf(stderr, "\nERROR: could not read configuration of fd %d\n", fd);
		return false;
	}

	// Input flags - Turn off input processing
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
						INLCR | PARMRK | INPCK | ISTRIP | IXON);

	// Output flags - Turn off output processing
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
						 ONOCR | OFILL | OPOST);

	#ifdef OLCUC
		config.c_oflag &= ~OLCUC;
	#endif

	#ifdef ONOEOT
		config.c_oflag &= ~ONOEOT;
	#endif

	// No line processing:
	// echo off, echo newline off, canonical mode off,
	// extended input processing off, signal chars off
	config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	// Turn off character processing
	// clear current char size mask, no parity checking,
	// no output processing, force 8 bit input
	config.c_cflag &= ~(CSIZE | PARENB);
	config.c_cflag |= CS8;

	// One input byte is enough to return from read()
	// Inter-character timer off
	config.c_cc[VMIN]  = 1;
	config.c_cc[VTIME] = 10; // was 0

	// Get the current options for the port
	////struct termios options;
	////tcgetattr(fd, &options);

	// Apply baudrate
	switch (baud)
	{
		case 1200:
			if (cfsetispeed(&config, B1200) < 0 || cfsetospeed(&config, B1200) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 1800:
			cfsetispeed(&config, B1800);
			cfsetospeed(&config, B1800);
			break;
		case 9600:
			cfsetispeed(&config, B9600);
			cfsetospeed(&config, B9600);
			break;
		case 19200:
			cfsetispeed(&config, B19200);
			cfsetospeed(&config, B19200);
			break;
		case 38400:
			if (cfsetispeed(&config, B38400) < 0 || cfsetospeed(&config, B38400) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 57600:
			if (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 115200:
			if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;

		// These two non-standard (by the 70'ties ) rates are fully supported on
		// current Debian and Mac OS versions (tested since 2010).
		case 460800:
			if (cfsetispeed(&config, B460800) < 0 || cfsetospeed(&config, B460800) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 921600:
			if (cfsetispeed(&config, B921600) < 0 || cfsetospeed(&config, B921600) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		default:
			fprintf(stderr, "ERROR: Desired baud rate %d could not be set, aborting.\n", baud);
			return false;

			break;
	}

	// Finally, apply the configuration
	if(tcsetattr(fd, TCSAFLUSH, &config) < 0)
	{
		fprintf(stderr, "\nERROR: could not set configuration of fd %d\n", fd);
		return false;
	}

	// Done!
	return true;
}

int serial::OpenDev(){
	do{
		fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK); 
		if (fd == -1){
			perror("open /dev/ttyACM0 error\n");
		}
		else{
			return fd;
		}


		fd = open("/dev/ttyACM1", O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK); 
		if (fd == -1){
			perror("open /dev/ttyACM1 error\n");
		}
		else{
			return fd;
		}


		fd = open("/dev/ttyACM2", O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK); 
		if (fd == -1){
			perror("open /dev/ttyACM2 error\n");
		}
		else{
			return fd;
		};
	}while(fd < 0);
	return 0;
};


class uploader:public serial{
	public:
		uploader(const char* portname, int baudrate):serial(portname, baudrate){
			flag_send_reboot = 1;
		};
		~uploader(){
		};

		void __send(uint8_t c);	
		void __send_array(const uint8_t* buffer, int length);	
		void identify();
		void _close();
		void __init__();
		void send_reboot();
		void nsh_send_reboot();
		uint8_t flag_send_reboot;
		uint8_t read_datas_tty(int fd, uint8_t *tmp, int sec, int usec);
		void __set_boot_delay();/*self, boot_delay*/
	private:
		uint8_t __recv();
		void __getSync();
		void __sync();
};


void 
uploader::__init__(){

	int databits = 8; 
	int stopbits = 'N';
	int parity = 1;

	OpenDev();
	_setup_port(115200, databits, parity, stopbits, false);
}

void 
uploader::_close(){
	close(getfd()); 
}

void 
uploader::__send(const uint8_t c){
	switch(c){
		case EOC:
			#if DEBUG
			printf("__send() : send EOC\n");
			#endif
			break;
		case GET_SYNC:
			#if DEBUG
			printf("__send() : send GET_SYNC\n");
			#endif
			break;
		case GET_DEVICE:
			#if DEBUG
			printf("__send() : send GET_DEVICE\n");
			#endif
			break;
		case CHIP_ERASE:
			#if DEBUG
			printf("__send() : send CHIP_ERASE\n");
			#endif
			break;
		case CHIP_VERIFY:
			#if DEBUG
			printf("__send() : send CHIP_VERIFY\n");
			#endif
			break;
		case PROG_MULTI:
			#if DEBUG
			printf("__send() : send PROG_MULTI\n");
			#endif
			break;
		case READ_MULTI: 
			#if DEBUG
			printf("__send() : send READ_MULTI\n");
			#endif
			break;
		case GET_CRC: 
			#if DEBUG
			printf("__send() : send GET_CRC\n");
			#endif
			break;
		case GET_OTP: 
			#if DEBUG
			printf("__send() : send GET_OTP\n");
			#endif
			break;
		case GET_SN: 
			#if DEBUG
			printf("__send() : send GET_SN\n");
			#endif
			break;
		case GET_CHIP: 
			#if DEBUG
			printf("__send() : send GET_CHIP\n");
			#endif
			break;
		case SET_BOOT_DELAY: 
			#if DEBUG
			printf("__send() : send SET_BOOT_DELAY\n");
			#endif
			break;
		case REBOOT: 
			#if DEBUG
			printf("__send() : send REBOOT\n");
			#endif
			break;
		default:
			break;

	}
	int ret = write(getfd(), &c, 1);
	if(ret == -1){
		printf("write error\n");
	}else {
		printf("write right, and ret = %d\n", ret);
	}

	tcdrain(getfd());
	printf("getfd() = %d\n", getfd());
} 
void
uploader::__send_array(const uint8_t* buffer, const int length){
	int bytes_left;   
	int written_bytes;   
	char *ptr;   
	ptr=(char *)buffer;   
	bytes_left=length;   
	while(bytes_left>0){   
		/* 开始写*/
		written_bytes=write(getfd(), ptr, bytes_left);   
		if(written_bytes<=0) /* 出错了*/
		{   
			if(errno==EINTR) /* 中断错误 我们继续写*/
			{  
				continue;  
				printf("[SeanSend]error errno==EINTR continue\n");  
			}  
			else if(errno==EAGAIN) /* EAGAIN : Resource temporarily unavailable*/   
			{  
				sleep(1);//等待一秒，希望发送缓冲区能得到释放
				continue;  
				printf("[SeanSend]error errno==EAGAIN continue\n");  
			}  
			else /* 其他错误 没有办法,只好退了*/
			{  
				printf("[SeanSend]ERROR: errno = %d, strerror = %s \n"  
						, errno, strerror(errno));  
				return ;  
			}  
		}  
		bytes_left-=written_bytes;   
		ptr+=written_bytes;/* 从剩下的地方继续写?? */
	}   
	tcdrain(getfd());

	return;   
}

uint8_t
uploader::__recv(){ 	

	uint8_t tmp = 0;
	read_datas_tty(getfd(), &tmp, 10, 10);

	return tmp;
}

uint8_t 
uploader::read_datas_tty(int fd, uint8_t *tmp, int sec, int usec){
	int retval;
	fd_set rfds;
	struct timeval tv;
	int ret;
	tv.tv_sec = sec;//set the rcv wait time
	tv.tv_usec = usec;//100000us = 0.1s

	while(1){
		printf("selecting\n");
		FD_ZERO(&rfds);
		FD_SET(fd, &rfds);
		retval = select(fd+1, &rfds, NULL, NULL, &tv);
		if(retval == -1){
			perror("select()");
			break;
		}
		else if(retval){
			ret = read(fd, tmp,1);
			if(ret == -1){
				printf("read error\n");
			}else{
				printf("read right, and ret = %d\n", ret);
			}
			tcdrain(fd);
			return 1;  
		}
		else{
			break;
		}
	}
	return 1;
}

void 
uploader::identify(){
#if DEBUG
	printf("identify()\n");
#endif
	__sync();

}

void 
uploader::__sync(){
	__send(GET_SYNC);	
	__send(EOC);	
	__getSync();
}

void 
uploader::__getSync(){
#if DEBUG
	printf("__getSync\n");
#endif
	uint8_t c[2];
	c[0] = __recv();
	c[1] = __recv();
#if DEBUG
	printf("c[0] = 0x%x\n", c[0]);
	printf("c[1] = 0x%x\n", c[1]);
#endif

	if(c[0] != INSYNC){
		flag_send_reboot =  1;
		return ;
	}
	if(c[1] == INVALID){
		flag_send_reboot = 1;
		return ; 
	}
	if(c[1] == FAILED){
		flag_send_reboot = 1;
		return ;
	}
	if(c[1] != OK){
		flag_send_reboot = 1;
		return ; 
	}
	else{
		flag_send_reboot = 0;
	}
	switch(c[0]){
		case INSYNC:
#if DEBUG
			printf("INSYNC received\n");
#endif
			break;
		case EOC:
#if DEBUG
			printf("EOC received\n");
#endif
			break;
		case INVALID:
#if DEBUG
			printf("INVALID received\n");
#endif
			break;
		case FAILED:
#if DEBUG
			printf("FAILED received\n");
#endif
			break;
		case OK:
#if DEBUG
			printf("OK received\n");
#endif
			break;
		default:
#if DEBUG
			printf("__getSync() c[0] undefined, c[0] = 0x%x\n", c[0]);
#endif
			break;
	}
	switch(c[1]){
		case INSYNC:
#if DEBUG
			printf("INSYNC received\n");
#endif
			break;
		case EOC:
#if DEBUG
			printf("EOC received\n");
#endif
			break;
		case INVALID:
#if DEBUG
			printf("INVALID received\n");
#endif
			break;
		case FAILED:
#if DEBUG
			printf("FAILED received\n");
#endif
			break;
		case OK:
#if DEBUG
			printf("OK received\n");
#endif
			break;
		default:
#if DEBUG
			printf("__getSync() c[1] undefined, c[1] = 0x%x\n", c[1]);
#endif
			break;
	}

}



void 
uploader::nsh_send_reboot(){
#if DEBUG
	printf("nsh_send_reboot()\n");
#endif
	__send_array(NSH_INIT, 3);
	__send_array(NSH_REBOOT_BL, 10);
	__send_array(NSH_INIT, 3);
	__send_array(NSH_REBOOT, 7);

	//then try MAVLINK command
	__send_array(MAVLINK_REBOOT_ID1, 41);
	__send_array(MAVLINK_REBOOT_ID0, 41);
};

void 
uploader::send_reboot(){
#if DEBUG
	printf("send_reboot()\n");
#endif
	__send(REBOOT);
	__send(EOC);
	//__getSync();
	
};

void 
uploader::__set_boot_delay(){
	tcdrain(getfd());
	__send(SET_BOOT_DELAY);
	__send(BOOT_DELAY_TIME);
	__send(EOC);
	__getSync();
};

/*允许bootloader设置启动延迟签名， 它告诉董事会延迟至少一个指定的秒数再启动。 */
int 
main(int argc, char *argv[]){ 
	uploader up = uploader("/dev/ttyACM0", 115200);
	up.__init__();	
	while(1){
		sleep(2);
		up.identify();
	}
	up._close();

	return 0;
}
