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
#include <pthread.h>
#include <time.h>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <vector>
#include "jsoncpp/json/json.h" 
#include "zlib.h"
#include "base64.h"

uLong length_after_decompress;
#define DEBUG 0
#define tcdrain(fd) ioctl(fd, TCSBRK, 1); 
#define NSH_SEND_REBOOT_SLEEP_TIME 0
#define ERASE_CHIP_TIME 6
#define REBOOT_SLEEP_TIME 0 //When you send REBOOT command in the time which bootloader is running, The program will sleep.
#define TTYACM_BAUDRATE 57600 //57600 is better than 115200 TODO
#define DATABITS 8
#define STOPBITS 'N'
#define	PARITY 1
#define PORT_ATTRIBUTE R_OK & W_OK & X_OK & F_OK

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

#define   PROG_MULTI_MAX   252  // protocol max is 255, must be multiple of 4
#define   READ_MULTI_MAX   252  // protocol max is 255

uint8_t   NSH_INIT[3] =  {'\x0d', '\x0d', '\x0d'};
uint8_t   GET_SYNC_EOC[2] = {'\x21', '\x20'};
uint8_t   NSH_REBOOT_BL[10] = {'r', 'e', 'b', 'o', 'o', 't', ' ', '-','b', '\n'};
uint8_t   NSH_REBOOT[7] = {'r', 'e', 'b', 'o', 'o', 't','\n'};
uint8_t   BOOT_DELAY[4] = {'\00', '\00', '\00', '\01'}; /*TODO*/

uint8_t   MAVLINK_REBOOT_ID1[41] = {'\xfe', '\x21', '\x72', '\xff', '\x00', '\x4c', '\x00', '\x00', '\x80', '\x3f', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00','\x00', '\x00','\x00', '\x00','\x00', '\x00','\x00', '\x00','\x00', '\x00','\x00', '\xf6', '\x00', '\x01', '\x00', '\x00', '\x48', '\xf0'};

uint8_t   MAVLINK_REBOOT_ID0[41] = {'\xfe', '\x21', '\x45', '\xff', '\x00', '\x4c', '\x00', '\x00', '\x80', '\x3f', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\xf6', '\x00', '\x00', '\x00', '\x00', '\xd7', '\xac'};


char serial_path_splicing_path_str[13] = {};	/*Used to storage the serial name. as example of "/dev/ttyACM0".*/
char* firmware_path = NULL;
//bool val0_inplug, val1_inplug, val2_inplug;
/* The calss used to comunication between the fly control and micro processor.*/

void parse_command(int argc, char *argv[]);
void  serial_path_splicing(const bool ttyACM0_val_replug, const bool ttyACM1_val_replug, const bool ttyACM2_val_replug);
class serial{
	public:
		serial(const char* portname, const int baudrate);
		~serial();

		bool setup_port(const int baud, const int data_bits, const int stop_bits, const bool parity, const bool hardware_control);
		int opendev();
		int fd;
		pthread_mutex_t lock;

	private:
		char* _portname;
};

/* The class used to output the debug messages to debugfile.txt. */
class debugfile{
	public:
		debugfile();
		~debugfile(){
		};
		int debugfd;

	private:
};

/* The class used to handle the json file to image file. */
class firmware:public debugfile{
	public:
		firmware(){
			image = NULL;
			__image == NULL;
		};
		~firmware(){
		};
		void init(); 
		Bytef* __image;

	private:
		uint8_t* image;
		int count;
};

/* The class used to erase and program the sky control. */
class uploader:public serial, debugfile{
	public:
		uploader(const char* portname, int baudrate):serial(portname, baudrate){
			flag_send_reboot = 1;
		};
		~uploader(){
		};

		void __send(const uint8_t c);	
		void __send_array(const uint8_t* buffer, const int length);	
		void __identify();
		void __upload(firmware fw);
		void __close();
		void __init();
		void __send_reboot();
		void __nsh_send_reboot();
		uint8_t __read_datas_tty(int fd, uint8_t *tmp, const int sec, const int usec);
		uint8_t flag_send_reboot;
	private:
		uint8_t __recv();
		void __getSync();
		void __sync();
		void __erase(const char* label);
		void __program_multi(const uint8_t* data, const int length);
		void __program(const char* label, firmware fw);
};


serial::serial(const char* portname, const int baudrate){ /*TODO simplify*/
	if (NULL == portname){
		_portname = new char[1];
		*_portname = '\0';
	}
	else {
		_portname = new char[strlen(portname) + 1];
		memcpy(_portname, portname, strlen(portname) + 1);
	}
	int result = pthread_mutex_init(&lock, NULL);
#if DEBUG
	if (0 != result){
		printf("\n mutex init failed\n");
	}
#endif
	
	return ;
};

serial::~serial(){
#if DEBUG
	printf("serial deconstructor\n");	
#endif 
	delete[] _portname;
	pthread_mutex_destroy(&lock);
	
	return;
};

bool
serial::setup_port(const int baud, const int data_bits, const int stop_bits, const bool parity, const bool hardware_control)
{
	// Check file descriptor
	if (!isatty(fd)){
#if DEBUG
		fprintf(stderr, "\nERROR: file descriptor %d is NOT a serial port\n", fd);
#endif
		return false;
	}

	// Read file descritor configuration
	struct termios  config;
	bzero(&config, sizeof(struct termios));
	if (tcgetattr(fd, &config) < 0){
#if DEBUG
		fprintf(stderr, "\nERROR: could not read configuration of fd %d\n", fd);
#endif
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
	switch (baud){
		case 1200:
			if (cfsetispeed(&config, B1200) < 0 || cfsetospeed(&config, B1200) < 0){
#if DEBUG
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
#endif
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
			if (cfsetispeed(&config, B38400) < 0 || cfsetospeed(&config, B38400) < 0){
#if DEBUG
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
#endif
				return false;
			}
			break;
		case 57600:
			if (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0){
#if DEBUG
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
#endif
				return false;
			}
			break;
		case 115200:
			if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0){
#if DEBUG
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
#endif
				return false;
			}
			break;

			// These two non-standard (by the 70'ties ) rates are fully supported on
			// current Debian and Mac OS versions (tested since 2010).
		case 460800:
			if (cfsetispeed(&config, B460800) < 0 || cfsetospeed(&config, B460800) < 0){
#if DEBUG
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
#endif
				return false;
			}
			break;
		case 921600:
			if (cfsetispeed(&config, B921600) < 0 || cfsetospeed(&config, B921600) < 0){
#if DEBUG
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
#endif
				return false;
			}
			break;
		default:
#if DEBUG
			fprintf(stderr, "ERROR: Desired baud rate %d could not be set, aborting.\n", baud);
#endif
			return false;

			break;
	}

	// Finally, apply the configuration
	if(tcsetattr(fd, TCSAFLUSH, &config) < 0){
#if DEBUG
		fprintf(stderr, "\nERROR: could not set configuration of fd %d\n", fd);
#endif
		return false;
	}

	// Done!
	return true;
}

int 
serial::opendev(){
	do {
		fd = open(serial_path_splicing_path_str, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK); 
		if (fd == -1){
#if DEBUG
			perror("open /dev/ttyACM* error\n");
#endif
		}
		else {
			return fd;
		}
	} while(fd < 0);
};

debugfile::debugfile(){
#if DEBUG
	#if 0
	/*Only run on PC, not Android board.*/
	system("touch /sdcard/debugfile.txt");
	system("chmod 777 /sdcard/debugfile.txt");
	debugfd = open("/sdcard/debugfile.txt", O_RDWR);
	if (debugfd < 0){
		printf("Open debugfile error\n");
		debugfd = 0;
	}
	else {
		printf("Debugfd = %d\n", debugfd);
	}
	#endif
#endif
};

/* Best practive that change json file to image. */
void 
firmware::init(){
	Json::Reader reader;
	Json::Value root;
	ifstream px4file;
	px4file.open(firmware_path, ios::binary);

	if (reader.parse(px4file, root)){

		string decode64_image = gloox::Base64::decode64(root["image"].asString());//64解码之后的字符流。
		count = decode64_image.length();
		char* _image = (char*)malloc((count) * sizeof(char));//解压缩需要使用Bytef*空间，所以定义这么一个空间, 而且zlib处理的是Bytef类型字符流。
#if DEBUG
		if (NULL == _image){
			printf("malloc failed\n");
		}
		else {
			printf("malloc successful\n");
		}
#endif 
		decode64_image.copy(_image, count, 0);//将string类型字符流转变为char*类型。因为copy只能从string到char*转换。0是放置的位置。

		length_after_decompress = root["image_size"].asInt();
		__image = (Bytef*)malloc((length_after_decompress) * sizeof(Bytef));//这是解压缩之后字符流放置的位置。
#if DEBUG
		if (NULL == __image){
			printf("malloc failed\n");
		}
		else {
			printf("malloc successful\n");
		}
#endif
		Z_OK == uncompress(__image, &length_after_decompress, (Bytef*)_image, (uLong)(count));
#if DEBUG
		if (NULL == Z_OK){
			printf("zlib uncompress OK and length_after_decompress = %d\n", (int)length_after_decompress);
		}
		else {
			printf("zlib uncompress NOT OK\n");
		}
#endif
		/*Compare in order to ensure unpress right.*/
		if(length_after_decompress != root["image_size"].asInt()){
			printf("decompress error\n");
			return ;
		}

		free(_image);
		_image = NULL;
#if DEBUG
		#if 0
		/* Only can be runed on PC, TO debug, write the image to the debug file.*/
		int q = 0;
		do {	
			write(debugfd, (char*)__image + q, 1);
			q++;
		} while(q <= length_after_decompress);
		#endif
#endif
	};

	px4file.close();
}




void 
uploader::__init(){
	opendev();
	setup_port(TTYACM_BAUDRATE, DATABITS, PARITY, STOPBITS, false);
	
	return ;
}

void 
uploader::__close(){
	close(fd); 

	return ;
}

void 
uploader::__send(const uint8_t c){
#if DEBUG 
	switch(c) {
		case EOC:
			break;
		case GET_SYNC:
			break;
		case GET_DEVICE:
			break;
		case CHIP_ERASE:
			break;
		case CHIP_VERIFY:
			break;
		case PROG_MULTI:
			break;
		case READ_MULTI: 
			break;
		case GET_CRC: 
			break;
		case GET_OTP: 
			break;
		case GET_SN: 
			break;
		case GET_CHIP: 
			break;
		case SET_BOOT_DELAY: 
			break;
		case REBOOT: 
			break;
		default:
			break;

	}
#endif
	pthread_mutex_lock(&lock);

	int ret = write(fd, &c, 1);
#if DEBUG
	if (ret == -1){
		printf("write error\n");
	} else {
		printf("write right\n");
	}
#endif
	tcdrain(fd);

	pthread_mutex_unlock(&lock);

	return ;
} 
void
uploader::__send_array(const uint8_t* buffer, const int length){

	int bytes_left;   
	int written_bytes;   
	char *ptr;   
	ptr = (char *)buffer;   
	bytes_left = length;   

	while (bytes_left > 0){   
		/* Start to write */
		pthread_mutex_lock(&lock);
		written_bytes = write(fd, ptr, bytes_left);   
		pthread_mutex_unlock(&lock);

		if(written_bytes <= 0){ /* error occur*/
			if(errno == EINTR){ /* Interrupt error, we continue to write*/
				continue;  
#if DEBUG
				printf("[SeanSend]error errno==EINTR continue\n");  
#endif
			}  
			else if(errno == EAGAIN){ /* EAGAIN : Resource temporarily unavailable*/   
				sleep(1);//Wait one second，wait for the send buffer can be freed	
				continue;  
#if DEBUG
				printf("[SeanSend]error errno==EAGAIN continue\n");  
#endif
			}  
			else { /* 其他错误 没有办法,只好退了*/
#if DEBUG
				printf("[SeanSend]ERROR: errno = %d, strerror = %s \n", errno, strerror(errno));  
#endif
				return ;  
			}  
		}  

		bytes_left -= written_bytes;   
		ptr += written_bytes;/* 从剩下的地方继续写?? */
	}   
	tcdrain(fd);

	return;   
}

uint8_t
uploader::__recv(){ 	
	uint8_t tmp = 0;
	__read_datas_tty(fd, &tmp, 10, 10);

	return tmp;
}

uint8_t 
uploader::__read_datas_tty(int fd, uint8_t *tmp, const int sec, const int usec){
	int retval;
	fd_set rfds;
	struct timeval tv;
	int ret, pos;
	tv.tv_sec = sec;//set the rcv wait time
	tv.tv_usec = usec;//100000us = 0.1s
	while(1){
		FD_ZERO(&rfds);
		FD_SET(fd, &rfds);
		retval = select(fd + 1, &rfds, NULL, NULL, &tv);
		if (retval == -1){
			perror("select()");
			break;
		}
		else if (retval){

			pthread_mutex_lock(&lock);
			ret = read(fd, tmp, 1);
#if DEBUG
			if (ret == -1){
				printf("read error\n");
			} else {
				printf("read right, and ret = %d\n", ret);
			}
#endif
			tcdrain(fd);
			pthread_mutex_unlock(&lock);

			return 1;  
		}
		else {
			break;
		}
	}
}

void 
uploader::__identify(){
#if DEBUG
	printf("identify()\n");
#endif
	__sync();
	return ;
}

void 
uploader::__sync(){
	//tcdrain(fd) 等待直到所有的数据全部传输完毕。
	//TCIFLUSH 清除正收到的数据，且不会读取出来。 	
	//TCOFLUSH 清除正写入的数据，且不会发送到终端。 
	//TCIOFLUSH 清除所有正在发生的输入输出的数据。 
	__send(GET_SYNC);	
	__send(EOC);	
	__getSync();
	
	return ;
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
	printf("c[0] = 0x%x\n c[1] = 0x%x\n", c[0], c[1]);
#endif

	if (c[0] != INSYNC){
		flag_send_reboot =  1;
		return ;
	}
	if (c[1] == INVALID){
		flag_send_reboot = 1;
		return ; 
	}
	if (c[1] == FAILED){
		flag_send_reboot = 1;
		return ;
	}
	if (c[1] != OK){
		flag_send_reboot = 1;
		return ; 
	}
	else {
		flag_send_reboot = 0;
	}
#if DEBUG
	switch (c[0]){
		case INSYNC:
			break;
		case EOC:
			break;
		case INVALID:
			break;
		case FAILED:
			break;
		case OK:
			break;
		default:
			break;
	}
	switch(c[1]){
		case INSYNC:
			break;
		case EOC:
			break;
		case INVALID:
			break;
		case FAILED:
			break;
		case OK:
			break;
		default:
			break;
	}
#endif
	return ;
}

void 
uploader::__upload(firmware fw){ 
	__erase("Erase");
	__program("Program", fw);

	return;
}

void 
uploader::__erase(const char* label){
	__send(CHIP_ERASE);
	__send(EOC);
	printf("earsing...wait 6s\n");
	sleep(ERASE_CHIP_TIME); /*给程序擦除留出6s的时间才能保证擦除完成。*/
	printf("earse ok!\n");
	__getSync();
	
	return ;
} 

void
uploader::__program(const char* label, firmware fw){
	uint8_t* code = fw.__image; 
	int i = 0; 
	int left;/*Best practive code*/
	for(i = 0; i < length_after_decompress;){
		left = length_after_decompress - i;
		if(left >= PROG_MULTI_MAX){
			__program_multi(code + i, PROG_MULTI_MAX);
			i += PROG_MULTI_MAX;
		}
		else{
			__program_multi(code + i, left);
			i += left;
		}
		#if 0
		if(i%180)
		printf("%c", '-');
		#endif
	}	
	printf("\n");
	free(fw.__image);
	fw.__image = NULL;

	return ;
}


void
uploader::__program_multi(const uint8_t* data, const int length){
	__send(PROG_MULTI);	
	__send(length);	
	__send_array(data, length);
	__send(EOC);	
	__getSync();	
	
	return ;
}

void 
uploader::__nsh_send_reboot(){
#if DEBUG
	printf("__nsh_send_reboot()\n");
#endif
	__send_array(NSH_INIT, 3);
	__send_array(NSH_REBOOT_BL, 10);
	__send_array(NSH_INIT, 3);
	__send_array(NSH_REBOOT, 7);

	//then try MAVLINK command
	__send_array(MAVLINK_REBOOT_ID1, 41);
	__send_array(MAVLINK_REBOOT_ID0, 41);
	
	return ;
};

void 
uploader::__send_reboot(){
#if DEBUG
	printf("send_reboot()\n");
#endif
	__send(REBOOT);
	__send(EOC);
	__getSync(); //在bootloader中有sync_response()这项，所以应该使用__getSync()去接收回应数据。 
	
	return ;
};

/* splice the serial path name according to the result of the access.*/
void serial_path_access_splicing(){

	int val0_inplug, val1_inplug, val2_inplug;
	val0_inplug = val1_inplug = val2_inplug = 0;
	do {
		val0_inplug = access("/dev/ttyACM0", PORT_ATTRIBUTE);
		val1_inplug = access("/dev/ttyACM1", PORT_ATTRIBUTE);
		val2_inplug = access("/dev/ttyACM2", PORT_ATTRIBUTE);
	} while (val0_inplug != 0 && val1_inplug != 0 && val2_inplug != 0);

	/*If val0_inplug is zero, means "/dev/ttyACM*" is OK.*/
	if (0 == val0_inplug){		
		sprintf(serial_path_splicing_path_str, "%s%s", "/dev/ttyACM", "0");
	} else if (0 == val1_inplug){
		sprintf(serial_path_splicing_path_str, "%s%s", "/dev/ttyACM", "1");
	} else if (0 == val2_inplug){
		sprintf(serial_path_splicing_path_str, "%s%s", "/dev/ttyACM", "2");
	}

	return ;
}

void parse_command(int argc, char *argv[]){	
	/*Caluactor the length of the argv[2], in order to malloc.*/
	int argv2_length = 1; /**/
	char* k = argv[2];
	while(*k != '\0'){
		argv2_length++;
		k++;
	}
	if(3 == argc){
		if (0 == strcmp("--path", argv[1])){
			firmware_path = (char*)malloc(argv2_length);
			if (NULL == firmware_path){
				printf("malloc failed\n");
			} else {
				strcpy(firmware_path, argv[2]);
			};
		}
	} else {
		printf("HELP: /px4_uploader --path filepath.\n");
		exit(-1);
	}

	return ;
}
/*允许bootloader设置启动延迟签名， 它告诉董事会延迟至少一个指定的秒数再启动。 */
int 
main(int argc, char *argv[]) { 
	if(argc < 3){
		printf("Use Method: ./a.out --path pathname\n");
		return 0;
	}

	printf("Ready...\n");
	parse_command(argc, argv);
	/* Forced to restart in order to enter bootloder environment.*/
	serial_path_access_splicing();
	uploader _up = uploader(serial_path_splicing_path_str, TTYACM_BAUDRATE);
	_up.__init();	
	_up.__nsh_send_reboot();	
	sleep(NSH_SEND_REBOOT_SLEEP_TIME);
	_up.__close();

LOOP:	
	serial_path_access_splicing();

	uploader up = uploader(serial_path_splicing_path_str, TTYACM_BAUDRATE);
	up.__init();	
	up.__identify();

	/* If sync with flight control unit serial, reboot the flight control and access the serial port again.*/
	while (up.flag_send_reboot == 1){
		up.__nsh_send_reboot();	
		sleep(NSH_SEND_REBOOT_SLEEP_TIME);
		up.__close();
		goto LOOP;
	};

	/*Base64 code or decode、zlib、json may need more time.*/
	firmware fw = firmware(); 
	fw.init();	

	up.__upload(fw);	
	up.__send_reboot();
	sleep(REBOOT_SLEEP_TIME);
	up.__close();

	free(firmware_path);
	printf("program ok!\n");

	return 0;
}
