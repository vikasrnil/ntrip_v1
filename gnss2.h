#ifndef GNSS_H
#define GNSS_H 

#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include <cjson/cJSON.h>
#include <netdb.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#define BAUDRATE B115200
#define DEVICE "/dev/ttyUSB0"
#define BUFSIZE 256
#define PORT 2101
#define BUFFER_SIZE 4096

FILE *gnss_log();
int init_read_port(const char* dev);
uint8_t verifyChecksum(const char *sentence);
void rmc_nmeaparser(char *bu,int *timk,char *va,double *latv,char *l_dir,double *lngv,char *ln_dir,double *sp, double *hea,int *da,char *fa);
void gga_nmeaparser(char *ga,int *timg,double *latg,char *l_dirg,double *lngvg,char *ln_dirg,int *qa, int *nsat,float *hdp,float *alt,int *difa,int *difs);
void convert_time_to_UTC(unsigned int UTC_Time,int *hrk ,int *mink,int *seck);
void log_json_to_file(FILE *file_ptr,char *timj,double latj, double lngj,double altj, double hdopj, double spdj, double headj,int gga_qaj,int gga_numsvj, int gga_diff_agej,int datj,int d_sat);
void close_port(int prt);

int init_ntrip_write_port(const char* sdev);
int NtripSocketInit(char *host,char *httpreq);

double lng_filt(float kef);
double lat_filt(float def);
void gll_nmeaparser(char *bu,int *timk,char *va,double *latv,char *l_dir,double *lngv,char *ln_dir,char *fa);
void write_json_to_file(char const * file,char *timj,double latj, double lngj,double altj, double hdopj, double spdj, double headj,int gga_qaj,int gga_numsvj, int gga_diff_agej,int datj,int d_sat);


#endif
