#ifndef GNSS_H
#define GNSS_H

#include <stdint.h>
#include <termios.h>
#include <stdio.h>
#include <netdb.h>

// ---------------- CONFIG ----------------
#define GNSS_BAUDRATE      B115200
#define GNSS_DEVICE        "/dev/ttyUSB0"
#define GNSS_BUFSIZE       256
#define GNSS_BUFFER_SIZE   4096
#define GNSS_PORT          2101

// ---------------- FILE ----------------
FILE *gnss_log(void);

// ---------------- SERIAL ----------------
int init_read_port(const char *device);
int init_ntrip_write_port(const char *device);
void close_port(int fd);

// ---------------- SOCKET ----------------
int NtripSocketInit(char *host, char *request);

// ---------------- PARSER ----------------
uint8_t verifyChecksum(const char *sentence);

void rmc_nmeaparser(char *buf,
                    int *time, char *valid,
                    double *lat, char *lat_dir,
                    double *lng, char *lng_dir,
                    double *speed, double *heading,
                    int *date, char *fix);

void gga_nmeaparser(char *buf,
                    int *time,
                    double *lat, char *lat_dir,
                    double *lng, char *lng_dir,
                    int *quality, int *num_sat,
                    float *hdop, float *alt,
                    int *diff_age, int *diff_station);

// ---------------- FILTER ----------------
double lat_filt(float val);
double lng_filt(float val);

// ---------------- JSON ----------------
void log_json_to_file(FILE *fp,
                      char *time,
                      double lat, double lng,
                      double alt, double hdop,
                      double speed, double heading,
                      int quality, int num_sat,
                      int diff_age, int date,
                      int station_id);

#endif
