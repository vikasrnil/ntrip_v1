#include "gnss.h"
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <arpa/inet.h>
#include <cjson/cJSON.h>

struct termios read_tio, write_tio;
struct sockaddr_in server_addr;
struct hostent *server;

// ---------------- LOG ----------------
FILE *gnss_log(void)
{
    time_t t = time(NULL);
    struct tm *tm_info = localtime(&t);

    char fname[64];
    strftime(fname, sizeof(fname), "log_%Y%m%d_%H%M%S.json", tm_info);

    return fopen(fname, "w");
}

// ---------------- SERIAL ----------------
int init_read_port(const char *device)
{
    int fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) return -1;

    tcgetattr(fd, &read_tio);

    cfsetispeed(&read_tio, GNSS_BAUDRATE);
    cfsetospeed(&read_tio, GNSS_BAUDRATE);

    read_tio.c_cflag |= (CLOCAL | CREAD);
    read_tio.c_cflag &= ~CSIZE;
    read_tio.c_cflag |= CS8;

    tcsetattr(fd, TCSANOW, &read_tio);
    tcflush(fd, TCIOFLUSH);

    return fd;
}

int init_ntrip_write_port(const char *device)
{
    int fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) return -1;

    tcgetattr(fd, &write_tio);

    cfsetispeed(&write_tio, GNSS_BAUDRATE);
    cfsetospeed(&write_tio, GNSS_BAUDRATE);

    write_tio.c_cflag |= (CLOCAL | CREAD);
    write_tio.c_cflag &= ~CSIZE;
    write_tio.c_cflag |= CS8;

    tcsetattr(fd, TCSANOW, &write_tio);
    tcflush(fd, TCIOFLUSH);

    return fd;
}

void close_port(int fd)
{
    close(fd);
}

// ---------------- SOCKET ----------------
int NtripSocketInit(char *host, char *request)
{
    int sockfd;

    while (1)
    {
        sockfd = socket(AF_INET, SOCK_STREAM, 0);
        if (sockfd < 0) continue;

        server = gethostbyname(host);
        if (!server) {
            close(sockfd);
            sleep(2);
            continue;
        }

        memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        memcpy(&server_addr.sin_addr.s_addr, server->h_addr, server->h_length);
        server_addr.sin_port = htons(GNSS_PORT);

        if (connect(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
            close(sockfd);
            sleep(2);
            continue;
        }

        write(sockfd, request, strlen(request));
        return sockfd;
    }
}

// ---------------- CHECKSUM ----------------
uint8_t verifyChecksum(const char *sentence)
{
    if (!sentence || sentence[0] != '$')
        return 0;

    uint8_t calc_sum = 0;
    const char *ptr = sentence + 1;

    // Calculate XOR until '*'
    while (*ptr && *ptr != '*')
    {
        calc_sum ^= *ptr;
        ptr++;
    }

    // No checksum found
    if (*ptr != '*')
        return 0;

    // Move past '*'
    ptr++;

    // Extract received checksum (2 hex chars)
    unsigned int recv_sum;
    if (sscanf(ptr, "%2x", &recv_sum) != 1)
        return 0;

    return (calc_sum == recv_sum);
}

// ---------------- PARSER ----------------
void rmc_nmeaparser(char *buf, int *time, char *valid,
                    double *lat, char *lat_dir,
                    double *lng, char *lng_dir,
                    double *speed, double *heading,
                    int *date, char *fix)
{
    if (!strstr(buf, "RMC")) return;

    char temp[GNSS_BUFSIZE];
    strcpy(temp, buf);

    char *tok = strtok(temp, ",");
    int i = 0;

    while (tok)
    {
        switch(i){
            case 1: *time = atoi(tok); break;
            case 2: *valid = tok[0]; break;
            case 3: *lat = lat_filt(atof(tok)); break;
            case 4: *lat_dir = tok[0]; break;
            case 5: *lng = lng_filt(atof(tok)); break;
            case 6: *lng_dir = tok[0]; break;
            case 7: *speed = atof(tok); break;
            case 8: *heading = atof(tok); break;
            case 9: *date = atoi(tok); break;
            case 10: *fix = tok[0]; break;
        }
        tok = strtok(NULL, ",");
        i++;
    }
}

void gga_nmeaparser(char *buf, int *time,
                    double *lat, char *lat_dir,
                    double *lng, char *lng_dir,
                    int *quality, int *num_sat,
                    float *hdop, float *alt,
                    int *diff_age, int *diff_station)
{
    if (!strstr(buf, "GGA")) return;

    char temp[GNSS_BUFSIZE];
    strcpy(temp, buf);

    char *tok = strtok(temp, ",");
    int i = 0;

    while (tok)
    {
        switch(i){
            case 1: *time = atoi(tok); break;
            case 2: *lat = lat_filt(atof(tok)); break;
            case 3: *lat_dir = tok[0]; break;
            case 4: *lng = lng_filt(atof(tok)); break;
            case 5: *lng_dir = tok[0]; break;
            case 6: *quality = atoi(tok); break;
            case 7: *num_sat = atoi(tok); break;
            case 8: *hdop = atof(tok); break;
            case 9: *alt = atof(tok); break;
            case 13: *diff_age = atoi(tok); break;
            case 14: *diff_station = atoi(tok); break;
        }
        tok = strtok(NULL, ",");
        i++;
    }
}

// ---------------- FILTER ----------------
double lat_filt(float val)
{
    int deg = val / 100;
    return deg + (val - deg * 100) / 60;
}

double lng_filt(float val)
{
    int deg = val / 100;
    return deg + (val - deg * 100) / 60;
}

// ---------------- JSON ----------------
void log_json_to_file(FILE *fp, char *time,
                      double lat, double lng,
                      double alt, double hdop,
                      double speed, double heading,
                      int quality, int num_sat,
                      int diff_age, int date,
                      int station_id)
{
    if (!fp) return;

    cJSON *obj = cJSON_CreateObject();

    cJSON_AddStringToObject(obj,"Time", time);
    cJSON_AddNumberToObject(obj,"Latitude", lat);
    cJSON_AddNumberToObject(obj,"Longitude", lng);

    char *json = cJSON_PrintUnformatted(obj);
    fprintf(fp, "%s\n", json);

    free(json);
    cJSON_Delete(obj);
}
