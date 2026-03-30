#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <sys/socket.h>
#include <pthread.h>
#include "gnss.h"

extern struct hostent *server;

// ---------------- MUTEX ----------------
pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t buffer_mutex = PTHREAD_MUTEX_INITIALIZER;

// ---------------- CONFIG ----------------
char *hst = "qrtksa1.quectel.com";   

char *get_request ="GET /AUTO_ITRF2020 HTTP/1.1\r\nHost: qrtksa1.quectel.com\r\nNtrip-Version: Ntrip/2.0\r\nUser-Agent: PythonNTRIP\r\nAuthorization: Basic ZXNjb3J0c2t1Ym90YV8wMDAwMDAxOm05amRlYjZ6\r\nConnection: keep-alive\r\n\r\n";


// =====================================================
//            FILE WRITE
// =====================================================
void write_gga_to_file(const char *filename, const char *gga)
{
    pthread_mutex_lock(&buffer_mutex);

    FILE *file = fopen(filename, "w");
    if (file == NULL) {
        perror("Error opening GGA file");
        pthread_mutex_unlock(&buffer_mutex);
        return;
    }

    fprintf(file, "%s", gga);
    fclose(file);

    pthread_mutex_unlock(&buffer_mutex);
}

// =====================================================
//            FILE READ
// =====================================================
char* read_gga_from_file(const char *filename)
{
    pthread_mutex_lock(&buffer_mutex);

    FILE *file = fopen(filename, "r");
    if (file == NULL) {
        pthread_mutex_unlock(&buffer_mutex);
        return NULL;
    }

    static char buffer[256];

    if (fgets(buffer, sizeof(buffer), file) == NULL) {
        fclose(file);
        pthread_mutex_unlock(&buffer_mutex);
        return NULL;
    }

    fclose(file);
    pthread_mutex_unlock(&buffer_mutex);

    return buffer;
}

// =====================================================
//            SEND GGA TO NTRIP
// =====================================================
void ntrip_gga(int sockfd, const char *filename)
{
    char *gga = read_gga_from_file(filename);

    if (gga != NULL) {
        printf("NTRIP GGA: %s\n", gga);
        write(sockfd, gga, strlen(gga));
    }
}

// =====================================================
//            NTRIP THREAD
// =====================================================
void *ntrip_dev(void *arg)
{
    int wprt, sckfd, n;
    char buffer[BUFFER_SIZE];

    wprt = init_ntrip_write_port(DEVICE);
    sckfd = NtripSocketInit(hst, get_request);

    int reconnect_count = 1;

while (1)
{
    ntrip_gga(sckfd, "ntrip_gga.txt");

    bzero(buffer, BUFFER_SIZE);

    n = read(sckfd, buffer, BUFFER_SIZE);

    if (n > 0)
    {
        write(wprt, buffer, n);
        printf("Wrote %d bytes\n", n);
    }

    if (n <= 0)
    {
        printf("Reconnecting... Attempt: %d\n", reconnect_count++);
        close(sckfd);
        sckfd = NtripSocketInit(hst, get_request);
    }

    usleep(500000); 
}
    close(sckfd);
    pthread_exit(NULL);
}

// =====================================================
//            SERIAL GNSS THREAD
// =====================================================
void *serial_reader_thread(void *arg)
{
    int hr, se, mi;
    char gnss_t[64];
    int res;

    int tim = 0, dat = 0, qua, nsati, diff_age, diff_sat;
    double lat, lng, spd, hd;
    float alti, hdop;
    char valid, ltdir, lngdir, fixt;

    char buf[BUFSIZE];

    int fd = init_read_port(DEVICE);

    while ((res = read(fd, buf, sizeof(buf) - 1)) > 0)
    {
        buf[res] = '\0';

        if (verifyChecksum(buf))
        {
            if (strstr(buf, "$GNGGA") || strstr(buf, "$GPGGA"))
            {
                write_gga_to_file("ntrip_gga.txt", buf);
            }

            // -------- PARSING ----------
            rmc_nmeaparser(buf, &tim, &valid, &lat, &ltdir,
                           &lng, &lngdir, &spd, &hd, &dat, &fixt);

            gga_nmeaparser(buf, &tim, &lat, &ltdir, &lng, &lngdir,
                           &qua, &nsati, &hdop, &alti, &diff_age, &diff_sat);

            if (lat != 0.0 && lng != 0.0 && alti != 0.0)
            {
                convert_time_to_UTC((unsigned)tim, &hr, &mi, &se);

                sprintf(gnss_t, "%02d:%02d:%02d", hr, mi, se);

                write_json_to_file("dev.json", gnss_t, lat, lng, alti,
                                   hdop, spd, hd, qua, nsati,
                                   diff_age, dat, diff_sat);

                usleep(5000);
            }
        }
    }

    close_port(fd);
    pthread_exit(NULL);
}

// =====================================================
//                    MAIN
// =====================================================
int main()
{
    pthread_t tid, wid;

    pthread_mutex_init(&lock, NULL);
    pthread_mutex_init(&buffer_mutex, NULL);

    // NTRIP thread
    if (pthread_create(&tid, NULL, ntrip_dev, NULL) != 0)
    {
        perror("NTRIP thread creation failed");
        return 1;
    }

    // GNSS serial thread
    if (pthread_create(&wid, NULL, serial_reader_thread, NULL) != 0)
    {
        perror("Serial thread creation failed");
        return 1;
    }

    pthread_join(tid, NULL);
    pthread_join(wid, NULL);

    pthread_mutex_destroy(&lock);
    pthread_mutex_destroy(&buffer_mutex);

    return 0;
}
