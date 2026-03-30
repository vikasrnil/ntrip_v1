#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <sys/socket.h>
#include <pthread.h>
#include "gnss.h"

#define GGA_BUF_SIZE 128
#define BUFFER_SIZE 1024

extern struct hostent *server;

/* ================= SHARED GGA BUFFER ================= */
char latest_gga[GGA_BUF_SIZE];
int gga_available = 0;

pthread_mutex_t gga_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t gga_cond = PTHREAD_COND_INITIALIZER;

/* ================= NTRIP CONFIG ================= */
char *hst = "";
char *get_request = "\r\n\r\n";

/* =========================================================
   NTRIP THREAD (Consumes GGA + Sends RTCM to GNSS)
   ========================================================= */
void *ntrip_thread(void *arg)
{
    int sockfd, serial_fd;
    char buffer[BUFFER_SIZE];
    int n;

    serial_fd = init_ntrip_write_port(DEVICE);
    sockfd = NtripSocketInit(hst, get_request);

    while (1)
    {
        /* ===== WAIT FOR NEW GGA ===== */
        pthread_mutex_lock(&gga_mutex);
        while (!gga_available)
        {
            pthread_cond_wait(&gga_cond, &gga_mutex);
        }

        /* Copy GGA locally */
        char gga_local[GGA_BUF_SIZE];
        strcpy(gga_local, latest_gga);
        gga_available = 0;

        pthread_mutex_unlock(&gga_mutex);

        /* ===== SEND GGA TO NTRIP SERVER ===== */
        write(sockfd, gga_local, strlen(gga_local));
        // printf("Sent GGA: %s\n", gga_local);

        /* ===== READ RTCM FROM SERVER ===== */
        n = read(sockfd, buffer, sizeof(buffer));
        if (n > 0)
        {
            write(serial_fd, buffer, n);
            // printf("RTCM bytes: %d\n", n);
        }
        else
        {
            printf("NTRIP reconnecting...\n");
            close(sockfd);
            sleep(1);
            sockfd = NtripSocketInit(hst, get_request);
        }
    }

    close(sockfd);
    close(serial_fd);
    return NULL;
}

/* =========================================================
   SERIAL THREAD (Produces GGA)
   ========================================================= */
void *serial_thread(void *arg)
{
    int fd = init_read_port(DEVICE);
    char buf[BUFSIZE];
    int res;

    while ((res = read(fd, buf, sizeof(buf) - 1)) > 0)
    {
        buf[res] = '\0';

        if (!verifyChecksum(buf))
            continue;

        if (strstr(buf, "$GNGGA") || strstr(buf, "$GPGGA"))
        {
            pthread_mutex_lock(&gga_mutex);

            strncpy(latest_gga, buf, GGA_BUF_SIZE - 1);
            latest_gga[GGA_BUF_SIZE - 1] = '\0';

            gga_available = 1;

            pthread_cond_signal(&gga_cond);
            pthread_mutex_unlock(&gga_mutex);

            // printf("Updated GGA: %s\n", latest_gga);
        }
    }

    close_port(fd);
    return NULL;
}

/* =========================================================
   MAIN
   ========================================================= */
int main()
{
    pthread_t tid_ntrip, tid_serial;

    if (pthread_create(&tid_ntrip, NULL, ntrip_thread, NULL) != 0)
    {
        perror("NTRIP thread failed");
        return 1;
    }

    if (pthread_create(&tid_serial, NULL, serial_thread, NULL) != 0)
    {
        perror("Serial thread failed");
        return 1;
    }

    pthread_join(tid_ntrip, NULL);
    pthread_join(tid_serial, NULL);

    pthread_mutex_destroy(&gga_mutex);
    pthread_cond_destroy(&gga_cond);

    return 0;
}
