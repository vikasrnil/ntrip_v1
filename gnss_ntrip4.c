#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <sys/socket.h>
#include <pthread.h>
#include "gnss.h"

extern struct hostent *server;

// NTRIP config
char *hst = "eklntrip.escortskubota.com";

char *get_request =
"GET /test2 HTTP/1.1\r\n"
"Host: eklntrip.escortskubota.com\r\n"
"Ntrip-Version: Ntrip/2.0\r\n"
"User-Agent: NTRIPClient\r\n"
"Authorization: Basic YWRtaW46UEFOQGJ4OTky\r\n"
"Connection: keep-alive\r\n\r\n";


// ================= NTRIP THREAD =================
void *ntrip_dev(void *arg)
{
    int serial_fd = *(int *)arg;

    int sockfd;
    char buffer[BUFFER_SIZE];
    int reconnect_count = 0;

connect:
    sockfd = NtripSocketInit(hst, get_request);

    if (sockfd < 0)
    {
        reconnect_count++;
        printf("NTRIP connect failed (%d)\n", reconnect_count);
        sleep(2);
        goto connect;
    }

    printf("NTRIP connected\n");

    while (1)
    {
        int n = read(sockfd, buffer, BUFFER_SIZE);

        if (n > 0)
        {
            write(serial_fd, buffer, n);  
        }
        else
        {
            reconnect_count++;
            printf("NTRIP disconnected, reconnecting (%d)\n", reconnect_count);

            close(sockfd);
            sleep(2);
            goto connect;
        }
    }

    return NULL;
}


// ================= SERIAL READER THREAD =================
void *serial_reader_thread(void *arg)
{
    int fd = *(int *)arg;

    char line[256];
    int idx = 0;
    char c;

    // Parsed data
    int tim=0, dat=0, qua=0, nsati=0, diff_age=0, diff_sat=0;
    double lat=0, lng=0, spd=0, hd=0;
    float alti=0, hdop=0;
    char valid=0, ltdir=0, lngdir=0, fixt=0;

    while (1)
    {
        if (read(fd, &c, 1) <= 0)
            continue;

        if (c == '\n')
        {
            line[idx] = '\0';
            idx = 0;

            if (line[0] != '$' || !verifyChecksum(line))
                continue;

            // GGA sentence
            else if (strstr(line, "GGA"))
            {
                gga_nmeaparser(line, &tim, &lat, &ltdir, &lng, &lngdir,
                               &qua, &nsati, &hdop, &alti, &diff_age, &diff_sat);

                printf("GGA: %d %f %c %f %c %d %d %f %f\n",
                       tim, lat, ltdir, lng, lngdir,
                       qua, nsati, hdop, alti);
            }
        }
        else if (c != '\r' && idx < sizeof(line) - 1)
        {
            line[idx++] = c;
        }
    }

    return NULL;
}


// ================= MAIN =================
int main()
{
    pthread_t tid, wid;

    int serial_fd = init_read_port(DEVICE);
    if (serial_fd < 0) {
        perror("Failed to open serial port");
        return 1;
    }

    // Pass address directly (NO struct)
    if (pthread_create(&tid, NULL, ntrip_dev, &serial_fd) != 0) {
        perror("Error creating NTRIP thread");
        close(serial_fd);
        return 1;
    }

    if (pthread_create(&wid, NULL, serial_reader_thread, &serial_fd) != 0) {
        perror("Error creating serial thread");
        pthread_cancel(tid);
        close(serial_fd);
        return 1;
    }

    pthread_join(tid, NULL);
    pthread_join(wid, NULL);

    close(serial_fd);

    return 0;
}
