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

    int sckfd, n;
    char buffer[BUFFER_SIZE];

    // Initial connect
    while ((sckfd = NtripSocketInit(hst, get_request)) < 0) {
        perror("NTRIP init failed, retrying...");
        sleep(2);
    }

    while (1)
    {
        n = read(sckfd, buffer, BUFFER_SIZE);

        if (n > 0)
        {
            ssize_t total = 0;
            while (total < n) {
                ssize_t w = write(serial_fd, buffer + total, n - total);
                if (w < 0) {
                    perror("Serial write error");
                    break;
                }
                total += w;
            }
        }
        else
        {
            perror("Socket error, reconnecting...");
            close(sckfd);

            while ((sckfd = NtripSocketInit(hst, get_request)) < 0) {
                perror("Reconnect failed, retrying...");
                sleep(2);
            }
        }
    }

    close(sckfd);
    return NULL;
}


// ================= SERIAL READER THREAD =================
void *serial_reader_thread(void *arg)
{
    int serial_fd = *(int *)arg;

    int res;
    char buf[BUFSIZE];

    static char line[256];   
    static int idx = 0;

    // GNSS parsed fields
    int tim = 0, dat = 0, qua = 0, nsati = 0, diff_age = 0, diff_sat = 0;
    double lat = 0.0, lng = 0.0, spd = 0.0, hd = 0.0;
    float alti = 0.0, hdop = 0.0;
    char valid = 0, ltdir = 0, lngdir = 0, fixt = 0;

    int got_rmc = 0;
    int got_gga = 0;

    while (1)
    {
        res = read(serial_fd, buf, sizeof(buf));

        if (res > 0)
        {
            for (int i = 0; i < res; i++)
            {
                char c = buf[i];

                if (c == '\n')
                {
                    line[idx] = '\0';

                    if (idx > 6 && line[0] == '$' && verifyChecksum(line))
                    {
                        // RMC
                        if (line[3]=='R' && line[4]=='M' && line[5]=='C')
                        {
                            rmc_nmeaparser(line, &tim, &valid, &lat, &ltdir,
                                           &lng, &lngdir, &spd, &hd, &dat, &fixt);
                            got_rmc = 1;
                        }
                        // GGA
                        else if (line[3]=='G' && line[4]=='G' && line[5]=='A')
                        {
                            gga_nmeaparser(line, &tim, &lat, &ltdir, &lng, &lngdir,
                                           &qua, &nsati, &hdop, &alti, &diff_age, &diff_sat);
                            got_gga = 1;
                        }

                        if (got_rmc && got_gga)
                        {
                            printf("%d %c %f %c %f %c %f %f %d %c %d %d\n",
                                   tim, valid, lat, ltdir, lng, lngdir,
                                   spd, hd, dat, fixt, qua, diff_age);

                            got_rmc = 0;
                            got_gga = 0;
                        }
                    }

                    idx = 0;
                }
                else if (c != '\r')
                {
                    if (idx < sizeof(line) - 1)
                        line[idx++] = c;
                    else
                        idx = 0; // overflow protection
                }
            }
        }
        else if (res < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                usleep(1000);
                continue;
            }

            perror("Serial read error");
            usleep(10000);
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
