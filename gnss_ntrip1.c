#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <stdlib.h>
#include "gnss.h"

// ---------------- CONFIG ----------------
static const char *host = "eklntrip.escortskubota.com";

static const char *request =
"GET /test2 HTTP/1.1\r\nHost: eklntrip.escortskubota.com\r\nNtrip-Version: Ntrip/2.0\r\nUser-Agent: NTRIPClient\r\nAuthorization: Basic YWRtaW46UEFOQGJ4OTky\r\nConnection: keep-alive\r\n\r\n";


// ---------------- SAFE WRITE ----------------
static int write_full(int fd, const char *buf, int len)
{
    int total = 0;

    while (total < len)
    {
        int n = write(fd, buf + total, len - total);
        if (n <= 0)
            return -1;
        total += n;
    }
    return total;
}

// ---------------- NTRIP THREAD ----------------
void *ntrip_thread(void *arg)
{
    int serial_fd = *(int *)arg;
    int sockfd;
    char buffer[GNSS_BUFFER_SIZE];

    while (1)
    {
        sockfd = NtripSocketInit((char *)host, (char *)request);
        if (sockfd < 0) {
            perror("Socket init failed");
            sleep(2);
            continue;
        }
        
        printf("NTRIP connected\n");

        while (1)
        {
            int n = read(sockfd, buffer, sizeof(buffer));

            if (n > 0)
            {
                if (write_full(serial_fd, buffer, n) < 0) {
                    perror("Serial write failed");
                    break;
                }
            }
            else
            {
                break;
            }
        }

        close(sockfd);
        sleep(1);
    }

    pthread_exit(NULL);
}

// ---------------- SERIAL THREAD ----------------
void *serial_reader_thread(void *arg)
{
    int fd = *(int *)arg;

    char buf[GNSS_BUFSIZE];
    char line[GNSS_BUFSIZE];
    int idx = 0;

    // Parsed data
    int tim = 0, dat = 0, qua = 0, nsati = 0, diff_age = 0, diff_sat = 0;
    double lat = 0, lng = 0, spd = 0, hd = 0;
    float alti = 0, hdop = 0;
    char valid = 0, ltdir = 0, lngdir = 0, fixt = 0;

    // Sync flags
    int rmc_updated = 0;
    int gga_updated = 0;

    while (1)
    {
        int res = read(fd, buf, sizeof(buf));

        if (res <= 0)
            continue;

        for (int i = 0; i < res; i++)
        {
            if (buf[i] == '\n')
            {
                line[idx] = '\0';

                // REMOVE '\r'
                line[strcspn(line, "\r")] = '\0';

                // -------- BASIC VALIDATION --------
                if (line[0] != '$' || strchr(line, '*') == NULL)
                {
                    idx = 0;
                    continue;
                }

                // -------- CHECKSUM --------
                if (!verifyChecksum(line))
                {
                    idx = 0;
                    continue;
                }

                // -------- RMC --------
                if (line[3] == 'R' && line[4] == 'M' && line[5] == 'C')
                {
                    char temp[GNSS_BUFSIZE];
                    strncpy(temp, line, sizeof(temp));
                    temp[sizeof(temp) - 1] = '\0';

                    rmc_nmeaparser(temp, &tim, &valid,
                                   &lat, &ltdir,
                                   &lng, &lngdir,
                                   &spd, &hd,
                                   &dat, &fixt);

                    rmc_updated = 1;
                }

                // -------- GGA --------
                if (line[3] == 'G' && line[4] == 'G' && line[5] == 'A')
                {
                    char temp[GNSS_BUFSIZE];
                    strncpy(temp, line, sizeof(temp));
                    temp[sizeof(temp) - 1] = '\0';

                    gga_nmeaparser(temp, &tim,
                                   &lat, &ltdir,
                                   &lng, &lngdir,
                                   &qua, &nsati,
                                   &hdop, &alti,
                                   &diff_age, &diff_sat);

                    gga_updated = 1;
                }

                // PRINT ONLY WHEN BOTH UPDATED
                if (rmc_updated && gga_updated && valid == 'A')
                {
                    printf("%d %c %.6f %c %.6f %c %.2f %.2f %d %d %.2f %.2f %d %d\n",
                           tim, valid,
                           lat, ltdir,
                           lng, lngdir,
                           spd, hd,
                           nsati, diff_age,
                           alti, hdop,
                           qua, diff_sat);

                    rmc_updated = 0;
                    gga_updated = 0;
                }

                idx = 0;
            }
            else
            {
                if (idx < GNSS_BUFSIZE - 1)
                    line[idx++] = buf[i];
            }
        }
    }

    pthread_exit(NULL);
}

// ---------------- MAIN ----------------
int main()
{
    pthread_t t1, t2;

    int gnss_fd = init_read_port(GNSS_DEVICE);
    if (gnss_fd < 0) {
        perror("Serial open failed");
        return 1;
    }

    if (pthread_create(&t1, NULL, ntrip_thread, &gnss_fd) != 0)
        return 1;

    if (pthread_create(&t2, NULL, serial_reader_thread, &gnss_fd) != 0)
        return 1;

    pthread_join(t1, NULL);
    pthread_join(t2, NULL);

    close_port(gnss_fd);

    return 0;
}
