#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <sys/socket.h>
#include <pthread.h>
#include "gnss.h"
#include <cjson/cJSON.h>

// ---------------- GLOBAL JSON ----------------
pthread_mutex_t json_mutex = PTHREAD_MUTEX_INITIALIZER;
cJSON *shared_json = NULL;

// ---------------- NTRIP CONFIG ----------------
char *hst = "qrtksa1.quectel.com"; 

char *get_request ="GET /AUTO_ITRF2020 HTTP/1.1\r\nHost: qrtksa1.quectel.com\r\nNtrip-Version: Ntrip/2.0\r\nUser-Agent: PythonNTRIP\r\nAuthorization: Basic ZXNjb3J0c2t1Ym90YV8wMDAwMDAxOm05amRlYjZ6\r\nConnection: keep-alive\r\n\r\n";


// ---------------- UPDATE JSON ----------------
void update_gga_json(const char *gga)
{
    pthread_mutex_lock(&json_mutex);

    if (shared_json == NULL) {
        shared_json = cJSON_CreateObject();
    }

    // Replace instead of delete
    cJSON *new_gga = cJSON_CreateString(gga);
    cJSON_ReplaceItemInObject(shared_json, "GGA", new_gga);

    pthread_mutex_unlock(&json_mutex);
}

// ---------------- SEND GGA TO NTRIP ----------------
void send_gga_to_ntrip(int sockfd)
{
    pthread_mutex_lock(&json_mutex);

    if (shared_json != NULL) {
        cJSON *gga_item = cJSON_GetObjectItemCaseSensitive(shared_json, "GGA");

        if (cJSON_IsString(gga_item) && gga_item->valuestring != NULL) {

            const char *gga = gga_item->valuestring;
            ssize_t total = 0;
            ssize_t len = strlen(gga);

            while (total < len) {
                ssize_t sent = write(sockfd, gga + total, len - total);
                if (sent <= 0) break;
                total += sent;
            }
        }
    }

    pthread_mutex_unlock(&json_mutex);
}

// ---------------- NTRIP THREAD ----------------
void *ntrip_dev(void *arg)
{
    int serial_fd = *(int *)arg;
    int sckfd, n;
    char buffer[BUFFER_SIZE];

    sckfd = NtripSocketInit(hst, get_request);
    if (sckfd < 0) {
        perror("NTRIP socket init failed");
        pthread_exit(NULL);
    }

    while (1)
    {
        //Send latest GGA from JSON
        send_gga_to_ntrip(sckfd);

        // Read RTCM from server
        n = read(sckfd, buffer, BUFFER_SIZE);

        if (n > 0)
        {
            ssize_t total = 0;
            while (total < n) {
                ssize_t written = write(serial_fd, buffer + total, n - total);
                if (written <= 0) {
                    perror("Serial write error");
                    break;
                }
                total += written;
            }
        }
        else
        {
            perror("Socket error, reconnecting...");
            close(sckfd);
            sleep(1);
            sckfd = NtripSocketInit(hst, get_request);
        }

        usleep(20000); // prevent CPU hog
    }
}


// ================= SERIAL READER THREAD =================
void *serial_reader_thread(void *arg)
{
    int serial_fd = *(int *)arg;

    int res;
    char buf[BUFSIZE];

    // Line buffer
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

                // End of NMEA line
                if (c == '\n')
                {
                    line[idx] = '\0';

                    if (idx > 6 && line[0] == '$' && verifyChecksum(line))
                    {
                        // ---------------- RMC ----------------
                        if ((line[3] == 'R' && line[4] == 'M' && line[5] == 'C'))
                        {
                            rmc_nmeaparser(line, &tim, &valid, &lat, &ltdir,
                                           &lng, &lngdir, &spd, &hd, &dat, &fixt);
                            got_rmc = 1;
                        }

                        // ---------------- GGA ----------------
                        else if ((line[3] == 'G' && line[4] == 'G' && line[5] == 'A'))
                        {
                            gga_nmeaparser(line, &tim, &lat, &ltdir, &lng, &lngdir,
                                           &qua, &nsati, &hdop, &alti, &diff_age, &diff_sat);

                            got_gga = 1;

                            // Update JSON here
                            update_gga_json(line);
                        }

                        //Print only when both received
                        if (got_rmc && got_gga)
                        {
                            printf("%d %c %f %c %f %c %f %f %d %c %d %d\n",
                                   tim, valid, lat, ltdir, lng, lngdir,
                                   spd, hd, dat, fixt, qua, diff_age);

                            got_rmc = 0;
                            got_gga = 0;
                        }
                    }

                    idx = 0; // reset line buffer
                }
                else if (c != '\r')
                {
                    if (idx < sizeof(line) - 1)
                    {
                        line[idx++] = c;
                    }
                    else
                    {
                        // Overflow protection
                        idx = 0;
                    }
                }
            }
        }
        else if (res == 0)
        {
            continue;
        }
        else
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

    pthread_exit(NULL);
}

// ---------------- MAIN ----------------
int main()
{
    pthread_t tid, wid;

    // Open serial port ONCE
    int serial_fd = init_read_port(DEVICE);
    if (serial_fd < 0) {
        perror("Serial open failed");
        return 1;
    }

    // Create threads
    if (pthread_create(&tid, NULL, ntrip_dev, &serial_fd) != 0) {
        perror("NTRIP thread failed");
        return 1;
    }

    if (pthread_create(&wid, NULL, serial_reader_thread, &serial_fd) != 0) {
        perror("Serial thread failed");
        return 1;
    }

    pthread_join(tid, NULL);
    pthread_join(wid, NULL);

    close(serial_fd);

    if (shared_json) {
        cJSON_Delete(shared_json);
    }

    pthread_mutex_destroy(&json_mutex);

    return 0;
}
