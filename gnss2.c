#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include "gnss.h"

struct termios newtio,newt;
struct timeval timeout;
struct sockaddr_in server_addr;
struct hostent *server;

FILE *gnss_log()
{
    time_t current_time;
    struct tm *local_time_info;
    // Get the current calendar time as a time_t object
    // Passing NULL to time() also stores the value in current_time
    time(&current_time); 
    // Convert to local time format (struct tm)
    local_time_info = localtime(&current_time);
    // Format and print the time
    // strftime is a more flexible and safer alternative to asctime/ctime
    char time_string[80];
    strftime(time_string, sizeof(time_string), "%Y-%m-%d %H:%M:%S", local_time_info);
    //printf("Current local time: %s\n", time_string);
    char filename[100];
    sprintf(filename,"log_%s.json", time_string);
    FILE *file = fopen(filename, "w");
    if (file == NULL) {
        fprintf(stderr, "Could not open %s for writing\n", filename);
        fclose(file);
    }
    return file;

}


int init_read_port(const char* dev){
    int rd;
    rd = open(dev,O_RDWR | O_NOCTTY | O_SYNC);
    if(rd < 0) { perror(DEVICE); exit(-1);}
    int status = tcgetattr(rd, &newtio); // get the serial port settings from the termios structure
    
    /*****************     Configure the Baudrate       *******************/
    cfsetispeed(&newtio,BAUDRATE); //Set input Baudrate 
    cfsetospeed(&newtio,BAUDRATE); //Set output Baudrate
    /*****************     Configure the termios structure   ************************/
    newtio.c_lflag |= ICANON;  // Enable CANONICAL Mode for Serial Port Comm
    newtio.c_lflag &= ~(ECHO | ECHOE | ISIG); 
    newtio.c_iflag &= ~(IXON | IXOFF | IXANY);         // Turn OFF software based flow control (XON/XOFF).
    newtio.c_cflag &= ~CRTSCTS;                        // Turn OFF Hardware based flow control RTS/CTS 
    
    newtio.c_cflag |=  CREAD | CLOCAL;         // Turn ON  the receiver of the serial port (CREAD)
    
    // Set 8N1 (8 bits, no parity, 1 stop bit)
    newtio.c_cflag &= ~PARENB;      // No parity
    newtio.c_cflag &= ~CSTOPB;      // One stop bit
    newtio.c_cflag &= ~CSIZE;       
    newtio.c_cflag |=  CS8;          // 8 bits
    newtio.c_oflag &= ~OPOST;//No Output Processing/
    
    newtio.c_cc[VMIN]  = 1; /* Read at least 1 characters */  
    newtio.c_cc[VTIME] = 0; /* Wait for 10 *100ms = 1 second ,measured in increments of 100ms */
    status = tcsetattr(rd,TCSANOW,&newtio);  // update new settings to termios structure,
                                                            // TCSANOW tells to make the changes now without waiting
    /* Flush both input and output buffers to clear out garbage values */
    if (tcflush(rd, TCIOFLUSH) != 0)
    {
        perror("tcflush");
    }
    return rd;
}

uint8_t verifyChecksum(const char *sentence){
    uint8_t checksum = 0;
    // Skip '$'
    if (*sentence == '$') sentence++;
    // XOR until '*'
    while (*sentence != '*' && *sentence != '\0') {
        checksum ^= *sentence++;
    }
    return checksum;
}


void rmc_nmeaparser(char *bu,int *timk,char *va,double *latv,char *l_dir,double *lngv,char *ln_dir,double *sp, double *hea,int *da,char *fa){
    if(strstr(bu,"$GNRMC")|| strstr(bu,"$GPRMC")){
        char *tok = strtok(bu,",");
        int z=0;
        while(tok!=NULL){
            if(z==1) *timk = atoi(tok);
            if(z==2) *va = tok[0];
            if(z==3) *latv = lat_filt(atof(tok));
            if(z==4) *l_dir = tok[0];
            if(z==5) *lngv = lng_filt(atof(tok));
            if(z==6) *ln_dir = tok[0];
            if(z==7) *sp = atof(tok);
            if(z==8) *hea = atof(tok);
            if(z==9) *da = atoi(tok);
            if(z==10) *fa = tok[0];
            tok=strtok(NULL,",");
            z+=1;
        }
    }
}

void gga_nmeaparser(char *ga,int *timg,double *latg,char *l_dirg,double *lngvg,char *ln_dirg,int *qa, int *nsat,float *hdp,float *alt,int *difa,int *difs){
    if(strstr(ga,"$GNGGA")|| strstr(ga,"$GPGGA")){
        char *tok = strtok(ga,",");
        int z=0;
        while(tok!=NULL){
            if(z==1) *timg = atoi(tok);
            if(z==2) *latg = lat_filt(atof(tok));
            if(z==3) *l_dirg = tok[0];
            if(z==4) *lngvg = lng_filt(atof(tok));
            if(z==5) *ln_dirg = tok[0];
            if(z==6) *qa = atoi(tok);
            if(z==7) *nsat = atoi(tok);
            if(z==8) *hdp = atof(tok);
            if(z==9) *alt = atof(tok);
            if(z==13) *difa = atoi(tok);
            if(z==14) *difs = atoi(tok);
            tok=strtok(NULL,",");
            z+=1;
        }
    }
}


void convert_time_to_UTC(unsigned int UTC_Time,int *hrk ,int *mink,int *seck) 
{ 
    unsigned int hour, min, sec;
    hour = (UTC_Time / 10000);
    min = (UTC_Time % 10000) / 100;
    sec = (UTC_Time % 10000) % 100;
    hour = hour+5;
    if (hour >= 23)
    {
        hour = 23-hour;     
    }
    min = min + 30;
    if (min > 59)
    {
        min = min-60;
        hour+=1;
    }
    *hrk = (int)hour;*mink = (int)min;*seck = (int)sec;
}


void log_json_to_file(FILE *file_ptr,char *timj,double latj, double lngj,double altj, double hdopj, double spdj, double headj,int gga_qaj,int gga_numsvj, int gga_diff_agej,int datj,int d_sat) 
{
    cJSON *monitor = cJSON_CreateObject();
    cJSON_AddStringToObject(monitor,"Time", timj);
    cJSON_AddNumberToObject(monitor,"Latitude", latj);
    cJSON_AddNumberToObject(monitor,"Longitude", lngj);
    cJSON_AddNumberToObject(monitor,"Altitude", altj);
    cJSON_AddNumberToObject(monitor,"Head", headj);
    cJSON_AddNumberToObject(monitor,"Speed", spdj);
    cJSON_AddNumberToObject(monitor,"HDOP", hdopj);
    cJSON_AddNumberToObject(monitor,"Quality", gga_qaj);
    cJSON_AddNumberToObject(monitor,"Number of Satellite", gga_numsvj);
    cJSON_AddNumberToObject(monitor,"Differential Age", gga_diff_agej);
    cJSON_AddNumberToObject(monitor,"Date", datj);
    cJSON_AddNumberToObject(monitor,"NTRIP Station ID", d_sat);
    char *json_string = cJSON_Print(monitor);
    if (file_ptr != NULL) {
        fputs(json_string, file_ptr);
        //fclose(file_ptr);
    } else {
        perror("Error opening file");
    }
    fprintf(file_ptr, "%s", json_string);
    cJSON_Delete(monitor);
    free(json_string);
}


void close_port(int prt)
{
    tcsetattr(prt,TCSANOW,&newtio);
    close(prt);
}


void write_json_to_file(char const * fil,char *timj,double latj, double lngj,double altj, double hdopj, double spdj, double headj,int gga_qaj,int gga_numsvj, int gga_diff_agej,int datj,int d_sat)
{
    FILE *file = fopen(fil, "w");
    if (file == NULL) {
        fprintf(stderr, "Could not open %s for writing\n", fil);
        fclose(file);
    }

    cJSON *monitor = cJSON_CreateObject();
    cJSON_AddStringToObject(monitor,"Time", timj);
    cJSON_AddNumberToObject(monitor,"Latitude", latj);
    cJSON_AddNumberToObject(monitor,"Longitude", lngj);
    cJSON_AddNumberToObject(monitor,"Altitude", altj);
    cJSON_AddNumberToObject(monitor,"Head", headj);
    cJSON_AddNumberToObject(monitor,"Speed", spdj);
    cJSON_AddNumberToObject(monitor,"HDOP", hdopj);
    cJSON_AddNumberToObject(monitor,"Quality", gga_qaj);
    cJSON_AddNumberToObject(monitor,"Number of Satellite", gga_numsvj);
    cJSON_AddNumberToObject(monitor,"Differential Age", gga_diff_agej);
    cJSON_AddNumberToObject(monitor,"Date", datj);
    cJSON_AddNumberToObject(monitor,"NTRIP Station ID", d_sat);
    char *json_string = cJSON_Print(monitor);

    if (file != NULL) {
        fputs(json_string, file);
        fclose(file);
    } else {
        perror("Error opening file");
    }
    fprintf(file, "%s", json_string);
    
    cJSON_Delete(monitor);
    free(json_string);
    
}


int init_ntrip_write_port(const char* sdev)
{
    int td;
    td = open(sdev,O_RDWR | O_NOCTTY | O_SYNC);
    if(td < 0) { perror(DEVICE); exit(-1);}
    int status = tcgetattr(td, &newt); // get the serial port settings from the termios structure
    
    /*****************     Configure the Baudrate       *******************/
    cfsetispeed(&newt,BAUDRATE); //Set input Baudrate 
    cfsetospeed(&newt,BAUDRATE); //Set output Baudrate
    /*****************     Configure the termios structure   ************************/
    // new.c_lflag |= ICANON;  // Enable CANONICAL Mode for Serial Port Comm
    newt.c_lflag &= ~(ECHO | ECHOE | ISIG); 
    newt.c_iflag &= ~(IXON | IXOFF | IXANY | ICANON); // Turn OFF software based flow control (XON/XOFF).
    newt.c_cflag &= ~CRTSCTS;                        // Turn OFF Hardware based flow control RTS/CTS 
    
    newt.c_cflag |=  CREAD | CLOCAL;         // Turn ON  the receiver of the serial port (CREAD)
    
    // Set 8N1 (8 bits, no parity, 1 stop bit)
    newt.c_cflag &= ~PARENB;      // No parity
    newt.c_cflag &= ~CSTOPB;      // One stop bit
    newt.c_cflag &= ~CSIZE;       
    newt.c_cflag |=  CS8;          // 8 bits
    newt.c_oflag &= ~OPOST;//No Output Processing/
    
    newt.c_cc[VMIN]  = 1; /* Read at least 1 characters */  
    newt.c_cc[VTIME] = 0; /* Wait for 10 *100ms = 1 second ,measured in increments of 100ms */
    status = tcsetattr(td,TCSANOW,&newt);  // update new settings to termios structure,
                                                            // TCSANOW tells to make the changes now without waiting
    /* Flush both input and output buffers to clear out garbage values */
    if (tcflush(td, TCIOFLUSH) != 0)
    {
        perror("tcflush");
    }
    return td;
}

int NtripSocketInit(char *host,char *httpreq)
{
    int optval = 1,sockfd; 
    timeout.tv_sec = 60;
    timeout.tv_usec = 0;

        // 1. Create a socket
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
         // Set the receive timeout option on the socket
    
    if (setsockopt (sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout,
                sizeof timeout) < 0){
        perror("setsockopt failed\n");}

    if (setsockopt (sockfd, SOL_SOCKET, SO_SNDTIMEO, &timeout,
                sizeof timeout) < 0){
        perror("setsockopt failed\n");
    }
        
    if (setsockopt(sockfd, SOL_SOCKET, SO_KEEPALIVE, &optval, sizeof(optval)) < 0) {
    perror("setsockopt(SO_KEEPALIVE)");
    } 

    if (sockfd < 0) 
        perror("ERROR opening socket");

    // 2. Get server details
    server = gethostbyname(host);
    if (server == NULL) {
        fprintf(stderr,"ERROR, no such host\n");
        close(sockfd);
        sleep(10); 
        sockfd = NtripSocketInit(host,httpreq);
    }
    
    bzero((char *) &server_addr, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, 
         (char *)&server_addr.sin_addr.s_addr,
         server->h_length);
    server_addr.sin_port = htons(PORT);

    // 3. Connect to the server
    if (connect(sockfd, (struct sockaddr *) &server_addr, sizeof(server_addr)) < 0) 
        perror("ERROR connecting");

    // 4. Send the HTTP request
    int writebytes = write(sockfd, httpreq, strlen(httpreq));
    if (writebytes < 0) 
        perror("ERROR writing to socket");
    
    printf("Sent HTTP Request:\n%s\n", httpreq);
    printf("--- Server Response ---\n");

    return sockfd;
  }


double lng_filt(float kef){
    float longitude = 0.0;
    float k_lng_deg=(kef*0.01);
    unsigned int deglng = (int)k_lng_deg;
    if(deglng > 68 && 97 > deglng){
        float seclng = (kef- (float)deglng*100)/60;
        longitude = (float)deglng + seclng;
    }
    return longitude;
}

double lat_filt(float def) {
    float latitude = 0.0;
    float k_lat_deg=(def*0.01);
    unsigned int deg = (int)k_lat_deg;
    if(deg > 8 && 37 > deg){
        float sec = (def- (float)deg*100)/60;
        latitude = (float)deg + sec;
    }
    return latitude;
}


void gll_nmeaparser(char *bu,int *timk,char *va,double *latv,char *l_dir,double *lngv,char *ln_dir,char *fa){
    if(strstr(bu,"$GNRMC")|| strstr(bu,"$GPRMC")){
        char *tok = strtok(bu,",");
        int z=0;
        while(tok!=NULL){
            if(z==5) *timk = atoi(tok);
            if(z==6) *va = tok[0];
            if(z==1) *latv = lat_filt(atof(tok));
            if(z==2) *l_dir = tok[0];
            if(z==3) *lngv = lng_filt(atof(tok));
            if(z==4) *ln_dir = tok[0];
            if(z==7) *fa = tok[0];
            tok=strtok(NULL,",");
            z+=1;
        }
    }
}

