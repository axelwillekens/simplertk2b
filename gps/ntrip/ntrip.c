#include "ntrip.h"

/* does not buffer overrun, but breaks directly after an error */
/* returns the number of required bytes */
int encode(char *buf, int size, const char *user, const char *pwd) {
    unsigned char inbuf[3];
    char *out = buf;
    int i, sep = 0, fill = 0, bytes = 0;
    while(*user || *pwd) {
        i = 0;
        while(i < 3 && *user) inbuf[i++] = *(user++);
        if(i < 3 && !sep) {inbuf[i++] = ':'; ++sep; }
        while(i < 3 && *pwd) inbuf[i++] = *(pwd++);
        while(i < 3) {inbuf[i++] = 0; ++fill; }
        if(out-buf < size-1) *(out++) = encodingTable[(inbuf [0] & 0xFC) >> 2];
        if(out-buf < size-1) *(out++) = encodingTable[((inbuf [0] & 0x03) << 4) | ((inbuf [1] & 0xF0) >> 4)];
        if(out-buf < size-1) {
            if(fill == 2) *(out++) = '=';
            else *(out++) = encodingTable[((inbuf [1] & 0x0F) << 2) | ((inbuf [2] & 0xC0) >> 6)];
        }
        if(out-buf < size-1) {
            if(fill >= 1) *(out++) = '=';
            else *(out++) = encodingTable[inbuf [2] & 0x3F];
        }
        bytes += 4;
    }
    if(out-buf < size) *out = 0;
    // printf("%s\n",buf);
    return bytes;
}

// Sets up connection to NTRIP server
// Returns sockfd if success
// Returns -1 if error
int connectNtrip(struct Args* args) {
    int i;
    int sockfd;
    struct hostent *he;
    struct sockaddr_in their_addr; /* connector's address information */

    char bufsend[MAXDATASIZESEND];
    emptybuf(bufsend, MAXDATASIZESEND);

    if(!(he=gethostbyname(args->server))) {
        perror("gethostbyname-blabla");
        return -1;
    }
    if((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        perror("socket");
        return -1;
    }
    their_addr.sin_family = AF_INET; /* host byte order */
    their_addr.sin_port = htons(args->port); /* short, network byte order */
    their_addr.sin_addr = *((struct in_addr *)he->h_addr);
    memset(&(their_addr.sin_zero), '\0', 8);
    if(connect(sockfd, (struct sockaddr *)&their_addr, sizeof(struct sockaddr)) == -1) {
        perror("connect");
        return -1;
    }
    if(!args->mount) {
        i = snprintf(bufsend, MAXDATASIZESEND,
        "GET / HTTP/1.1\r\n"
        "User-Agent: %s/%s\r\n"
        "Accept: */*\r\n"
        "Connection: close \r\n"
        "\r\n"
        , AGENTSTRING, REVISIONSTRING);
    } else {
        i=snprintf(bufsend, MAXDATASIZESEND-40, /* leave some space for login */
            "GET /%s HTTP/1.1\r\n"
            // "Ntrip-Version: Ntrip/2.0\r\n"
            "User-Agent: %s/%s\r\n"
            // "Accept: */*\r\n"
            // "Connection: close \r\n"
            "Authorization: Basic "
            , args->mount, AGENTSTRING, REVISIONSTRING);
        if(i > MAXDATASIZESEND-40 && i < 0) /* second check for old glibc */ {
            fprintf(stderr, "Requested mount too long\n");
            return -1;
        }
        i += encode(bufsend+i, MAXDATASIZESEND-i-5, args->user, args->password);
        if(i > MAXDATASIZESEND-5) {
            fprintf(stderr, "Username and/or password too long\n");
            return -1;
        }
        snprintf(bufsend+i, 5, "\r\n\r\n");
        i += 5;
    }
    if(send(sockfd, bufsend, i, 0) != i) {
        perror("send");
        return -1;
    }
    
    return sockfd;
}

int serial_port = 0;

void updateSerial_port(int port) {
    serial_port = port;
}

void socketcallback(int sockfd, struct Args* args) {
    int numbytes;
    char bufrecv[MAXDATASIZERCV];
    emptybuf(bufrecv, MAXDATASIZERCV);

    if(args->mount) {
        int k = 0;
        while((numbytes=recv(sockfd, bufrecv, MAXDATASIZERCV-1, 0)) != -1) {
            if (serial_port != -1) {
                WriteSerialPort(serial_port, bufrecv, numbytes);
            }
            emptybuf(bufrecv, MAXDATASIZERCV);
        }
        printf("End of fun!\n");
    }
    else {
        // print SOURCETABLE
        while((numbytes=recv(sockfd, bufrecv, MAXDATASIZERCV-1, 0)) != -1) {
            fwrite(bufrecv, numbytes, 1, stdout);
            if(!strncmp("ENDSOURCETABLE\r\n", bufrecv+numbytes-16, 16)) { break; }
        }
    }
    close(sockfd);
}

int sendGGA(int sockfd, const char* data, int size) {
    if (sockfd == -1) {
        perror("no connection");
        return -1;
    } else {
        if(send(sockfd, data, size, 0) != size) {
            perror("send");
            return -1;
        }
    }

    return 0;
}

void emptybuf(char* buf, int size) {
    for (int i=0; i < size; i++) {
        buf[i] = '\0';
    }
}