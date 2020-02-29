#include "ntrip.h"

void initNtrip(char* server, char* mountpoint, int port, char* user, char* password){
    args.server = server;
    args.mount = mountpoint;
    args.port = port;
    args.user = user;
    args.password = password;
}

void initNtripDefault() {
    args.server = "flepos.vlaanderen.be";
    args.port = 2101;
    args.user = "852a009";
    args.password = "97115";
    // args.mount = "FLEPOSVRS32GREC";
    args.mount = 0;
}

/* does not buffer overrun, but breaks directly after an error */
/* returns the number of required bytes */
int encode(char *buf, int size, char *user, char *pwd) {
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
    return bytes;
}

int ntripConnect() {
    char buf[MAXDATASIZE];

    if(!(he=gethostbyname(args.server))) {
        perror("gethostbyname-blabla");
        return -1;
    }
    if((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        perror("socket");
        return -1;
    }
    their_addr.sin_family = AF_INET; /* host byte order */
    their_addr.sin_port = htons(args.port); /* short, network byte order */
    their_addr.sin_addr = *((struct in_addr *)he->h_addr);
    memset(&(their_addr.sin_zero), '\0', 8);
    if(connect(sockfd, (struct sockaddr *)&their_addr, sizeof(struct sockaddr)) == -1) {
        perror("connect");
        return -1;
    }
    if(!args.mount) {
        i = snprintf(buf, MAXDATASIZE,
        "GET / HTTP/1.1\r\n"
        "User-Agent: %s/%s\r\n"
        "Accept: */*\r\n"
        "Connection: close \r\n"
        "\r\n"
        , AGENTSTRING, REVISIONSTRING);
    } else {
        i=snprintf(buf, MAXDATASIZE-40, /* leave some space for login */
            "GET /%s HTTP/1.1\r\n"
            "User-Agent: %s/%s\r\n"
            // "Accept: */*\r\n"
            // "Connection: close \r\n"
            "Authorization: Basic "
            , args.mount, AGENTSTRING, REVISIONSTRING);
        if(i > MAXDATASIZE-40 && i < 0) /* second check for old glibc */ {
            fprintf(stderr, "Requested mount too long\n");
            return -1;
        }
        i += encode(buf+i, MAXDATASIZE-i-5, args.user, args.password);
        if(i > MAXDATASIZE-5) {
            fprintf(stderr, "Username and/or password too long\n");
            return -1;
        }
        snprintf(buf+i, 5, "\r\n\r\n");
        i += 5;
        // snprintf(buf+i, 75, "$GPGGA,122724.00,5104.07582,N,00337.45089,E,1,12,0.54,13.8,M,46.0,M,,*4D\r\n");
        // i += 75;
    }
    if(send(sockfd, buf, i, 0) != i) {
        perror("send");
        return -1;
    }
    if(args.mount) {
        int k = 0;
        while((numbytes=recv(sockfd, buf, MAXDATASIZE-1, 0)) != -1) {
            if(!k) {
                if(numbytes != 14 || strncmp("ICY 200 OK\r\n", buf, 12)) {
                    fprintf(stderr, "Could not get the requested mount\n");
                    return -1;
                }
                ++k;
                printf("%s\r\n",buf);
            }
            else {
                printf("numbytes %d\r\n", numbytes);
                fwrite(buf, numbytes, 1, stdout);
            }
        }
    }
    else {
        while((numbytes=recv(sockfd, buf, MAXDATASIZE-1, 0)) != -1) {
            fwrite(buf, numbytes, 1, stdout);
            if(!strncmp("ENDSOURCETABLE\r\n", buf+numbytes-16, 16))
            break;
        }
    }
    close(sockfd);
    
    return 0;
}
