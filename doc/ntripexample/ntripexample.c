/*
Easy example NTRIP client for Linux/Unix.
Copyright (C) 2003 by Dirk Stoecker <stoecker@epost.de>
This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
or read http://www.gnu.org/licenses/gpl.txt
*/
/* Version history
Please always keep revision history and the two related strings up to date!
1.1 2003-02-24 stoecker initial version
1.2 2003-02-25 stoecker fixed agent string
*/
#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <netdb.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
/* The string, which is send as agent in HTTP request */
#define AGENTSTRING "NTRIP LinuxClient"
#define MAXDATASIZE 1000 /* max number of bytes we can get at once */
char buf[MAXDATASIZE];
/* CVS revision and version */
static char revisionstr[] = "$Revision: 1.2 $";
static char datestr[] = "$Date: 2003/03/25 13:00:00 $";
struct Args {
    char *server;
    int port;
    char *user;
    char *password;
    char *mount;
};

static int getargs(int argc, char **argv, struct Args *args)
{

    args->server = "flepos.vlaanderen.be";
    args->port = 2101;
    args->user = "852a009";
    args->password = "97115";
    // args->mount = 0;
    args->mount = "FLEPOSVRS32GREC";

    return 1;
}

static char encodingTable [64] = {
'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P',
'Q','R','S','T','U','V','W','X','Y','Z','a','b','c','d','e','f',
'g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v',
'w','x','y','z','0','1','2','3','4','5','6','7','8','9','+','/'
};

/* does not buffer overrun, but breaks directly after an error */
/* returns the number of required bytes */
static int encode(char *buf, int size, char *user, char *pwd) {
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

int main(int argc, char **argv) {
    struct Args args;
    if(getargs(argc, argv, &args))
    {
        int i, sockfd, numbytes;
        char buf[MAXDATASIZE];
        struct hostent *he;
        struct sockaddr_in their_addr; /* connector's address information */
        if(!(he=gethostbyname(args.server))) {
            perror("gethostbyname-blabla");
            exit(1);
        }
        if((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
            perror("socket");
            exit(1);
        }
        their_addr.sin_family = AF_INET; /* host byte order */
        their_addr.sin_port = htons(args.port); /* short, network byte order */
        their_addr.sin_addr = *((struct in_addr *)he->h_addr);
        memset(&(their_addr.sin_zero), '\0', 8);
        if(connect(sockfd, (struct sockaddr *)&their_addr, sizeof(struct sockaddr)) == -1) {
            perror("connect");
            exit(1);
        }
        if(!args.mount) {
            i = snprintf(buf, MAXDATASIZE,
            "GET / HTTP/1.0\r\n"
            "User-Agent: %s/%s\r\n"
            "Accept: */*\r\n"
            "Connection: close \r\n"
            "\r\n"
            , AGENTSTRING, revisionstr);
        } else {
            i=snprintf(buf, MAXDATASIZE-40, /* leave some space for login */
                "GET /%s HTTP/1.0\r\n"
                "User-Agent: %s/%s\r\n"
                // "Accept: */*\r\n"
                // "Connection: close \r\n"
                "Authorization: Basic "
                , args.mount, AGENTSTRING, revisionstr);
            if(i > MAXDATASIZE-40 && i < 0) /* second check for old glibc */ {
                fprintf(stderr, "Requested mount too long\n");
                exit(1);
            }
            i += encode(buf+i, MAXDATASIZE-i-5, args.user, args.password);
            if(i > MAXDATASIZE-5) {
                fprintf(stderr, "Username and/or password too long\n");
                exit(1);
            }
            snprintf(buf+i, 5, "\r\n\r\n");
            i += 5;
            snprintf(buf+i, 75, "$GPGGA,122724.00,5104.07582,N,00337.45089,E,1,12,0.54,13.8,M,46.0,M,,*4D\r\n");
            i += 75;
        }
        if(send(sockfd, buf, i, 0) != i) {
            perror("send");
            exit(1);
        }
        if(args.mount) {
            int k = 0;
            while((numbytes=recv(sockfd, buf, MAXDATASIZE-1, 0)) != -1) {
                if(!k) {
                    if(numbytes != 14 || strncmp("ICY 200 OK\r\n", buf, 12)) {
                        fprintf(stderr, "Could not get the requested mount\n");
                        exit(1);
                    }
                    ++k;
                }
                else {
                    fwrite("alive\n", 6, 1, stdout);
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
    }
    return 0;
}
