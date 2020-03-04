#ifndef NTRIP_H
#define NTRIP_H

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
#include "../serialcomm/serialComm.h"

/* The string, which is send as agent in HTTP request */
// #define AGENTSTRING "NTRIP LinuxClient"
// #define REVISIONSTRING "1.51"
#define AGENTSTRING "NTRIP LefebureNTRIPClient"
#define REVISIONSTRING "20131124"
#define MAXDATASIZESEND 1000 /* max number of bytes we can get at once */
#define MAXDATASIZERCV 2024 /* max number of bytes we can get at once */


struct Args {
    const char *server;
    int port;
    const char *user;
    const char *password;
    const char *mount;
};

int i, sockfd, numbytes;
struct hostent *he;
struct sockaddr_in their_addr; /* connector's address information */

static char encodingTable [64] = {
'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P',
'Q','R','S','T','U','V','W','X','Y','Z','a','b','c','d','e','f',
'g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v',
'w','x','y','z','0','1','2','3','4','5','6','7','8','9','+','/'
};

struct Args args;
int encode(char *buf, int size, const char *user, const char *pwd);

int connectNtrip(const char* server, const char* mountpoint, int port, const char* user, const char* password);
int sendGGA(const char* data, int size);
void socketcallback();
void emptybuf(char* buf, int size);

#endif