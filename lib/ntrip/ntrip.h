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
#define AGENTSTRING "NTRIP LinuxClient"
#define MAXDATASIZE 1000 /* max number of bytes we can get at once */

struct Args {
    char *server;
    int port;
    char *user;
    char *password;
    char *mount;
};

int i, sockfd, numbytes;
char buf[MAXDATASIZE];
struct hostent *he;
struct sockaddr_in their_addr; /* connector's address information */

static char revisionstr[] = "$Revision: 1.2 $";
static char encodingTable [64] = {
'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P',
'Q','R','S','T','U','V','W','X','Y','Z','a','b','c','d','e','f',
'g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v',
'w','x','y','z','0','1','2','3','4','5','6','7','8','9','+','/'
};

char buf[MAXDATASIZE];
struct Args args;

void initNtrip(char* server, char* mountpoint, int port, char* user, char* password);
void initNtripDefault();
int encode(char *buf, int size, char *user, char *pwd);
int connect();

#endif