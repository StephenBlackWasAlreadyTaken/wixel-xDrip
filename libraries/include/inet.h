#ifndef __INET_H__
#define __INET_H__

/* Wixel is little endian */
unsigned short htons(unsigned short n)
{
    return ((n & 0xFF) << 8) | ((n & 0xFF00) >> 8);
}

unsigned short ntohs(unsigned short n)
{
    return htons(n);
}

unsigned long htonl(unsigned long n)
{
    return ((n & 0xFF) << 24) | ((n & 0xFF00) << 8) | 
        ((n & 0xFF0000) >> 8) | ((n & 0xFF000000) >> 24);
}

unsigned long ntohl(unsigned long n)
{
    return htonl(n);
}

#endif
