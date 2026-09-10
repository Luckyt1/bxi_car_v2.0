#ifndef BXI_CANFD_H
#define BXI_CANFD_H

#include <linux/can.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
    unsigned int bus;
    struct canfd_frame frame;
}can_packet __attribute__((__aligned__(8)));

typedef int (*canfd_rx_call)(void *arg, can_packet *msg);

int canfd_init(canfd_rx_call func, void *arg, int cpu);
int canfd_send(int num, can_packet *msg);
int pcie_gpio_set(unsigned int val);

#ifdef __cplusplus
}
#endif

#endif