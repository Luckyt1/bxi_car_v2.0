#include <stdio.h>
#include <unistd.h>
#include "xcanfd_polled.h"

int func_call(void *arg, can_packet *msg){
    printf("recv can, bus: %d,  id: 0x%X, len: %d\n", msg->bus, msg->frame.can_id, msg->frame.len);
    for (int i = 0; i < msg->frame.len; i++){
        // printf(" %02X", msg->frame.data[i]);
        // printf("%c", msg->frame.data[i]);
    }
    // printf("\n");
    return 0;
}


int main(void)
{
    canfd_init(func_call, NULL, -1);

    pcie_gpio_set(0);
    sleep(1);
    pcie_gpio_set(0xff);
    printf("pwr on\n");

    while(1){
        char c = getchar();
        #define FRAME_NUM 1
        can_packet msg[FRAME_NUM];
        msg[0].bus = 2;
        msg[0].frame.can_id = 0x7f1;
        msg[0].frame.len = 1;
        msg[0].frame.data[0] = c;
        // for (int i = 0; i < msg[0].frame.len; i++){
        //     msg[0].frame.data[i] = i;
        // }
        // canfd_send(FRAME_NUM, msg);
        // printf("send can[1] id: 0x%X, len: %d\n", msg[0].frame.can_id, msg[0].frame.len);
        canfd_send(FRAME_NUM, msg);
        // printf("send: %c\n", c);
    }

    return 0;
}
