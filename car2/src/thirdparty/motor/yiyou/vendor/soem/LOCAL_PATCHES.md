# Local SOEM patches

Base: upstream SOEM v1.4.0. Keep local changes explicit when updating this copy.

## Linux nonblocking receive

`oshw/linux/nicdrv.c`: `ecx_recvpkt()` passes `MSG_DONTWAIT` to `recv()`, matching
[upstream SOEM](https://github.com/OpenEtherCATsociety/SOEM/blob/master/oshw/linux/nicdrv.c).
The receive loop already owns the response deadline. A blocking socket receive
inside `rx_mutex` can delay the other receiver when CoE mailbox traffic and PDO
cycles share the socket; a short `SO_RCVTIMEO` does not make the call nonblocking.
See [Linux recv(2)](https://man7.org/linux/man-pages/man2/recv.2.html).

On elf3-59/PHU14, concurrent SDO reads reproduced PDO `EC_NOFRAME` with both
1 ms and 2 ms master receive timeouts. This change passed the same test with
the original 1 ms timeout. It changes neither fault latching nor the host and
ESC watchdog limits. The offline `soem_nicdrv` test exercises the actual Linux
receive path using local sockets, without root privileges or EtherCAT hardware.
