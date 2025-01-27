
```c
#include <stdio.h>
#include <ufr.h>

int main() {
    // open publisher
    link_t pub = ufr_publisher("@new zmq:topic");

    // write 5 messages
    for (int i=0; i<5; i++) {
        ufr_put(&pub, "iiis\n", 100, 2434, 123344, "mensagem");
        sleep(1);
    }

    // end
    ufr_close(&pub);
    return 0;
}
```