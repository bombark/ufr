# 1. Publicador

```c
#include <ufr.h>

int main() {
    // abre um publicador
    link_t pub = ufr_publisher("@new zmq:topic @coder msgpack @debug 4"); 

    // escreve 5 mensagens
    for (int i=0; i<10; i++) {
        ufr_put(&pub, "ifs\n", 100, 2.23, "mensagem");
        sleep(1);
    }

    // fim
    ufr_close(&pub);
    return 0;
}
```


# 2. Assinante

```c
#include <ufr.h>

int main() {
    // abre um publicador
    link_t sub = ufr_publisher("@new zmq:topic @coder msgpack @debug 4"); 

    // lee 5 mensagens
    for (int i=0; i<10; i++) {
        int a;
        float b;
        char str[1024];
        ufr_get(&sub, "^ifs", &a, &b, str);
        printf("%d %f %s\n", a, b, str);
    }

    // fim
    ufr_close(&sub);
    return 0;
}
```


# 3. Cliente

# 4. Servidor


# 