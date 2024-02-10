# Unified Framework for Robotics (UFR)

[:uk: English](README.md) [:brazil: Portuguese](README.pt.md)

Robot projects typically use a robotics framework to modularize their code into small parts. This modularization is necessary to increase the portability of the same code for different robots and also to increase the sharing of complex algorithms for SLAM. It is worth remembering that Free Software is very important to optimize this sharing and allow all countries to advance technologically.

Since the 2000s, different frameworks have been developed. The architecture of most frameworks is based on the design of subscriber and publisher topics along with remote service invocation mechanisms. After a few decades, the Robot Operating System (ROS) framework has become a reference in this area. However, its learning curve is quite high, the code organization based on callbacks does not help with understanding, and the module code becomes tied to ROS.

This problem is the same as with operating systems. Each operating system offers an interface for accessing devices. An application programmer will use this interface. Therefore, the application code is tied to the target operating system. For example, applications made for Android are tied to Android. Windows and Linux also have the same problem. However, they adopted a common interface called POSIX to access mainly the file system, which allows code portability between them.

The POSIX interface has been enormously successful in handling files and threads. Therefore, there is great code portability between Windows and Linux as long as you are careful with the file path patterns, which are different. However, POSIX fails to port the graphical interface.

The advancement of IOT and microservices architectures for WEB systems has also boosted communication protocols such as MQTT, ZMQ and RabbitMQ. IOT projects are typically embedded systems and directly use the protocol functions. A simple one with low dependency is normally chosen due to the limited context of the embedded system. In WEB projects using microservices, they usually implement a small abstract interface to avoid tying the code to any protocols.

# Goals

This project proposes an interface based on POSIX that is portable between different communication protocols, easy to understand the code, high flexibility, easy implementation for different languages and easy to debug the code.

Development of an interface for different communication protocols inspired by the POSIX standard. This allows portability of the same code for different communication protocols. Therefore, the code is not restricted to the universe of a specific framework, as is the case with ROS modules.

# Requirements

- Portable between different communication protocols;
- The code of the modules must be easy to understand and using the close POSIX standard;
- Compilation of modules must be easy, and possible to use gcc or g++ directly;
- Development of new drivers must be easy and organized;
- Allow the use of the same functions for different languages, such as Python, Pascal and others;
- Allow flexibility of use between different protocols and different message encodings such as CSV, JSON, YAML or Msgpack;

# Building

```
mkdir build
cmake..
make
```

You can use ccmake to configure enable compilation of drivers that require other libraries.

# Publisher Example

publisher.c file
```
#include <stdio.h>
#include <ufr.h>

int main() {
    ufr_output_init("@new zmq:topic");
    // ufr_output_init("@new mqtt:topic");
    // ufr_output_init("@new ros_humble:topic");
    while (1) {
        int vel, rotvel;
        scanf("%d %d", &vel, &rotvel);
        ufr_output("ii\n", vel, rotvel);
    }
    return 0;
}
```

gcc publisher.c -o publisher -lufr

# Subscriber Example

subscriber.c file
```
#include <stdio.h>
#include <ufr.h>

int main() {
    ufr_input_init("@new zmq:topic");
    // ufr_input_init("@new mqtt:topic");
    // ufr_input_init("@new ros_humble:topic");
    while (1) {
        int vel, rotvel;
        ufr_input("^ii", &vel, &rotvel);
        printf("%d %d\n", vel, rotvel);
    }
    return 0;
}
```

gcc subscriber.c -o subscriber -lufr

# Server Example

server.c file
```
#include <stdio.h>
#include <ufr.h>

int main() {
    ufr_inoutput_init("@new zmq:socket");
    while (1) {
        char command[256];
        ufr_input("^s", command);
        if ( strcmp(command, "exit") == 0 ) {
            break;
        } else if ( strcmp(command, "ping") == 0 ) {
            ufr_output("s\n", "OK");
        } else {
            ufr_output("s\n", "ERROR");
        }
    }
    return 0;
}
```

gcc server.c -o server -lufr

# Client Example

client.c file
```
#include <stdio.h>
#include <ufr.h>

int main() {
    ufr_inoutput_init("@new zmq:socket");
    char buffer[256];
    ufr_output("s\n", "ping");
    ufr_input("^s", buffer);
    printf("%s\n", buffer);
    return 0;
}
```

gcc client.c -o client -lufr

# Current Limitations
- The code does not use callbacks, so there is a limitation on reading multiple devices from different sources or frequencies. Do not use callbacks as this considerably reduces understanding of the code. Since the project prioritizes understanding the code by different programmers rather than optimizing speed.

# Future works
- carry out generic tests to guarantee the same compartment for different drivers;
- improve the battery of tests for each driver;
- develop a select mechanism to read devices at different frequencies;
- make drivers for databases;
- create a driver for graphical interfaces, therefore proving that the interface can abstract event-based programming;

# List of Abbreviations
- gtw: gateway
- ecr: encoder
- dcr: decoder
- stk: stack
- cc: decoder and encoder
- app: application
