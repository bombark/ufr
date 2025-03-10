Projetos de robôs normalmente utilizam um framework de robótica para modularizar seu código em pequenas partes. Essa modularização torna-se necessária para aumentar a portabilidade do mesmo código para diferentes robos e também aumentar o compartilhamento dos complexos algoritmos para o SLAM. Vale lembrar que o Software Livre é muito importante para otimizar este compartilhamento e permitir todos os países avançarem tecnologicamente.

A partir do anos 2000, elaborou-se diferentes frameworks. A arquitetura da maioria dos frameworks se baseia no design de topicos de assinante e publicante junto com mecanismos de chamada remota de serviço. Após algumas decadas o framework Robot Operating System (ROS) é se tornou referencia nesta area. Porém sua curva de aprendizagem é bastante alta, a organização do código baseada em callbacks não ajuda no entendimento, e o código dos módulos se torna atrelada ao ROS.

Este problema é o mesmo dos sistemas operacionais. Cada sistema operacional oferece uma interface para acessar os dispositivos. O programador de um aplicativo utilizará essa interface. Desde modo, o código do aplicativo fica atrelado ao sistema operacional alvo. Por exemplo, os aplicativos feito para Android estão atrelados ao Android. O Windows e Linux também tem o mesmo problema. Entretanto eles adotaram uma interface comum chamada POSIX para o acesso principalmente ao sistema de arquivo, o que permite uma portabilidade do código entre eles.

A interface POSIX teve um enorme sucesso na manipulação de arquivos e threads. logo tem uma grande portabilidade do código entre Windows e Linux desde que tomando cuidado com o padrão de caminho dos arquivos, que são diferentes. Porém POSIX falha na portabilidade para a interface gráfica.

O avanço do IOT e arquiteturas de microserviços para sistemas WEBs também impulsionou protocolos de comunicação como MQTT, ZMQ e RabbitMQ. Os projetos de IOT normalmente são sistemas embarcados e utilizam direto as funções do protocolo. Escolhe-se normalmente uma simples com baixa dependencia devido ao contexto limitado do sistema embarcado. Já em projetos WEBs usando microserviços normalmente implementam uma pequena interface abstrata para evitar atrelar o código à algum protocolos.

# Objetivos

Este projeto propõem uma interface baseada no POSIX que seja portável entre diferentes protocolos de comunicação, fácil entendimento do código, alta flexibilidade, fácil implementação para diferentes linguagens e fácil depuração do código.

Desenvolvimento de uma interface para os diferentes protocolos de comunicação inspirada no padrão POSIX. Desde modo permite-se a portabilidade de um mesmo código para diferentes protocolos de comunicação. Portanto o código não fica restrito ao universo de um de terminado framework, como ocorre com os módulos do ROS.

# Requisitos do Projeto

- Portável entre diferentes protocolos de comunicação;
- O código dos módulos deve ser fácil de entender e usando o padrão próximo POSIX;
- A compilação dos módulos deve ser fácil, e possível de usar gcc ou g++ diretamente;
- Desenvolvimento de novos drivers devem ser faceis e organizados;
- Permitir o uso das mesmas funções para diferentes linguanges, como Python, Pascal e outros;
- Permitir uma flexibilidade de uso entre diferentes protocolo e diferentes codificação de mensagem como CSV, JSON, YAML ou Msgpack;
