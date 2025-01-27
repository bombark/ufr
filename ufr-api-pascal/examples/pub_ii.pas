program TesteBiblioteca;
uses ufr;

var
    link: ufr_link;
    timer: ufr_link;
    res: integer;
    number: integer;

begin
    link := ufr_publisher('@new mqtt @coder msgpack @debug 5');
    timer := ufr_subscriber('@new posix:timer @time 2s');

    // ufr_write(link, 'opa\n', 4);
    number := 0;
    repeat
        res := ufr_put(link, 'iffs'+#10, number, 12.5, 33.1213, 'opa');
        writeln(res);
        ufr_recv(timer);
        number := number + 1;
    until number = 2;

    // ufr_publisher('@new sqlite:table @file pub_ii.db @sql ''insert into dados values(?,?)'' ');
    // ufr_output_init('@new zmq:topic @host 127.0.0.1 @coder std:csv @port 5000');
    // ufr_output_init('@new mqtt:topic @host 185.209.160.8 @topic teste @coder std:csv');
    // ufr_writeln('ii', 10, 20);
    // ufr_writeln('ii', 10, 20);
    // ufr_writeln('ii', 10, 20);
end.
