program TesteBiblioteca;
uses ufr;


begin
    ufr_output_init('@new sqlite:table @file pub_ii.db @sql ''insert into dados values(?,?)'' ');
    // ufr_output_init('@new zmq:topic @host 127.0.0.1 @coder std:csv @port 5000');
    // ufr_output_init('@new mqtt:topic @host 185.209.160.8 @topic teste @coder std:csv');

    ufr_writeln('ii', 10, 20);
    ufr_writeln('ii', 10, 20);
    ufr_writeln('ii', 10, 20);
end.
