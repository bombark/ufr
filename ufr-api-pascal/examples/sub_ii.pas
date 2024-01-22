program TesteBiblioteca;
uses ufr;

var
    v1, v2: Integer;

begin
    ufr_input_init('@new mqtt:topic @host 185.209.160.8 @topic teste @coder msgpack:obj');
    // ufr_input_init('@new zmq:topic @host 127.0.0.1 @port 5000 @coder std:csv');

    ufr_readln('^ii', @v1, @v2);

    // writeln(v1,v2);
end.
