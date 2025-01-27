program Assinante;
uses ufr;

var
    link: ufr_link;
    v0, v1, v2: single;
    i: integer;

begin
    link := ufr_subscriber('@new mqtt @host 185.159.82.136 @coder msgpack');

    i := 0;
    repeat 
        ufr_get(link, '^fff', @v0, @v1, @v2);
        writeln(v0, v1, v2);
        i := i + 1;
    until i = 10;

    ufr_close(link);
end.
