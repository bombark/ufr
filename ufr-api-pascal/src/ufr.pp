unit ufr;

interface

type
    ufr_link = record
        // gtw_api: pointer;
        error: array [1..264] of char;
    end;


// Open Functions
function ufr_subscriber(text : pchar):ufr_link; cdecl; external 'ufr.so' name 'ufr_subscriber';
function ufr_publisher(text : pchar):ufr_link; cdecl; external 'ufr.so' name 'ufr_publisher';
function ufr_client(text : pchar):ufr_link; cdecl; external 'ufr.so' name 'ufr_client';
function ufr_server(text : pchar):ufr_link; cdecl; external 'ufr.so' name 'ufr_server';

// Close Function
procedure ufr_close(var link: ufr_link); cdecl; external 'ufr.so' name 'ufr_close';

// Sending Functions
function ufr_write(var link: ufr_link; buffer: pchar; nbytes: integer):integer; cdecl; external 'ufr.so' name 'ufr_write';
function ufr_put(var link: ufr_link; format: pchar):integer; cdecl; varargs; external 'ufr.so' name 'ufr_put';

// Receiving Functions
function ufr_recv(var link: ufr_link):integer; cdecl; varargs; external 'ufr.so' name 'ufr_recv';
function ufr_get(var link: ufr_link; format: pchar):integer; cdecl; varargs; external 'ufr.so' name 'ufr_get';


// procedure ufr_output_init(text : pchar); cdecl; external 'ufr.so' name 'ufr_output_init';
// procedure ufr_input_init(text : pchar); cdecl; external 'ufr.so' name 'ufr_input_init';
// procedure ufr_readln(text : pchar); cdecl; varargs; external 'ufr.so' name 'lt_input';
// procedure ufr_write(text : pchar); cdecl; varargs; external 'ufr.so' name 'ufr_output';
// procedure ufr_writeln(text : pchar); cdecl; varargs; external 'ufr.so' name 'ufr_output_ln';


implementation

// Implementação da função Somar
// function Somar(a, b: Integer): Integer;
// begin
//  Somar := a + b;
// end;

end.