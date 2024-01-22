unit ufr;

interface

type
    Link = record
        gtw_api: pointer;
        gtw_shr: pointer;
        gtw_obj: pointer;
        ecr_api: pointer;
        ecr_obj: pointer;
        dcr_api: pointer;
        dcr_obj: pointer;
        error: array [1..128] of char;
    end;


function ufr_new(text : pchar):Link; cdecl; external 'ufr.so' name 'lt_new';
function ufr_publisher(text : pchar):Link; cdecl; external 'ufr.so' name 'ufr_publisher';

procedure ufr_output_init(text : pchar); cdecl; external 'ufr.so' name 'ufr_output_init';
procedure ufr_input_init(text : pchar); cdecl; external 'ufr.so' name 'ufr_input_init';

procedure ufr_readln(text : pchar); cdecl; varargs; external 'ufr.so' name 'lt_input';

procedure ufr_write(text : pchar); cdecl; varargs; external 'ufr.so' name 'ufr_output';
procedure ufr_writeln(text : pchar); cdecl; varargs; external 'ufr.so' name 'ufr_output_ln';


implementation

// Implementação da função Somar
// function Somar(a, b: Integer): Integer;
// begin
//  Somar := a + b;
// end;

end.