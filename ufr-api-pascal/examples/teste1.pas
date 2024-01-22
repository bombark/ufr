program Hello;
uses dynlibs, urf;

// https://forum.lazarus.freepascal.org/index.php?topic=60112.0

type
    Link = record
        gw_api: pointer;
        gw_shr: pointer;
        gw_obj: pointer;
        enc_api: pointer;
        enc_obj: pointer;
        dec_api: pointer;
        dec_obj: pointer;
        error: array [0..128] of char;
    end;

    TMyFunc = function (text: pchar):Link; cdecl;
    TLtInput = procedure (text: pchar); cdecl; varargs;
    TLtOutput = procedure (text: pchar); cdecl; varargs;

var
    handle: TLibHandle;
    nota, nota1, nota2: integer;
    lt_new_ftr: TMyFunc;
    lt_input_ftr: TLtInput;
    lt_output_ftr: TLtOutput;
    entrada: Link;


begin
    handle := LoadLibrary('liblt_api.so');

    nota1 := 10;
    nota2 := 20;
    
    lt_input_ftr := TLtInput( GetProcedureAddress(handle, 'lt_input') );
    lt_input_ftr('^ii', @nota1, @nota2);

    lt_output_ftr := TLtOutput( GetProcedureAddress(handle, 'lt_output') );
    lt_output_ftr('ii', nota1, nota2);

    lt_new_ftr := TMyFunc( GetProcedureAddress(handle, 'lt_new') );
    entrada := lt_new_ftr('@new zmq:socket @host 127.0.0.1 @port 5000');

    // nota := 0;
    // writeln(nota,nota1,nota2)
end.