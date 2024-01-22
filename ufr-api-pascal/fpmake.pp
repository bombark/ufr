{$ifndef ALLPACKAGES}
{$mode objfpc}{$H+}
program fpmake;

uses fpmkunit;

Var
  P : TPackage;
  T : TTarget;
begin
  With Installer do
    begin
{$endif ALLPACKAGES}

    P:=AddPackage('ufr');
    P.ShortName:='ufr';
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='0.1.0';
    P.SourcePath.Add('src');
    P.OSes := AllUnixOSes-[qnx]+AllWindowsOSes;
    if Defaults.CPU=jvm then
      P.OSes := P.OSes - [java,android];

    P.IncludePath.Add('src');

    T:=P.Targets.AddUnit('ufr.pp');

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}