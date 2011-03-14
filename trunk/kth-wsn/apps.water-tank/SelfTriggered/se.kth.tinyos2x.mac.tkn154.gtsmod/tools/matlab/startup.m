%This is a template for the startup.m file.
%
%Matlab automatically starts a session by running the script startup.m,
%invoking this file if it is in your Matlab path.  In Windows, this
%file will probably be called C:\matlab\bin\startup.m or maybe
%C:\matlab\R12\work\startup.m. In Unix, this file should be in
%~/matlab/startup.m
%
%The code below will find and run the defineTOSEnvironment script,
%which should always be called before using tinyOS with matlab.
%This is useful if you don't want to run the defineTOSEnvironment
%script by hand each time you start matlab.

%[flag, value] = system('ncc -print-tosdir');
flag=0; value = '/home/aitorhh/workspace/se.kth.tinyos2x.mac.tkn154/tools/matlab';
if(flag==0)
  addpath(value);
  addpath([value '/comm']);
  addpath([value '/comm/callbacks']);
  addpath([value '/apps']);

  defineTOSEnvironment;
else
  error('Cant find the defineTOSEnvironment file')
  return
end

