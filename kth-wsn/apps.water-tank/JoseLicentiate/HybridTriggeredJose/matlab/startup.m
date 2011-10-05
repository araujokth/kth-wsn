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

projectFolder = '/home/kthwsn/workspace/apps.water-tank/HybridJoseXtraSensors/';

value = [projectFolder 'matlab/'];
addpath(value);
addpath([value 'functions/']);
addpath([value 'gui/']);
addpath([value 'mat/']);

javaclasspath({[projectFolder 'apps/BaseStation'], ...
    '/home/kthwsn/workspace/tinyos-2.x-svn/support/sdk/java/tinyos.jar', ...
    });
    
defineTOSEnvironment;