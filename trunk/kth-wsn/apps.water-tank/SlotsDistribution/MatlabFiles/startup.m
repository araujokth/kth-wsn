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

%%projectFolder = '/home/kthwsn/workspace/apps.water-tank/SlotsDistribution/BoModification';
projectFolder = pwd;

addpath(projectFolder);
addpath([projectFolder '/comm']);               % Add the functions for the communication
addpath([projectFolder '/apps/callbacks']);     % When we receive a message from the baseStation
addpath([projectFolder '/apps']);               % Main application
addpath([projectFolder '/gui']);                % Graphical User Interface
addpath([projectFolder '/final_files']);        % Control functions


projectFolder = [projectFolder '/../../BoModification/'];

javaclasspath({[projectFolder 'BaseStation'], ...       % The java classes for the messages
    '/home/kthwsn/workspace/tinyos-2.x-svn/support/sdk/java/tinyos.jar', ... 
    });                                                 % Change the folder and point to your tinyos.jar which should
                                                        % include matlab libraries

    addpath /opt/matlab/other_toolbox/cvx
    addpath /opt/matlab/other_toolbox/cvx/structures
    addpath /opt/matlab/other_toolbox/cvx/lib
    addpath /opt/matlab/other_toolbox/cvx/functions
    addpath /opt/matlab/other_toolbox/cvx/commands
    addpath /opt/matlab/other_toolbox/cvx/builtins
    
defineTOSEnvironment;