%defineTOSEnvironment
%
%This file should be run before working in the TOS/matlab environment
%
%It defines all the global variables and matlab paths usefule for working with TOS

% "Copyright (c) 2000 and The Regents of the University of California.  All rights reserved.
% Permission to use, copy, modify, and distribute this software and its documentation for any purpose, 
% without fee, and without written agreement is hereby granted, provided that the above copyright notice 
% and the following two paragraphs appear in all copies of this software.
% 
% IN NO EVENT SHALL THE UNIVERSITY OF CALIFORNIA BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, 
% INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, 
% EVEN IF THE UNIVERSITY OF CALIFORNIA HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
% THE UNIVERSITY OF CALIFORNIA SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
% THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED 
% HEREUNDER IS ON AN "AS IS" BASIS, AND THE UNIVERSITY OF CALIFORNIA HAS NO OBLIGATION TO
% PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS."
% 
% Authors:  Kamin Whitehouse <kamin@cs.berkeley.edu>
% Date:     May 10, 2002

global DEBUG                        %if this is 1 then debug info comes out
DEBUG = 0;

global COMM
%conventional broadcast address
COMM.TOS_BCAST_ADDR = 65535;

%conventional UART address
COMM.TOS_UART_ADDR = 126;

%GROUP ID for this set of nodes (used for sending packets, but could also be used for filtering packets)
COMM.GROUP_ID = hex2dec('22');

global TOSDIR            %the root directory of the matlab files
%[flag, TOSDIR] = system('ncc -print-tosdir');
[flag, TOSDIR] = system('echo $TOSDIR');
if(flag==0)
  TOSDIR=TOSDIR(1:end-1);
else
  TOSDIR=[];
  error('TOSDIR not defined')
  return
end

defineComm;
