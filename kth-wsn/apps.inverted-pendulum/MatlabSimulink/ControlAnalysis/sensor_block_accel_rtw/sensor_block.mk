# Copyright 1994-2008 The MathWorks, Inc.
#
# File    : accel_unix.tmf   $Revision: 1.27.4.24 $
#
# Abstract:
#	Accelerator template makefile for building a UNIX-based,
#       RTW generated mex-file of Simulink model using generated C code.
#
# 	This makefile attempts to conform to the guidelines specified in the
# 	IEEE Std 1003.2-1992 (POSIX) standard. It is designed to be used
#       with GNU Make which is located in matlabroot/rtw/bin.
#
# 	Note that this template is automatically customized by the Real-Time
#	Workshop build procedure to create "<model>.mk"
#
#       The following defines can be used to modify the behavior of the
#	build:
#
#         MEX_OPTS       - User specific mex options.
#	  OPT_OPTS       - Optimization options. Default is -O. To enable 
#                          debugging specify as OPT_OPTS=-g. 
#                          Because of optimization problems in IBM_RS, 
#                          default is no-optimization. 
#	  USER_SRCS      - Additional user sources, such as files needed by
#			   S-functions.
#	  USER_INCLUDES  - Additional include paths
#			   (i.e. USER_INCLUDES="-Iwhere-ever -Iwhere-ever2")
#
#       This template makefile is designed to be used with a system target
#       file that contains 'rtwgensettings.BuildDirSuffix' see accel.tlc

#------------------------ Macros read by make_rtw ------------------------------
#
# The following macros are read by the Real-Time Workshop build procedure:
#
#  MAKECMD         - This is the command used to invoke the make utility
#  HOST            - What platform this template makefile is targeted for
#                    (i.e. PC or UNIX)
#  BUILD           - Invoke make from the Real-Time Workshop build procedure
#                    (yes/no)?
#  SYS_TARGET_FILE - Name of system target file.

MAKECMD         = /home/aitorhh/Programs/matlab/bin/glnx86/gmake
HOST            = UNIX
BUILD           = yes
SYS_TARGET_FILE = accel.tlc
COMPILER_TOOL_CHAIN = unix

#---------------------- Tokens expanded by make_rtw ----------------------------
#
# The following tokens, when wrapped with "|>" and "<|" are expanded by the
# Real-Time Workshop build procedure.
#
#  MODEL_NAME      - Name of the Simulink block diagram
#  MODEL_MODULES   - Any additional generated source modules
#  MAKEFILE_NAME   - Name of makefile created from template makefile <model>.mk
#  MATLAB_ROOT     - Path to were MATLAB is installed.
#  S_FUNCTIONS     - List of S-functions.
#  S_FUNCTIONS_LIB - List of S-functions libraries to link.
#  SOLVER          - Solver source file name
#  NUMST           - Number of sample times
#  TID01EQ         - yes (1) or no (0): Are sampling rates of continuous task
#                    (tid=0) and 1st discrete task equal.
#  NCSTATES        - Number of continuous states
#  COMPUTER        - Computer type. See the MATLAB computer command.
#  MEXEXT          - extension that a mex file has. See the MATLAB mexext 
#                    command
#  BUILDARGS       - Options passed in at the command line.

MODEL              = sensor_block
MODULES            = rtGetInf.c rtGetNaN.c rt_nonfinite.c rt_zcfcn.c sensor_block_acc_data.c 
MAKEFILE           = sensor_block.mk
MATLAB_ROOT        = /home/aitorhh/Programs/matlab
ALT_MATLAB_ROOT    = /home/aitorhh/Programs/matlab
MASTER_ANCHOR_DIR  = 
START_DIR          = /home/aitorhh/KTH/reglerteknik/ControlAnalysis
S_FUNCTIONS        = sf_sfun.c
S_FUNCTIONS_LIB    = 
SOLVER             = 
NUMST              = 2
TID01EQ            = 0
NCSTATES           = 0
MEM_ALLOC          = RT_STATIC
COMPUTER           = GLNX86
MEXEXT             = mexglx
BUILDARGS          =  GENERATE_REPORT=0
PURIFY             = 0
MODELREFS          = 
SHARED_SRC         = 
SHARED_SRC_DIR     = 
SHARED_BIN_DIR     = 
SHARED_LIB         = 
TARGET_LANG_EXT    = c
OPTIMIZATION_FLAGS = -O0 -ffloat-store -fPIC -m32 -DNDEBUG
ADDITIONAL_LDFLAGS = 


#--------------------------- Model and reference models -----------------------
MODELLIB                  = sensor_blocklib.a
MODELREF_LINK_LIBS        = 
MODELREF_INC_PATH         = 
RELATIVE_PATH_TO_ANCHOR   = ..
# NONE: standalone, SIM: modelref sim, RTW: modelref rtw
MODELREF_TARGET_TYPE       = NONE

#-- In the case when directory name contains space ---
ifneq ($(MATLAB_ROOT),$(ALT_MATLAB_ROOT))
MATLAB_ROOT := $(ALT_MATLAB_ROOT)
endif

#--------------------------- Tool Specifications -------------------------------
include $(MATLAB_ROOT)/rtw/c/tools/unixtools.mk

CC = $(MATLAB_ROOT)/bin/mex

#------------------------------ Include Path -----------------------------------
MATLAB_INCLUDES = \
	-I$(MATLAB_ROOT)/simulink/include \
	-I$(MATLAB_ROOT)/extern/include \
	-I$(MATLAB_ROOT)/rtw/c/src

# Additional includes

ADD_INCLUDES = \
	-I$(START_DIR)/sensor_block_accel_rtw \
	-I$(START_DIR) \


SHARED_INCLUDES =
ifneq ($(SHARED_SRC_DIR),)
SHARED_INCLUDES = -I$(SHARED_SRC_DIR) 
endif

INCLUDES = -I. -I$(RELATIVE_PATH_TO_ANCHOR) $(MATLAB_INCLUDES) $(ADD_INCLUDES) $(USER_INCLUDES) \
	$(MODELREF_INC_PATH) $(SHARED_INCLUDES)

#-------------------------------- Mex Options  ---------------------------------
MEX_OPTS  = 

ifndef MEX_OPT_FILE
MEX_OPT_FILE = 
endif

ifndef OPT_OPTS
OPT_OPTS  = $(DEFAULT_OPT_OPTS)
endif

GCC_WARN_OPTS := 

ifneq ($(ADDITIONAL_LDFLAGS),)
MEX_LDFLAGS = LDFLAGS='$$LDFLAGS $(ADDITIONAL_LDFLAGS)'
else
MEX_LDFLAGS =
endif

# See rtw/c/tools/unixtools.mk for the definition of GCC_WARN_OPTS
ifeq ($(PURIFY),1)
   MEX_FLAGS = $(MEX_OPTS) -g COPTIMFLAGS="$(GCC_WARN_OPTS)" $(MEX_LDFLAGS) $(MEX_OPT_FILE) 
else
  ifeq ($(MEX_OPTS),-g)
    MEX_FLAGS = -g COPTIMFLAGS="$(GCC_WARN_OPTS)" $(MEX_LDFLAGS) $(MEX_OPT_FILE)
  else
    ifeq ($(OPT_OPTS),-g)
     MEX_FLAGS = $(MEX_OPTS) -g COPTIMFLAGS="$(GCC_WARN_OPTS)" $(MEX_LDFLAGS) $(MEX_OPT_FILE)
    else
       ifneq ($(OPTIMIZATION_FLAGS),)	
           MEX_FLAGS = $(MEX_OPTS) COPTIMFLAGS="$(OPTIMIZATION_FLAGS) $(GCC_WARN_OPTS)" $(MEX_LDFLAGS) $(MEX_OPT_FILE)
       else
           MEX_FLAGS = $(MEX_OPTS) COPTIMFLAGS="$(OPT_OPTS) -DNDEBUG $(GCC_WARN_OPTS)" $(MEX_LDFLAGS) $(MEX_OPT_FILE)
       endif
    endif
  endif
endif

#----------------------------- Source Files -----------------------------------
USER_SRCS =

USER_OBJS       = $(USER_SRCS:.c=.o)
LOCAL_USER_OBJS = $(notdir $(USER_OBJS))

SRCS      = $(MODEL)_acc.$(TARGET_LANG_EXT) $(MODULES)
SRCS_OBJS = $(addsuffix .o, $(basename $(SRCS)))
OBJS      = $(SRCS_OBJS) $(USER_OBJS)
LINK_OBJS = $(SRCS_OBJS) $(LOCAL_USER_OBJS)

SHARED_SRC := $(wildcard $(SHARED_SRC))
SHARED_OBJS = $(addsuffix .o, $(basename $(SHARED_SRC)))

#-------------------------- Additional Libraries ------------------------------

LIBS =
 
LIBS += $(S_FUNCTIONS_LIB)

ifeq ($(PURIFY),1)
LIBUT = -L$(MATLAB_ROOT)/bin/PURIFY/$(ARCH) -lut
LIBMWMATHUTIL = -L$(MATLAB_ROOT)/bin/PURIFY/$(ARCH) -lmwmathutil
else
LIBUT = -L$(MATLAB_ROOT)/bin/$(ARCH) -lut
LIBMWMATHUTIL = -L$(MATLAB_ROOT)/bin/$(ARCH) -lmwmathutil
endif

MATHLIBS = $(LIBUT) $(LIBMWMATHUTIL)

PROGRAM = ../$(MODEL)_acc.$(MEXEXT)

#--------------------------------- Rules --------------------------------------

$(PROGRAM) : $(OBJS) $(SHARED_LIB) $(LIBS)
	@echo "### Linking ..."
	$(CC) $(MEX_FLAGS) -MATLAB_ARCH=$(ARCH) -silent -outdir .. $(LINK_OBJS) $(SHARED_LIB) $(LIBS) $(MATHLIBS) 
	@echo "### Created mex file: $(MODEL)_acc.$(MEXEXT)"

%.o : %.c
	$(CC) -c $(MEX_FLAGS) $(INCLUDES) "$<"

%.o : %.cpp
	$(CC) -c $(MEX_FLAGS) $(INCLUDES) "$<"

%.o : ../%.c
	$(CC) -c $(MEX_FLAGS) $(INCLUDES) "$<"

%.o : ../%.cpp
	$(CC) -c $(MEX_FLAGS) $(INCLUDES) "$<"

%.o : $(MATLAB_ROOT)/rtw/c/src/%.c
	$(CC) -c $(MEX_FLAGS) $(INCLUDES) "$<"


%.o : $(MATLAB_ROOT)/rtw/c/src/%.cpp
	$(CC) -c $(MEX_FLAGS) $(INCLUDES) "$<"


%.o : $(MATLAB_ROOT)/simulink/src/%.c
	$(CC) -c $(MEX_FLAGS) $(INCLUDES) "$<"

#------------------------------- Libraries -------------------------------------






clean :
	@echo "### Deleting the objects, libraries and $(PROGRAM)"
	@\rm -f $(wildcard *.o) $(PROGRAM)

#----------------------------- Dependencies -------------------------------

$(OBJS) : $(MAKEFILE) rtw_proj.tmw

$(SHARED_LIB) : $(SHARED_SRC)
	@echo "### Creating $@ "
	cd $(SHARED_BIN_DIR); $(CC) -c $(MEX_FLAGS) $(INCLUDES) $(notdir $?)
	ar ruvs $@ $(SHARED_OBJS)
	@echo "### $@ Created "

