#! /usr/bin/env bash
# Here we setup the environment
# variables needed by the tinyos 
# make system

echo "Setting up for Tinyos-2.x for TKN154"
export TOSROOT=
export TOSDIR=
export MAKERULES=

TOSROOT=/home/kthwsn/workspace/apps.water-tank/SelfTriggered/se.kth.tinyos2x.mac.tkn154.gtsmod
TOSDIR=$TOSROOT/tos
CLASSPATH=$CLASSPATH:$TOSROOT/support/sdk/java/tinyos.jar:.
MAKERULES=$TOSROOT/support/make/Makerules
PATH=/usr/msp430/bin:$PATH
PYTHONPATH=.:$TOSROOT/support/sdk/python:$PYTHONPATH

export PYTHONPATH
export PATH
export TOSROOT
export TOSDIR
export CLASSPATH
export MAKERULES

