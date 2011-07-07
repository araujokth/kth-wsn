# Top-level Makefile for tools/java 

SUBDIRS =
ROOT = ../../..
JAVA = $(shell ls *.java)


matlab: $(JAVA)
	javac $(JAVA);

msg:
	@echo 'You need to install matlab to use this package.\
		Please include MATLAB_DIR/java/jar/jmi.jar\
		to your classpath, then run "make matlab"';
.PHONY: matlab
