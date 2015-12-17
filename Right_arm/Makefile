include /usr/src/rtlinux/rtl.mk

BPATH=../../bin/

all:   $(BPATH)right_arm

$(BPATH)right_arm: main.o hoap3_io.o kinematics.o matrixes.o
	$(CC) -g -O2 -Wall -lm -o $@ hoap3_io.o kinematics.o matrixes.o main.o

%.o: %.c
	$(CC) -g -O2 -Wall -D__LINUX__ $(INCLUDE) -I./ -I../../include -c $< -o $@
	
clean:
	rm $(BPATH)right_arm
