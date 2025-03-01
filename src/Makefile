CC = arm-linux-gnueabi-gcc
CFLAGS = -W -Wall -I/usr/local/include/
LDFLAGS = -L/usr/local/lib -lev3dev-c -lm -lpthread
sources = communication.c mdd.c myev3.c time_util.c workers.c main.c bal.c
objects = $(sources:.c=.o)
DEPS = $(sources:.c=.h)

robot_bin: $(objects)
	$(CC) -static -o $@ $^ $(LDFLAGS)
	rm -f *.o

%.o : %.c $(DEPS)
	$(CC) $(CFLAGS) -c $< -o $@ 

clean:
	rm -f *.o
	rm robot_bin