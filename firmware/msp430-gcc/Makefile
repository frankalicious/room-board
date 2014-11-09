CC=msp430-gcc
#CFLAGS=-Os -Wall -g -mmcu=msp430x2012
#CFLAGS=-Os -Wall -g -mmcu=msp430g2231
CFLAGS=-Os -g -mmcu=msp430g2553 \
	-Wall -Wextra -pedantic -Wundef -Wshadow \
	-Wmissing-prototypes -Wmissing-declarations \
	-Wredundant-decls -Wunreachable-code
#OBJS=uart.o main.o adc.o DHT11_LIB.o
SRC = main.c
SRC += uart/uart.c
SRC += adc/adc.c
SRC += DHT11_LIB/DHT11_LIB.c
OBJS = $(SRC:%.c=%.o)


all: $(OBJS)
	$(CC) $(CFLAGS) -o main.elf $(OBJS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -fr main.elf $(OBJS) *~ *.bk

program:
	mspdebug rf2500 "prog main.elf"

server:
	mspdebug rf2500 "gdb"

gdb:
	msp430-gdb -x gdbinit
