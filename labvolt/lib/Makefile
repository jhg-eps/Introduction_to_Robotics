TARGET=labvolt
EOBJS=${TARGET}.o
LOBJS=serial.o
LOBJS1=5150.o

CC=gcc
AR=ar
CFLAGS=-g -Wall
LIBS=-L. -lserial

all: lib${TARGET}.a

# Pack the static library
lib${TARGET}.a: ${EOBJS} ${LOBJS} ${LOBJS1}
	${AR} crs lib${TARGET}.a ${EOBJS} ${LOBJS} ${LOBJS1}

# Compile the 5150 library
${LOBJS1}: 5150.c 5150USB.h
	${CC} ${CFLAGS} -c -o ${LOBJS1} 5150.c ${LIBS}

# Compile the user interface
${EOBJS}: labvolt.h
	${CC} ${CFLAGS} -c -o ${EOBJS} labvolt.c ${LIBS}


# Object dependencies
${TARGET}.a: ${EOBJS} ${LOBJS1} ${LOBJS}
${EOBJS}: ${TARGET}.c ${TARGET}.h ${LOBJS} ${LOBJS1}
${LOBJS1}: 5150.c 5150USB.h ${LOBJS}
${LOBJS}: serial.c serial.h


clean:
	rm -f *.o core* lib${TARGET}.a
