TARGET=example
OBJS=${TARGET}.o

CXX=g++
CC=gcc
CXXFLAGS=-g -Wall -Wextra
CFLAGS=-g -Wall -Wextra

LIBDIR=../../../lib
LIBLABVOLT=${LIBDIR}/liblabvolt.a

# Include the labvolt and usb libraries
INC=-I../../../inc
LIBSDIR=-L../../../lib
LIBS=${LIBSDIR} -llabvolt -ludev

all: ${TARGET}

# Compile lab
${TARGET}: ${OBJS}
	${CC} ${CFLAGS} -o ${TARGET} ${OBJS} ${INC} ${LIBS}

${OBJS}: ${TARGET}.c
	${CC} ${CFLAGS} -c -o ${TARGET}.o ${TARGET}.c ${INC} ${LIBS}


# If the compiler complains about a missing -llabvolt library type
# make labvolt
# Or it will auto generate it if the library isn't in LIBDIR


clean:
	rm -f *.o core* ${TARGET}
