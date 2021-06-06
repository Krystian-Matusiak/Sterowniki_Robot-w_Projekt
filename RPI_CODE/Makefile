#=========================== CVS =======================================
# $Author: $
# $Revision: $
# $State: $
# $Date: $
#=======================================================================



FILE=klient
OBJ=obj
SRC=src
APP=./klient
COMPFLAGS= -Iinc -I../LIVIa/inc/tools -W -Wall -pedantic 
LINKFLAGS= -W -Wall
LIBS=

__start__: ${OBJ} ${APP}
	rm -f core.* core; ${APP} "przyklad napisu"

${OBJ}:
	mkdir -p "${OBJ}"

${APP}: ${OBJ}/${FILE}.o
	g++ ${LINKFLAGS} -o ${APP} ${OBJ}/${FILE}.o ${LIBS} -lpthread -lwiringPi

${OBJ}/${FILE}.o: ${SRC}/${FILE}.cpp 
	g++  ${COMPFLAGS} -c -Wall -o ${OBJ}/${FILE}.o ${SRC}/${FILE}.cpp

clean:
	rm -f "${OBJ}"/* "${APP}" core.* core

clean-copies:
	find . \( -name \*~ -o -name \*\.flc -o -name \*.BAK \) -type f -print -exec rm {} \;

help:
	#
	#   make              - tworzenie i uruchomienie aplikacji
	#   make clean        - usuniecie produktow kompilacji i konsolidacji
	#   make clean-copies - usuniecie kopii edytowanych plikow
	#
