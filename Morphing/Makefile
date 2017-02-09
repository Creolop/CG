#define the compiler
CC = g++

#define any compile-time flags
#here include the -pg, however there is no gprofiler in my computer
CFLAGS = -Wall

#define any directories containing header files other than ./ and /usr/include
INCLUDES = -I/usr/local/include

#define library paths if essential
LFLAGS = `pkg-config --libs opencv`

#define any libraries to link into excutable


#define the C source files
SRCS = main.cpp class.cpp

#define the C object files
#Pay attention: This uses Suffix Replacement within a macro:
# $(name:string1=string2)
OBJS = $(SRCS:.cpp=.o)

#define target file
MAIN = Morphing

.PHONY:clean

all: $(MAIN) 
	@echo Done.

$(MAIN):$(OBJS)
	@$(CC) $(CFLAGS) $(INCLUDES) -o $(MAIN) $(OBJS) $(LFLAGS)

.cpp.o:
	@$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

clean:
	rm -r *.o
