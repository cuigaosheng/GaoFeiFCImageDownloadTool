CC = g++

CFLAGES += $(shell pkg-config --cflags json-c) -g -Wall
LDFLAGS += $(shell pkg-config --libs json-c)

#CFLAGES += -I /home/cuigaosheng/ruimei_work/json-c/include -L /home/cuigaosheng/ruimei_work/json-c/libs/linux-gcc-4.9.2/ -g -Wall
#LDFLAGS += -ljson_linux-gcc-4.9.2_libmt


OBJS = px4_uploader_main.o
TARGETS = px4_uploader_main

RM = rm -f 

$(TARGETS):$(OBJS)
	$(CC) -o $(TARGETS) $(OBJS) $(CFLAGES) $(LDFLAGS)

$(OBJS):%.o:%.cpp
	$(CC) -c $(TARGETS) $< -o $@
clean:
	$(RM) $(TARGETS) $(OBJS)
