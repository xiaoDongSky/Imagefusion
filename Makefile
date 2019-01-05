libprotobuf-devCC := g++
CFLAGS += -std=c++11 -O3 -Wall
lib := libcanParser.a

LIB := libcontrolcan.a

src := $(wildcard *.cpp)
src += $(wildcard *.cc)
obj := $(patsubst %.cpp,%.o, $(src))

all:test

$(obj):$(src)
	$(CC) -c $(CFLAGS) $(src) 
	

$(LIB):$(obj)
	rm -f $@
	ar cr $@ $(obj) $(lib) 

.PHONY:test
test:main.o
	g++ -O3 -o main main.cpp CCarCamCal.cpp -L . -I/usr/include -lcontrolcan -lSDK -lopencv_highgui -lopencv_core -lopencv_imgproc -lopencv_calib3d -std=c++11 -lpthread -lprotobuf

#.PHONY:clean
clean:
	rm -f $(obj)
