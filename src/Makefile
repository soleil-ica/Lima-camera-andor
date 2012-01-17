andor-objs = AndorCamera.o AndorInterface.o

SRCS = $(andor-objs:.o=.cpp) 

CXXFLAGS += -I../include -I../../../hardware/include -I../../../common/include \
			-I/usr/local/include \
			-Wall -pthread -fPIC -g

all:	Andor.o

Andor.o:	$(andor-objs)
	$(LD) -o $@ -r $+

clean:
	rm -f *.o *.P

%.o : %.cpp
	$(COMPILE.cpp) -MD $(CXXFLAGS) -o $@ $<
	@cp $*.d $*.P; \
	sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
	-e '/^$$/ d' -e 's/$$/ :/' < $*.d >> $*.P; \
	rm -f $*.d

-include $(SRCS:.cpp=.P)
