TARGET     = sweep_laser_scan_client
PLAYERINCLUDE=/home/bender/local/include/player-2.2
PLAYERLIBS=/home/bender/local/lib
INCS=-I$(PLAYERINCLUDE)/libplayerc -I$(PLAYERINCLUDE)
LIBS=-L$(PLAYERLIBS) -lplayerc -lncurses -lplayercore -lplayerxdr
CC = g++

all: $(TARGET).c
	$(CC) $(INCS) -c -o $(TARGET).o $<
	$(CC) $(LIBS) $(TARGET).o -o $(TARGET)

clean:
	rm -f *.o *.so $(TARGET) *~