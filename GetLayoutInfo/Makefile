#sigverse header
SIG_SRC  = $(SIGVERSE_PATH)/include/sigverse

#オブジェクトファイルの指定
OBJS     = CleanUpRobot.so TrashBox.so Entity.so PositionManager.so

all: $(OBJS)

#compile
./%.so: ./%.cpp
	g++ -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE -I$(SIG_SRC) -I$(SIG_SRC)/comm/controller -fPIC -shared -o $@ $<

clean:
	rm ./*.so
