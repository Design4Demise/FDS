# FDS makefile

CC=g++
CFLAGS=--std=c++0x

EXE=FDS

DIR=.
SOURCEDIR=$(DIR)/src
OBJECTDIR=$(DIR)/obj

SRCS:=$(wildcard $(SOURCEDIR)/*.cpp)
OBJS:=$(addprefix $(OBJECTDIR)/, $(notdir $(SRCS:.cpp=.o)))

LIB=-lboost_system -lboost_filesystem

# Build FDS
$(EXE): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LIB)

# Build object files
$(OBJS): $(OBJECTDIR)/%.o : $(SOURCEDIR)/%.cpp
	$(CC) $(CFLAGS) -c -o $@ $<

.PHONY: clean
clean:
	rm -rf $(EXE) $(OBJECTDIR) Results && mkdir $(OBJECTDIR) Results

.PHONY: depend
depend:
	makedepend -Y -- $(CFLAGS) -- $(SRCS)
