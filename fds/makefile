# FDS makefile

CC=g++
CFLAGS=--std=c++0x

EXE=FDS

DIR=.
SOURCEDIR=$(DIR)/src
HEADERDIR=$(DIR)/inc
OBJECTDIR=$(DIR)/obj

SRCS:=$(wildcard $(SOURCEDIR)/*.cpp)
OBJS:=$(addprefix $(OBJECTDIR)/, $(notdir $(SRCS:.cpp=.o)))

# Build FDS
$(EXE): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^

# Build object files
$(OBJS): $(OBJECTDIR)/%.o : $(SOURCEDIR)/%.cpp
	$(CC) $(CFLAGS) -c -o $@ $<

.PHONY: clean
clean:
	rm -rf $(EXE) $(OBJECTDIR) && mkdir $(OBJECTDIR)

.PHONY: depend
depend:
	makedepend -Y -- $(CFLAGS) -- $(SRCS)