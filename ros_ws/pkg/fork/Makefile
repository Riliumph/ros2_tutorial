# compiler setting
COMPILER = g++ -v
CXXFLAGS = -g -MMD -Wall -Wextra -Winit-self -std=c++17

# library setting
LDFLAGS = -L/usr/local/lib
LIBS    = -lstdc++ -lpthread

# include setting
INCLUDE = -I$(SRCROOT)/
EXCLUDE = .git%

# binary setting
TARGET1  = $(shell basename $$(pwd))
BINROOT = bin

# source setting
SRCROOT = src
SRCS    = $(filter-out $(EXCLUDE) $(TEST_EXT), $(shell find $(SRCROOT) -name "*.cpp"))
SRCDIRS = $(dir $(SRCS))

# object setting
OBJROOT = obj
OBJDIRS = $(addprefix $(OBJROOT)/, $(patsubst $(SRCROOT)/%, %, $(SRCDIRS)))
OBJS    = $(addprefix $(OBJROOT)/, $(patsubst $(SRCROOT)/%.cpp, %.o, $(SRCS)))
TARGET1_OBJ = $(filter-out %/sub.o, $(OBJS))

# test setting
TEST_EXT= %_test.cpp
TESTLIBS = $(LIBS) -lpthread -lgtest -lgtest_main
TESTROOT = src
TESTSRCS = $(filter-out $(EXCLUDE) %main.cpp, $(shell find $(SRCROOT) -name "*.cpp"))
TESTOBJS = $(addprefix $(OBJROOT)/, $(patsubst $(TESTROOT)/%.cpp, %.o, $(TESTSRCS)))
TESTDIRS = $(dir $(TESTSRCS))

# dependency setting
DEPS = $(OBJS:.o=.d)

.PHONY: clean all analyze

all: $(TARGET1)
	@echo "make all"

test: $(TESTOBJS)
	@echo "build $< -> $@"
	@[ -d $(BINROOT) ] || mkdir -p $(BINROOT)
	$(COMPILER) -o $(BINROOT)/$@ $^ $(LDFLAGS) $(TESTLIBS)

$(TARGET1): $(TARGET1_OBJ)
	@echo "build exe $@"
	@[ -d $(BINROOT) ] || mkdir -p $(BINROOT)
	$(COMPILER) -o $(BINROOT)/$@ $^ $(LDFLAGS) $(LIBS)

$(TARGET2): $(TARGET2_OBJ)
	@echo "build exe $@"
	@[ -d $(BINROOT) ] || mkdir -p $(BINROOT)
	$(COMPILER) -o $(BINROOT)/$@ $^ $(LDFLAGS) $(LIBS)

$(OBJROOT)/%.o: $(SRCROOT)/%.cpp
	@echo "build $< -> $@"
	@[ -d $@ ] || mkdir -p $(dir $@)
	$(COMPILER) $(CXXFLAGS) $(INCLUDE) -o $@ -c $<

analyze:
	cppcheck --std=c++17 --enable=all $(SRCROOT)

fix_lib:
	@ldconfig

clean:
	@rm -rf $(OBJROOT) $(BINROOT)

-include $(DEPS)
