CXX = g++

ROOTDIR = ./

LIB_PATH := -L$(ROOTDIR)lib
# LIB_PATH += -L$(ROOTDIR)lib/opencv4/3rdparty
# LIB_PATH += -L$(ROOTDIR)lib

CXXFLAGS := -Wall -g -std=c++11
LDFLAGS := $(LIB_PATH)
LIB := -ldl -lpthread -lopencv_world

INCLUDES := -I$(ROOTDIR)include
INCLUDES += -I$(ROOTDIR)include/opencv4/opencv2
INCLUDES += -I$(ROOTDIR)include/opencv4
# 列出所有的源文件
SRCS = $(ROOTDIR)/main.cpp

# 生成目标文件列表
OBJS = $(SRCS:.cpp=.o)

EXECU = test_sgbm
# 默认目标
all: $(EXECU)

# 生成可执行文件
$(EXECU): $(OBJS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(LDFLAGS) $(LIB_PATH) -o $@ $(OBJS) $(LIB)

# 编译每个源文件为目标文件
%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c -o $@ $<

# 清理规则
clean:
	rm -f *.o $(EXECU)*
