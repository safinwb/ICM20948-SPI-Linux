# Compiler and flags
CXX = g++
CXXFLAGS = -w

# Target executable
TARGET = ins

# Source and header files
SRCS = ins.cpp AHRSAlgorithms.cpp
OBJS = $(SRCS:.cpp=.o)

# Default target
all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $<

clean:
	rm -f $(TARGET) *.o
