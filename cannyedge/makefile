# Compiler
CXX = g++

# Compiler flags
CXXFLAGS = -std=c++11 -Wall

# OpenCV flags
OPENCV = `pkg-config --cflags --libs opencv4`

# Object files
OBJS = canny_edge_detector_rtp.o

# Output executable
OUT = canny_edge_detector_rtp

# Target to build the executable
$(OUT): $(OBJS)
	$(CXX) $(OBJS) -o $(OUT) $(OPENCV)

# Rule to build .o files from .cpp
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< $(OPENCV)

# Clean up
clean:
	rm -f $(OBJS) $(OUT)

