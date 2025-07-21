SRC = $(wildcard src/*.cpp)
BIN = build/main

CXX = g++
CFLAGS = -Iinclude `pkg-config --cflags opencv4` -Wall -Wextra -O2
LDFLAGS = `pkg-config --libs opencv4`

build: $(SRC)
	mkdir -p build
	$(CXX) $(CFLAGS) $(SRC) -o $(BIN) $(LDFLAGS)
# Usage: ./build/main <checkerboard_width> <checkerboard_height> <checkerboard_size_mm> <calibrate_camera:0|1> <num_calib_images> <webcam_name>
run: build
	./$(BIN) 10 7 28.0 0 30 /dev/video2
clean:
	rm -rf build
