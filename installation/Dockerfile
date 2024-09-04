FROM nvidia/cuda:12.2.2-devel-ubuntu20.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

RUN apt-get update && apt-get -y install tzdata

RUN apt-get update && apt-get install -y \
    freeglut3-dev \
    libglfw3-dev \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
  && apt-get install -y build-essential \
      gcc \
      g++ \
      gdb \
      clang \
      make \
      ninja-build \
      cmake \
      autoconf \
      automake \
      libtool \
      valgrind \
      locales-all \
      dos2unix \
      rsync \
      tar \
      python \
      python-dev \
      wget\
  && apt-get clean



# Install Eigen3
RUN apt-get update && apt-get install -y \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# Install OpenCV

RUN apt-get update && apt-get install -y \
    git \
    pkg-config \
    libgtk-3-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    libxvidcore-dev \
    libx264-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    gfortran \
    openexr \
    libatlas-base-dev \
    python3-dev \
    python3-numpy \
    libtbb2 \
    libtbb-dev \
    libdc1394-dev \
    libopenexr-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer1.0-dev \
    libhdf5-dev \
    && rm -rf /var/lib/apt/lists/*

# Clone OpenCV and OpenCV contrib repositories
RUN git clone https://github.com/opencv/opencv.git /tmp/opencv && \
    git clone https://github.com/opencv/opencv_contrib.git /tmp/opencv_contrib

# Configure OpenCV build with CMake
RUN mkdir -p /tmp/opencv/build
RUN cd /tmp/opencv/build && \
    cmake -D CMAKE_BUILD_TYPE=RELEASE \
          -D CMAKE_INSTALL_PREFIX=/usr/local \
          -D INSTALL_C_EXAMPLES=ON \
          -D INSTALL_PYTHON_EXAMPLES=ON \
          -D OPENCV_GENERATE_PKGCONFIG=ON \
          -D OPENCV_EXTRA_MODULES_PATH=/tmp/opencv_contrib/modules \
          -D BUILD_EXAMPLES=ON \
          ..

# Build and install OpenCV
RUN cd /tmp/opencv/build && \
    make -j"$(nproc)" && \
    make install

# Verify OpenCV installation
RUN pkg-config --modversion opencv4

# Clean up
RUN rm -rf /tmp/opencv /tmp/opencv_contrib

# Install Ceres
RUN apt-get update && apt-get install -y\
	libgflags2.2\
	libgflags-dev\
	libgoogle-glog-dev
RUN mkdir /tmp/ceres &&\
	cd /tmp/ceres &&\
	wget http://ceres-solver.org/ceres-solver-2.1.0.tar.gz &&\
	tar zxf ceres-solver-2.1.0.tar.gz
RUN mkdir /tmp/ceres/ceres-bin &&\
	cd /tmp/ceres/ceres-bin &&\
	cmake ../ceres-solver-2.1.0 &&\
	make -j3 &&\
	make install

# Install COLMAP
RUN apt-get update && apt-get install -y \
    libboost-program-options-dev \
    libboost-filesystem-dev \
    libboost-graph-dev \
    libboost-system-dev \
    libeigen3-dev \
    libflann-dev \
    libfreeimage-dev \
    libmetis-dev \
    libgoogle-glog-dev \
    libgtest-dev \
    libsqlite3-dev \
    libglew-dev \
    qtbase5-dev \
    libqt5opengl5-dev \
    libcgal-dev \
    libceres-dev \
    && rm -rf /var/lib/apt/lists/*

# Clone COLMAP repository
RUN git clone https://github.com/colmap/colmap.git /tmp/colmap
RUN cd /tmp/colmap && git checkout 1f69517d360463bf2e8936fdd1f1e61f719ea6eb
RUN mkdir -p /tmp/colmap/build

# Set CUDA_TOOLKIT_ROOT_DIR
ENV CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda

# Check if CUDA is installed and set the compiler accordingly
RUN if command -v nvcc > /dev/null 2>&1; then \
      echo "CUDA found, configuring with CUDA support"; \
      cd /tmp/colmap/build && \
      cmake .. -GNinja \
        -DCMAKE_CUDA_ARCHITECTURES=native \
        -DCUDA_TOOLKIT_ROOT_DIR=$CUDA_TOOLKIT_ROOT_DIR \
        -DCMAKE_CUDA_COMPILER=$CUDA_TOOLKIT_ROOT_DIR/bin/nvcc; \
    else \
      echo "CUDA not found, configuring without CUDA support"; \
      cd /tmp/colmap/build && \
      cmake .. -GNinja; \
    fi

# Build COLMAP with Ninja
RUN cd /tmp/colmap/build && ninja
RUN cd /tmp/colmap/build && ninja install

# Clean up
RUN rm -rf /tmp/colmap



ARG UID=1000
RUN useradd -m -u ${UID} -s /bin/bash builder
USER builder

