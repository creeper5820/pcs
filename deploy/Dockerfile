FROM ubuntu:22.04

SHELL ["/bin/bash", "-lc"]

ARG DEBIAN_FRONTEND=noninteractive
ARG VTK_VERSION=9.3.1
ARG VTK_SERIES=9.3

ENV PATH="/usr/lib/qt6/bin:/usr/local/bin:${PATH}" \
    CC=gcc-14 \
    CXX=g++-14 \
    CMAKE_PREFIX_PATH=/opt/vtk \
    VTK_DIR=/opt/vtk/lib/cmake/vtk-${VTK_SERIES} \
    LD_LIBRARY_PATH=/opt/vtk/lib

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      software-properties-common \
      ca-certificates \
      curl \
      git \
      gnupg \
 && add-apt-repository -y ppa:ubuntu-toolchain-r/test \
 && apt-get update \
 && apt-get install -y --no-install-recommends \
      cmake \
      file \
      gcc-14 \
      g++-14 \
      libfuse2 \
      libpcl-dev \
      libqt6opengl6-dev \
      libspdlog-dev \
      ninja-build \
      qt6-base-dev \
      qt6-base-dev-tools \
 && rm -rf /var/lib/apt/lists/*

RUN curl -L --fail "https://www.vtk.org/files/release/${VTK_SERIES}/VTK-${VTK_VERSION}.tar.gz" -o /tmp/vtk.tar.gz \
 && tar -xf /tmp/vtk.tar.gz -C /opt \
 && mv "/opt/VTK-${VTK_VERSION}" /opt/vtk-src \
 && cmake -S /opt/vtk-src -B /opt/vtk-build -GNinja \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/opt/vtk \
      -DCMAKE_C_COMPILER="${CC}" \
      -DCMAKE_CXX_COMPILER="${CXX}" \
      -DVTK_BUILD_SHARED_LIBS=ON \
      -DVTK_ENABLE_WRAPPING=OFF \
      -DVTK_QT_VERSION=6 \
      -DVTK_GROUP_ENABLE_Imaging=DONT_WANT \
      -DVTK_GROUP_ENABLE_MPI=DONT_WANT \
      -DVTK_GROUP_ENABLE_Qt=DONT_WANT \
      -DVTK_GROUP_ENABLE_Rendering=DONT_WANT \
      -DVTK_GROUP_ENABLE_StandAlone=DONT_WANT \
      -DVTK_GROUP_ENABLE_Views=DONT_WANT \
      -DVTK_MODULE_ENABLE_VTK_CommonColor=YES \
      -DVTK_MODULE_ENABLE_VTK_CommonCore=YES \
      -DVTK_MODULE_ENABLE_VTK_CommonDataModel=YES \
      -DVTK_MODULE_ENABLE_VTK_FiltersSources=YES \
      -DVTK_MODULE_ENABLE_VTK_GUISupportQt=YES \
      -DVTK_MODULE_ENABLE_VTK_InteractionStyle=YES \
      -DVTK_MODULE_ENABLE_VTK_RenderingAnnotation=YES \
      -DVTK_MODULE_ENABLE_VTK_RenderingContextOpenGL2=YES \
      -DVTK_MODULE_ENABLE_VTK_RenderingCore=YES \
      -DVTK_MODULE_ENABLE_VTK_RenderingFreeType=YES \
      -DVTK_MODULE_ENABLE_VTK_RenderingGL2PSOpenGL2=YES \
      -DVTK_MODULE_ENABLE_VTK_RenderingOpenGL2=YES \
 && cmake --build /opt/vtk-build --parallel "$(nproc)" \
 && cmake --install /opt/vtk-build \
 && ln -sfn . "/opt/vtk/include/vtk-${VTK_SERIES}/vtk" \
 && rm -rf /opt/vtk-src /opt/vtk-build /tmp/vtk.tar.gz

WORKDIR /workspace

CMD ["bash"]
