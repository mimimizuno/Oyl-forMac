#!/usr/bin/env bash

set -e  # どこかでエラーが出たら即終了

# 出力ディレクトリ
mkdir -p output
rm -f output/*

BUILD_DIR=build
EXECUTABLE=MainApp   # mac では .exe なし
TESTING=OFF
JOBS=8

# --- OpenCV のプレフィックスを Homebrew から取得 ---
OPENCV_PREFIX=$(brew --prefix opencv)
OPENCV_DIR="${OPENCV_PREFIX}/lib/cmake/opencv4"

echo "Using OpenCV_DIR=${OPENCV_DIR}"

# 初回CMake構成
if [ ! -f ${BUILD_DIR}/build.ninja ] && [ ! -f ${BUILD_DIR}/Makefile ]; then
  # Ninja を使う場合（使わないなら -G Ninja を消せばOK）
  cmake -S . -B ${BUILD_DIR} -G Ninja \
    -DCMAKE_C_COMPILER=/opt/homebrew/opt/llvm/bin/clang \
    -DCMAKE_CXX_COMPILER=/opt/homebrew/opt/llvm/bin/clang++ \
    -DOpenCV_DIR="${OPENCV_DIR}" \
    -DBUILD_TESTING=${TESTING}
fi

# ビルド
cmake --build ${BUILD_DIR} --parallel ${JOBS}

# 実行
./${BUILD_DIR}/${EXECUTABLE}