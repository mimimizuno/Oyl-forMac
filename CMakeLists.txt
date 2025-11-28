cmake_minimum_required(VERSION 3.16)
project(MyProject)

# C++20指定（全体に対して）
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ======== ここから追加 ========
# mac だけ OpenCV_DIR を決め打ち
if(APPLE)
    set(OpenCV_DIR
        "/opt/homebrew/Cellar/opencv/4.12.0_15/lib/cmake/opencv4"
        CACHE PATH "Path to OpenCVConfig.cmake" FORCE
    )
endif()
# ======== ここまで追加 ========

# OpenCV
find_package(OpenCV REQUIRED CONFIG)

# OpenMP
find_package(OpenMP REQUIRED)

# ソースファイルとインクルード
set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
file(GLOB OYL_SOURCES "${PROJECT_SOURCE_DIR}/src/*.cpp")

# ライブラリ構築
add_library(oyl-utils ${OYL_SOURCES})
target_include_directories(oyl-utils
    PUBLIC
        ${PROJECT_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
)
target_compile_features(oyl-utils PUBLIC cxx_std_20)
target_link_libraries(oyl-utils PUBLIC OpenMP::OpenMP_CXX)

# メイン実行ファイル
# add_executable(MainApp data/synchronized-2particle-loop.cpp)
add_executable(MainApp main.cpp)
# サンプルを実行したい場合は下のように記述。↑のmain.cppはコメントアウト
# add_executable(MainApp sample/parameter_calc.cpp)
target_link_libraries(MainApp PRIVATE oyl-utils ${OpenCV_LIBS})

# テストオプション
option(BUILD_TESTING "Build unit tests" ON)
if (BUILD_TESTING)
    enable_testing()
    find_package(GTest REQUIRED)
    include_directories(${GTEST_INCLUDE_DIRS})

    add_executable(UnitTests
        test/test_seo_class.cpp
        test/grid_2dim_seo_test.cpp
        test/test_simulation2d_output.cpp
    )

    target_link_libraries(UnitTests
        PRIVATE
        GTest::GTest
        GTest::Main
        oyl-utils
        ${OpenCV_LIBS}
    )

    add_test(NAME AllTests COMMAND UnitTests)
endif()