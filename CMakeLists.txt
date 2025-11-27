cmake_minimum_required(VERSION 3.16)
project(MyProject)

# C++20指定（全体に対して）
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# インクルードディレクトリを追加（oyl/utils.hpp を使うため）
include_directories(${PROJECT_SOURCE_DIR}/include)

# ライブラリ（src/**.cpp のビルド）
add_library(oyl-utils
 src/seo_class.cpp
 src/multi_seo_class.cpp
 src/memory_class.cpp
 src/oyl_video.cpp
)
target_include_directories(oyl-utils PUBLIC ${PROJECT_SOURCE_DIR}/include)

# main.cpp 実行ファイル
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
