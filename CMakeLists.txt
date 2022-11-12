cmake_minimum_required(VERSION 3.0.2)
project(testzk)

##find_package放在前面   target_link_libraries要放在 add_executable后边
#头文件目录
include_directories( 
 /usr/local/include/eigen3
# /usr/local/include/osqp
 /usr/local/include/matplotlibcpp
 /usr/include/python3.6
)

#库文件目录
link_directories(
  /usr/local/lib 
  /usr/lib/python3.6/config-3.6m-x86_64-linux-gnu
  )

# Find OSQP library and headers
#搜索qsqp的库文件和头文件
find_package(osqp REQUIRED)

#生成可执行文件
add_executable(testzk  osqp_eigen.cpp  )

target_link_libraries(testzk
                       #libosqp.so  
                       libOsqpEigen.so 
                       libqdldl.so 
                       libpython3.6.so
                       )
# Link the OSQP shared library
target_link_libraries(testzk osqp::osqp)
