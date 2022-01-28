# Create a library called "constraints_comp" which includes all the source file
# The extension is already found. Any number of sources could be listed here.
add_library (constraints_comp omega_computation.cpp)

# Make sure the compiler can find include files for our constraints_comp library
# when other libraries or executables link to Hello
target_include_directories (constraints_comp PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})