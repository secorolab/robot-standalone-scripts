EIGEN_FLAGS := $(shell pkg-config --cflags --libs eigen3)
PINOCCHIO_FLAGS := $(shell PKG_CONFIG_PATH=install/lib/pkgconfig pkg-config --cflags --libs pinocchio)
SPDLOG_FLAGS := $(shell pkg-config --cflags --libs spdlog)
DEP_LIBS := -lorocos-kdl install/lib/libkdl_parser.so install/lib/libKortexApiCpp.a
KORTEX_API_INCLUDES := -Iinstall/include/kortex_api -Iinstall/include/kortex_api/client -Iinstall/include/kortex_api/client_stubs -Iinstall/include/kortex_api/common -Iinstall/include/kortex_api/google -Iinstall/include/kortex_api/messages
W_FLAGS := -pedantic -Wall -Wextra

kdl_gc:
	g++ -std=c++17 kdl_gc.cpp -Iinstall/include/ $(SPDLOG_FLAGS) $(EIGEN_FLAGS) install/lib/libkdl_parser.so -lorocos-kdl -Wl,-rpath,${PWD}/install/lib -o bin/kdl_gc

kinova_kdl_gc:
	g++ -std=c++17 -D_OS_UNIX -Wno-deprecated-declarations $(W_FLAGS) kinova_kdl_gc.cpp -Iinstall/include/ ${KORTEX_API_INCLUDES} $(EIGEN_FLAGS) $(SPDLOG_FLAGS) ${DEP_LIBS} -Wl,-rpath,${PWD}/install/lib -o bin/kinova_kdl_gc

kinova_read:
	g++ -std=c++17 -D_OS_UNIX -Wno-deprecated-declarations kinova_read.cpp -Iinstall/include/ ${KORTEX_API_INCLUDES} $(EIGEN_FLAGS) ${DEP_LIBS} -Wl,-rpath,${PWD}/install/lib -o bin/kinova_read

eddie_kdl_gc:
	g++ -std=c++17 eddie_kdl_gc.cpp -Iinstall/include/ $(EIGEN_FLAGS) install/lib/libkdl_parser.so -lorocos-kdl -Wl,-rpath,${PWD}/install/lib -o bin/eddie_kdl_gc

pinocchio_gc:
	g++ -std=c++17 pinocchio_gc.cpp -Iinstall/include/ $(EIGEN_FLAGS) install/lib/libkdl_parser.so -lorocos-kdl -Wl,-rpath,${PWD}/install/lib ${PINOCCHIO_FLAGS} -o bin/pinocchio_gc
