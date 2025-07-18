BIN_DIR 		:= ${PWD}/bin
INSTALL_DIR := ${PWD}/install

KORTEX_API_INCLUDES := \
  -I$(INSTALL_DIR)/include/kortex_api \
  -I$(INSTALL_DIR)/include/kortex_api/client \
  -I$(INSTALL_DIR)/include/kortex_api/client_stubs \
  -I$(INSTALL_DIR)/include/kortex_api/common \
  -I$(INSTALL_DIR)/include/kortex_api/google \
  -I$(INSTALL_DIR)/include/kortex_api/messages

INCLUDE_DIRS 	:= -I$(INSTALL_DIR)/include/

EIGEN_FLAGS 	:= $(shell pkg-config --cflags --libs eigen3)
SPDLOG_FLAGS 	:= $(shell pkg-config --cflags --libs spdlog)

W_FLAGS 			:= -pedantic -Wall -Wextra

LIB_DIRS 			:= -L$(INSTALL_DIR)/lib
KDL_LIBS 			:= -lorocos-kdl -lkdl_parser
KORTEX_LIBS 	:= -lKortexApiCpp

RUNTIME_PATH 	:= -Wl,-rpath,$(INSTALL_DIR)/lib

KORTEX_API 		:= https://artifactory.kinovaapps.com/artifactory/generic-local-public/kortex/API/2.6.0/linux_x86_64_gcc_5.4.zip
KDL_PARSER 		:= https://github.com/secorolab/kdl_parser/archive/refs/tags/v1.0.0.zip

CXX        	  := g++
CXXFLAGS    	:= -std=c++17 -D_OS_UNIX -Wno-deprecated-declarations $(W_FLAGS) $(INCLUDE_DIRS) $(EIGEN_FLAGS) $(RUNTIME_PATH) $(LIB_DIRS)

KINOVA_DIR 		:= src/kinova/

setup:
	mkdir -p $(BIN_DIR) $(INSTALL_DIR)
	cp *.urdf bin/

# --------- libraries ----------------------

$(INSTALL_DIR)/lib/libkdl_parser.so:
	wget -O kdl_parser.zip $(KDL_PARSER)
	unzip -q kdl_parser.zip -d kdl_parser_tmp
	rm kdl_parser.zip
	mkdir -p kdl_parser
	mv kdl_parser_tmp/kdl_parser-*/* kdl_parser/
	rm -rf kdl_parser_tmp
	cd kdl_parser  && \
	mkdir -p build && cd build && \
	cmake -DCMAKE_INSTALL_PREFIX=$(INSTALL_DIR) .. && \
	make install

kdl_parser_setup: $(INSTALL_DIR)/lib/libkdl_parser.so
	@echo "kdl-parser present"

$(INSTALL_DIR)/lib/libKortexApiCpp.a:
	wget -O kortex_api.zip $(KORTEX_API)
	unzip kortex_api.zip -d kortex_api
	rm kortex_api.zip
	mkdir -p $(INSTALL_DIR)/include/kortex_api
	cp -r kortex_api/include/* $(INSTALL_DIR)/include/kortex_api/
	mkdir -p $(INSTALL_DIR)/lib
	cp kortex_api/lib/release/*.a $(INSTALL_DIR)/lib/
	rm -rf kortex_api/

kortex_setup: $(INSTALL_DIR)/lib/libKortexApiCpp.a
	@echo "Kortex API present"

# --------- executbles ----------------------

kdl_gc: $(KINOVA_DIR)/kdl_gc.cpp setup
	$(CXX) $(CXXFLAGS) $< $(KDL_LIBS) -o $(BIN_DIR)/$@

kinova_read: $(KINOVA_DIR)/kinova_read.cpp setup kortex_setup
	$(CXX) $(CXXFLAGS) $< ${KORTEX_API_INCLUDES} ${KDL_LIBS} $(KORTEX_LIBS) -o $(BIN_DIR)/$@

kinova_kdl_gc: $(KINOVA_DIR)/kinova_kdl_gc.cpp setup kortex_setup
	$(CXX) $(CXXFLAGS) $< ${KORTEX_API_INCLUDES} ${KDL_LIBS} $(KORTEX_LIBS) -o $(BIN_DIR)/$@

kinova_kdl_gc_ctrl: $(KINOVA_DIR)/kinova_kdl_gc_ctrl.cpp setup kortex_setup
	$(CXX) $(CXXFLAGS) $< ${KORTEX_API_INCLUDES} ${KDL_LIBS} $(KORTEX_LIBS) -o $(BIN_DIR)/$@
# --------- clean ----------------------------
clean:
	rm -rf $(BIN_DIR) $(INSTALL_DIR)
