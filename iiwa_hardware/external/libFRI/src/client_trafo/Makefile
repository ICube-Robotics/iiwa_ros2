BASE_DIR	= ../..
include $(BASE_DIR)/build/GNUMake/paths.mak
include $(BASE_DIR)/build/GNUMake/$(TOOLS_MAK)

CXX_SRC 		 = friTransformationClient.cpp

INC_DIR		+= $(CLIENTBASE_DIR) $(NANOPB_DIR) $(PROTOBUF_DIR) $(PROTOBUF_GEN_DIR)
CXXFLAGS 	+=
LDFLAGS 		+=

include $(BASE_DIR)/build/GNUMake/rules.mak
