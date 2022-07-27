BASE_DIR	= ../..
include $(BASE_DIR)/build/GNUMake/paths.mak
include $(BASE_DIR)/build/GNUMake/$(TOOLS_MAK)

CC_SRC 		 = pb_frimessages_callbacks.c

CXX_SRC		 = friMonitoringMessageDecoder.cpp \
					friCommandMessageEncoder.cpp

INC_DIR		+= $(NANOPB_DIR) $(PROTOBUF_GEN_DIR)
CFLAGS 		+=
CXXFLAGS		+=
LDFLAGS 		+=

################################################################################
### Include general makefile (at the end)
################################################################################

include $(BASE_DIR)/build/GNUMake/rules.mak
