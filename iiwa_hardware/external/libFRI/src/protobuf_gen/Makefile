BASE_DIR	= ../..
include $(BASE_DIR)/build/GNUMake/paths.mak
include $(BASE_DIR)/build/GNUMake/$(TOOLS_MAK)

CC_SRC 		 = FRIMessages.pb.c

INC_DIR		+= $(NANOPB_DIR)
CFLAGS 		+=
LDFLAGS 		+=

################################################################################
### Include general makefile (at the end)
################################################################################

include $(BASE_DIR)/build/GNUMake/rules.mak
