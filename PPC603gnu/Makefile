# Wind River Workbench generated Makefile.
# Do not edit!!!
#
# The file ".wrmakefile" is the template used by the Wind River Workbench to
# generate the makefiles of this project. Add user-specific build targets and
# make rules only(!) in this project's ".wrmakefile" file. These will then be
# automatically dumped into the makefiles.

WIND_HOME := $(subst \,/,$(WIND_HOME))
WIND_BASE := $(subst \,/,$(WIND_BASE))
WIND_USR := $(subst \,/,$(WIND_USR))

all : pre_build main_all post_build

_clean ::
	@echo "make: removing targets and objects of `pwd`"

TRACE=0
TRACEON=$(TRACE:0=@)
TRACE_FLAG=$(TRACEON:1=)

MAKEFILE := Makefile

FLEXIBLE_BUILD := 1

BUILD_SPEC = PPC603gnu
DEBUG_MODE = 1
ifeq ($(DEBUG_MODE),1)
MODE_DIR := Debug
else
MODE_DIR := NonDebug
endif
OBJ_DIR := .
WS_ROOT_DIR := C:/WindRiver/workspace
PRJ_ROOT_DIR := C:/Users/clemenj/Desktop/Beta\ Program/071212\ Windriver\ Backup/2012_Bot_Code/workspace/TMW2012



#Global Build Macros
PROJECT_TYPE = DKM
DEFINES = 
EXPAND_DBG = 0


#BuildSpec specific Build Macros
VX_CPU_FAMILY = ppc
CPU = PPC603
TOOL_FAMILY = gnu
TOOL = gnu
TOOL_PATH = 
CC_ARCH_SPEC = -mcpu=603 -mstrict-align -mno-implicit-fp -mlongcall
LIBPATH = 
LIBS = 

IDE_INCLUDES = -I$(WIND_BASE)/target/h/WPILib -I$(WIND_BASE)/target/h/wrn/coreip -I$(WIND_BASE)/target/h 

IDE_LIBRARIES = $(WIND_BASE)/target/lib/WPILib.a 

IDE_DEFINES = 



#BuildTool flags
ifeq ($(DEBUG_MODE),1)
DEBUGFLAGS_C-Compiler = -g
DEBUGFLAGS_C++-Compiler = -g
DEBUGFLAGS_Linker = -g
DEBUGFLAGS_Partial-Image-Linker = 
DEBUGFLAGS_Librarian = 
DEBUGFLAGS_Assembler = -g
else
DEBUGFLAGS_C-Compiler = -O2 -fstrength-reduce -fno-builtin
DEBUGFLAGS_C++-Compiler = -O2 -fstrength-reduce -fno-builtin
DEBUGFLAGS_Linker = -O2 -fstrength-reduce -fno-builtin
DEBUGFLAGS_Partial-Image-Linker = 
DEBUGFLAGS_Librarian = 
DEBUGFLAGS_Assembler = -O2 -fstrength-reduce -fno-builtin
endif


#Project Targets
PROJECT_TARGETS = TMW2012/$(MODE_DIR)/TMW2012.out \
	 TMW2012_partialImage/$(MODE_DIR)/TMW2012_partialImage.o


#Rules

# TMW2012
ifeq ($(DEBUG_MODE),1)
TMW2012/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -g
TMW2012/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -g
TMW2012/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -g
TMW2012/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
TMW2012/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
TMW2012/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -g
else
TMW2012/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -O2 -fstrength-reduce -fno-builtin
TMW2012/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -O2 -fstrength-reduce -fno-builtin
TMW2012/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -O2 -fstrength-reduce -fno-builtin
TMW2012/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
TMW2012/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
TMW2012/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -O2 -fstrength-reduce -fno-builtin
endif
TMW2012/$(MODE_DIR)/% : IDE_INCLUDES = -I$(WIND_BASE)/target/h/WPILib -I$(WIND_BASE)/target/h/wrn/coreip -I$(WIND_BASE)/target/h 
TMW2012/$(MODE_DIR)/% : IDE_LIBRARIES = $(WIND_BASE)/target/lib/WPILib.a 
TMW2012/$(MODE_DIR)/% : IDE_DEFINES = 
TMW2012/$(MODE_DIR)/% : PROJECT_TYPE = DKM
TMW2012/$(MODE_DIR)/% : DEFINES = 
TMW2012/$(MODE_DIR)/% : EXPAND_DBG = 0
TMW2012/$(MODE_DIR)/% : VX_CPU_FAMILY = ppc
TMW2012/$(MODE_DIR)/% : CPU = PPC603
TMW2012/$(MODE_DIR)/% : TOOL_FAMILY = gnu
TMW2012/$(MODE_DIR)/% : TOOL = gnu
TMW2012/$(MODE_DIR)/% : TOOL_PATH = 
TMW2012/$(MODE_DIR)/% : CC_ARCH_SPEC = -mcpu=603 -mstrict-align -mno-implicit-fp -mlongcall
TMW2012/$(MODE_DIR)/% : LIBPATH = 
TMW2012/$(MODE_DIR)/% : LIBS = 
TMW2012/$(MODE_DIR)/% : OBJ_DIR := TMW2012/$(MODE_DIR)

OBJECTS_TMW2012 = TMW2012_partialImage/$(MODE_DIR)/TMW2012_partialImage.o

TMW2012/$(MODE_DIR)/TMW2012.out : $(OBJECTS_TMW2012)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@";rm -f "$@";nmppc $(OBJECTS_TMW2012) $(IDE_LIBRARIES) | tclsh $(WIND_BASE)/host/resource/hutils/tcl/munch.tcl -c ppc > $(OBJ_DIR)/ctdt.c; $(TOOL_PATH)ccppc $(DEBUGFLAGS_Linker) $(CC_ARCH_SPEC) -fdollars-in-identifiers -Wall $(ADDED_CFLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES)  -o $(OBJ_DIR)/ctdt.o -c $(OBJ_DIR)/ctdt.c; $(TOOL_PATH)ccppc -r -nostdlib -Wl,-X -T $(WIND_BASE)/target/h/tool/gnu/ldscripts/link.OUT -o "$@" $(OBJ_DIR)/ctdt.o $(OBJECTS_TMW2012) $(IDE_LIBRARIES) $(LIBPATH) $(LIBS)  $(ADDED_LIBPATH) $(ADDED_LIBS) && if [ "$(EXPAND_DBG)" = "1" ]; then plink "$@";fi

TMW2012/$(MODE_DIR)/TMW2012_compile_file : $(FILE) ;

_clean :: TMW2012/$(MODE_DIR)/TMW2012_clean

TMW2012/$(MODE_DIR)/TMW2012_clean : 
	$(TRACE_FLAG)if [ -d "TMW2012" ]; then cd "TMW2012"; rm -rf $(MODE_DIR); fi


# TMW2012_partialImage
ifeq ($(DEBUG_MODE),1)
TMW2012_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -g
TMW2012_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -g
TMW2012_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -g
TMW2012_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
TMW2012_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
TMW2012_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -g
else
TMW2012_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -O2 -fstrength-reduce -fno-builtin
TMW2012_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -O2 -fstrength-reduce -fno-builtin
TMW2012_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -O2 -fstrength-reduce -fno-builtin
TMW2012_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
TMW2012_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
TMW2012_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -O2 -fstrength-reduce -fno-builtin
endif
TMW2012_partialImage/$(MODE_DIR)/% : IDE_INCLUDES = -I$(WIND_BASE)/target/h/WPILib -I$(WIND_BASE)/target/h/wrn/coreip -I$(WIND_BASE)/target/h 
TMW2012_partialImage/$(MODE_DIR)/% : IDE_LIBRARIES = $(WIND_BASE)/target/lib/WPILib.a 
TMW2012_partialImage/$(MODE_DIR)/% : IDE_DEFINES = 
TMW2012_partialImage/$(MODE_DIR)/% : PROJECT_TYPE = DKM
TMW2012_partialImage/$(MODE_DIR)/% : DEFINES = 
TMW2012_partialImage/$(MODE_DIR)/% : EXPAND_DBG = 0
TMW2012_partialImage/$(MODE_DIR)/% : VX_CPU_FAMILY = ppc
TMW2012_partialImage/$(MODE_DIR)/% : CPU = PPC603
TMW2012_partialImage/$(MODE_DIR)/% : TOOL_FAMILY = gnu
TMW2012_partialImage/$(MODE_DIR)/% : TOOL = gnu
TMW2012_partialImage/$(MODE_DIR)/% : TOOL_PATH = 
TMW2012_partialImage/$(MODE_DIR)/% : CC_ARCH_SPEC = -mcpu=603 -mstrict-align -mno-implicit-fp -mlongcall
TMW2012_partialImage/$(MODE_DIR)/% : LIBPATH = 
TMW2012_partialImage/$(MODE_DIR)/% : LIBS = 
TMW2012_partialImage/$(MODE_DIR)/% : OBJ_DIR := TMW2012_partialImage/$(MODE_DIR)

TMW2012_partialImage/$(MODE_DIR)/Objects/TMW2012/CameraAim.o : $(PRJ_ROOT_DIR)/CameraAim.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


TMW2012_partialImage/$(MODE_DIR)/Objects/TMW2012/GamePad.o : $(PRJ_ROOT_DIR)/GamePad.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


TMW2012_partialImage/$(MODE_DIR)/Objects/TMW2012/HOT_PID.o : $(PRJ_ROOT_DIR)/HOT_PID.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


TMW2012_partialImage/$(MODE_DIR)/Objects/TMW2012/Kinecto.o : $(PRJ_ROOT_DIR)/Kinecto.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


TMW2012_partialImage/$(MODE_DIR)/Objects/TMW2012/Pickup.o : $(PRJ_ROOT_DIR)/Pickup.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


TMW2012_partialImage/$(MODE_DIR)/Objects/TMW2012/RAWCConstants.o : $(PRJ_ROOT_DIR)/RAWCConstants.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


TMW2012_partialImage/$(MODE_DIR)/Objects/TMW2012/Shooter.o : $(PRJ_ROOT_DIR)/Shooter.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


TMW2012_partialImage/$(MODE_DIR)/Objects/TMW2012/Steer.o : $(PRJ_ROOT_DIR)/Steer.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


TMW2012_partialImage/$(MODE_DIR)/Objects/TMW2012/Swerve.o : $(PRJ_ROOT_DIR)/Swerve.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


TMW2012_partialImage/$(MODE_DIR)/Objects/TMW2012/TMW2012.o : $(PRJ_ROOT_DIR)/TMW2012.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


OBJECTS_TMW2012_partialImage = TMW2012_partialImage/$(MODE_DIR)/Objects/TMW2012/CameraAim.o \
	 TMW2012_partialImage/$(MODE_DIR)/Objects/TMW2012/GamePad.o \
	 TMW2012_partialImage/$(MODE_DIR)/Objects/TMW2012/HOT_PID.o \
	 TMW2012_partialImage/$(MODE_DIR)/Objects/TMW2012/Kinecto.o \
	 TMW2012_partialImage/$(MODE_DIR)/Objects/TMW2012/Pickup.o \
	 TMW2012_partialImage/$(MODE_DIR)/Objects/TMW2012/RAWCConstants.o \
	 TMW2012_partialImage/$(MODE_DIR)/Objects/TMW2012/Shooter.o \
	 TMW2012_partialImage/$(MODE_DIR)/Objects/TMW2012/Steer.o \
	 TMW2012_partialImage/$(MODE_DIR)/Objects/TMW2012/Swerve.o \
	 TMW2012_partialImage/$(MODE_DIR)/Objects/TMW2012/TMW2012.o

TMW2012_partialImage/$(MODE_DIR)/TMW2012_partialImage.o : $(OBJECTS_TMW2012_partialImage)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc -r -nostdlib -Wl,-X  -o "$@" $(OBJECTS_TMW2012_partialImage) $(LIBPATH) $(LIBS) $(ADDED_LIBPATH) $(ADDED_LIBS)  && if [ "$(EXPAND_DBG)" = "1" ]; then plink "$@";fi

TMW2012_partialImage/$(MODE_DIR)/TMW2012_partialImage_compile_file : $(FILE) ;

_clean :: TMW2012_partialImage/$(MODE_DIR)/TMW2012_partialImage_clean

TMW2012_partialImage/$(MODE_DIR)/TMW2012_partialImage_clean : 
	$(TRACE_FLAG)if [ -d "TMW2012_partialImage" ]; then cd "TMW2012_partialImage"; rm -rf $(MODE_DIR); fi

force : 

DEP_FILES := TMW2012_partialImage/$(MODE_DIR)/Objects/TMW2012/CameraAim.d TMW2012_partialImage/$(MODE_DIR)/Objects/TMW2012/GamePad.d TMW2012_partialImage/$(MODE_DIR)/Objects/TMW2012/HOT_PID.d \
	 TMW2012_partialImage/$(MODE_DIR)/Objects/TMW2012/Kinecto.d TMW2012_partialImage/$(MODE_DIR)/Objects/TMW2012/Pickup.d TMW2012_partialImage/$(MODE_DIR)/Objects/TMW2012/RAWCConstants.d \
	 TMW2012_partialImage/$(MODE_DIR)/Objects/TMW2012/Shooter.d TMW2012_partialImage/$(MODE_DIR)/Objects/TMW2012/Steer.d TMW2012_partialImage/$(MODE_DIR)/Objects/TMW2012/Swerve.d \
	 TMW2012_partialImage/$(MODE_DIR)/Objects/TMW2012/TMW2012.d
-include $(DEP_FILES)



-include $(PRJ_ROOT_DIR)/*.makefile

-include *.makefile

main_all : external_build $(PROJECT_TARGETS)
	@echo "make: built targets of `pwd`"

# entry point for extending the build
external_build ::
	@echo ""

# main entry point for pre processing prior to the build
pre_build :: $(PRE_BUILD_STEP) generate_sources
	@echo ""

# entry point for generating sources prior to the build
generate_sources ::
	@echo ""

# main entry point for post processing after the build
post_build :: $(POST_BUILD_STEP) deploy_output
	@echo ""

# entry point for deploying output after the build
deploy_output ::
	@echo ""

clean :: external_clean $(CLEAN_STEP) _clean

# entry point for extending the build clean
external_clean ::
	@echo ""
