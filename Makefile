#############################################################################
# Makefile for building: hplanning
# Generated by qmake (2.01a) (Qt 4.8.6) on: Wed Apr 13 23:22:35 2016
# Project:  hplanning.pro
# Template: app
# Command: /usr/bin/qmake-qt4 -spec /usr/share/qt4/mkspecs/linux-g++-64 -o Makefile hplanning.pro
#############################################################################

####### Compiler, tools and options

CC            = gcc
CXX           = g++
DEFINES       = 
CFLAGS        = -m64 -pipe -O2 -Wall -W $(DEFINES)
CXXFLAGS      = -m64 -pipe -std=c++11 -Wall -Wextra -DNDEBUG -O2 -Wall -W $(DEFINES)
INCPATH       = -I/usr/share/qt4/mkspecs/linux-g++-64 -I. -I. -Isrc
LINK          = g++
LFLAGS        = -m64 -Wl,-O1
LIBS          = $(SUBLIBS)   -lboost_program_options 
AR            = ar cqs
RANLIB        = 
QMAKE         = /usr/bin/qmake-qt4
TAR           = tar -cf
COMPRESS      = gzip -9f
COPY          = cp -f
SED           = sed
COPY_FILE     = $(COPY)
COPY_DIR      = $(COPY) -r
STRIP         = strip
INSTALL_FILE  = install -m 644 -p
INSTALL_DIR   = $(COPY_DIR)
INSTALL_PROGRAM = install -m 755 -p
DEL_FILE      = rm -f
SYMLINK       = ln -f -s
DEL_DIR       = rmdir
MOVE          = mv -f
CHK_DIR_EXISTS= test -d
MKDIR         = mkdir -p

####### Output directory

OBJECTS_DIR   = ./

####### Files

SOURCES       = src/battleship.cpp \
		src/beliefstate.cpp \
		src/coord.cpp \
		src/experiment.cpp \
		src/main.cpp \
		src/flatmcts.cpp \
		src/network.cpp \
		src/node.cpp \
		src/pocman.cpp \
		src/rocksample.cpp \
		src/simulator.cpp \
		src/tag.cpp \
		src/testsimulator.cpp \
		src/utils.cpp \
		src/distribution.cpp \
		src/rooms.cpp \
		src/hierarchicalmcts.cpp \
		src/redundantobject.cpp \
		src/mcts.cpp \
		src/continousrooms.cpp 
OBJECTS       = battleship.o \
		beliefstate.o \
		coord.o \
		experiment.o \
		main.o \
		flatmcts.o \
		network.o \
		node.o \
		pocman.o \
		rocksample.o \
		simulator.o \
		tag.o \
		testsimulator.o \
		utils.o \
		distribution.o \
		rooms.o \
		hierarchicalmcts.o \
		redundantobject.o \
		mcts.o \
		continousrooms.o
DIST          = /usr/share/qt4/mkspecs/common/unix.conf \
		/usr/share/qt4/mkspecs/common/linux.conf \
		/usr/share/qt4/mkspecs/common/gcc-base.conf \
		/usr/share/qt4/mkspecs/common/gcc-base-unix.conf \
		/usr/share/qt4/mkspecs/common/g++-base.conf \
		/usr/share/qt4/mkspecs/common/g++-unix.conf \
		/usr/share/qt4/mkspecs/qconfig.pri \
		/usr/share/qt4/mkspecs/features/qt_functions.prf \
		/usr/share/qt4/mkspecs/features/qt_config.prf \
		/usr/share/qt4/mkspecs/features/exclusive_builds.prf \
		/usr/share/qt4/mkspecs/features/default_pre.prf \
		/usr/share/qt4/mkspecs/features/release.prf \
		/usr/share/qt4/mkspecs/features/default_post.prf \
		/usr/share/qt4/mkspecs/features/shared.prf \
		/usr/share/qt4/mkspecs/features/unix/gdb_dwarf_index.prf \
		/usr/share/qt4/mkspecs/features/warn_on.prf \
		/usr/share/qt4/mkspecs/features/resources.prf \
		/usr/share/qt4/mkspecs/features/uic.prf \
		/usr/share/qt4/mkspecs/features/yacc.prf \
		/usr/share/qt4/mkspecs/features/lex.prf \
		/usr/share/qt4/mkspecs/features/include_source_dir.prf \
		hplanning.pro
QMAKE_TARGET  = hplanning
DESTDIR       = 
TARGET        = hplanning

first: all
####### Implicit rules

.SUFFIXES: .o .c .cpp .cc .cxx .C

.cpp.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.cc.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.cxx.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.C.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.c.o:
	$(CC) -c $(CFLAGS) $(INCPATH) -o "$@" "$<"

####### Build rules

all: Makefile $(TARGET)

$(TARGET):  $(OBJECTS)  
	$(LINK) $(LFLAGS) -o $(TARGET) $(OBJECTS) $(OBJCOMP) $(LIBS)

Makefile: hplanning.pro  /usr/share/qt4/mkspecs/linux-g++-64/qmake.conf /usr/share/qt4/mkspecs/common/unix.conf \
		/usr/share/qt4/mkspecs/common/linux.conf \
		/usr/share/qt4/mkspecs/common/gcc-base.conf \
		/usr/share/qt4/mkspecs/common/gcc-base-unix.conf \
		/usr/share/qt4/mkspecs/common/g++-base.conf \
		/usr/share/qt4/mkspecs/common/g++-unix.conf \
		/usr/share/qt4/mkspecs/qconfig.pri \
		/usr/share/qt4/mkspecs/features/qt_functions.prf \
		/usr/share/qt4/mkspecs/features/qt_config.prf \
		/usr/share/qt4/mkspecs/features/exclusive_builds.prf \
		/usr/share/qt4/mkspecs/features/default_pre.prf \
		/usr/share/qt4/mkspecs/features/release.prf \
		/usr/share/qt4/mkspecs/features/default_post.prf \
		/usr/share/qt4/mkspecs/features/shared.prf \
		/usr/share/qt4/mkspecs/features/unix/gdb_dwarf_index.prf \
		/usr/share/qt4/mkspecs/features/warn_on.prf \
		/usr/share/qt4/mkspecs/features/resources.prf \
		/usr/share/qt4/mkspecs/features/uic.prf \
		/usr/share/qt4/mkspecs/features/yacc.prf \
		/usr/share/qt4/mkspecs/features/lex.prf \
		/usr/share/qt4/mkspecs/features/include_source_dir.prf
	$(QMAKE) -spec /usr/share/qt4/mkspecs/linux-g++-64 -o Makefile hplanning.pro
/usr/share/qt4/mkspecs/common/unix.conf:
/usr/share/qt4/mkspecs/common/linux.conf:
/usr/share/qt4/mkspecs/common/gcc-base.conf:
/usr/share/qt4/mkspecs/common/gcc-base-unix.conf:
/usr/share/qt4/mkspecs/common/g++-base.conf:
/usr/share/qt4/mkspecs/common/g++-unix.conf:
/usr/share/qt4/mkspecs/qconfig.pri:
/usr/share/qt4/mkspecs/features/qt_functions.prf:
/usr/share/qt4/mkspecs/features/qt_config.prf:
/usr/share/qt4/mkspecs/features/exclusive_builds.prf:
/usr/share/qt4/mkspecs/features/default_pre.prf:
/usr/share/qt4/mkspecs/features/release.prf:
/usr/share/qt4/mkspecs/features/default_post.prf:
/usr/share/qt4/mkspecs/features/shared.prf:
/usr/share/qt4/mkspecs/features/unix/gdb_dwarf_index.prf:
/usr/share/qt4/mkspecs/features/warn_on.prf:
/usr/share/qt4/mkspecs/features/resources.prf:
/usr/share/qt4/mkspecs/features/uic.prf:
/usr/share/qt4/mkspecs/features/yacc.prf:
/usr/share/qt4/mkspecs/features/lex.prf:
/usr/share/qt4/mkspecs/features/include_source_dir.prf:
qmake:  FORCE
	@$(QMAKE) -spec /usr/share/qt4/mkspecs/linux-g++-64 -o Makefile hplanning.pro

dist: 
	@$(CHK_DIR_EXISTS) .tmp/hplanning1.0.0 || $(MKDIR) .tmp/hplanning1.0.0 
	$(COPY_FILE) --parents $(SOURCES) $(DIST) .tmp/hplanning1.0.0/ && (cd `dirname .tmp/hplanning1.0.0` && $(TAR) hplanning1.0.0.tar hplanning1.0.0 && $(COMPRESS) hplanning1.0.0.tar) && $(MOVE) `dirname .tmp/hplanning1.0.0`/hplanning1.0.0.tar.gz . && $(DEL_FILE) -r .tmp/hplanning1.0.0


clean:compiler_clean 
	-$(DEL_FILE) $(OBJECTS)
	-$(DEL_FILE) *~ core *.core


####### Sub-libraries

distclean: clean
	-$(DEL_FILE) $(TARGET) 
	-$(DEL_FILE) Makefile


check: first

compiler_rcc_make_all:
compiler_rcc_clean:
compiler_uic_make_all:
compiler_uic_clean:
compiler_image_collection_make_all: qmake_image_collection.cpp
compiler_image_collection_clean:
	-$(DEL_FILE) qmake_image_collection.cpp
compiler_yacc_decl_make_all:
compiler_yacc_decl_clean:
compiler_yacc_impl_make_all:
compiler_yacc_impl_clean:
compiler_lex_make_all:
compiler_lex_clean:
compiler_clean: 

####### Compile

battleship.o: src/battleship.cpp src/battleship.h \
		src/simulator.h \
		src/history.h \
		src/utils.h \
		src/memorypool.h \
		src/prettyprint.h \
		src/node.h \
		src/beliefstate.h \
		src/statistic.h \
		src/distribution.h \
		src/grid.h \
		src/coord.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o battleship.o src/battleship.cpp

beliefstate.o: src/beliefstate.cpp src/beliefstate.h \
		src/statistic.h \
		src/distribution.h \
		src/utils.h \
		src/memorypool.h \
		src/prettyprint.h \
		src/simulator.h \
		src/history.h \
		src/node.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o beliefstate.o src/beliefstate.cpp

coord.o: src/coord.cpp src/coord.h \
		src/utils.h \
		src/memorypool.h \
		src/prettyprint.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o coord.o src/coord.cpp

experiment.o: src/experiment.cpp src/experiment.h \
		src/mcts.h \
		src/simulator.h \
		src/history.h \
		src/utils.h \
		src/memorypool.h \
		src/prettyprint.h \
		src/node.h \
		src/beliefstate.h \
		src/statistic.h \
		src/distribution.h \
		src/dot_graph.h \
		src/hierarchicalmcts.h \
		src/flatmcts.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o experiment.o src/experiment.cpp

main.o: src/main.cpp src/battleship.h \
		src/simulator.h \
		src/history.h \
		src/utils.h \
		src/memorypool.h \
		src/prettyprint.h \
		src/node.h \
		src/beliefstate.h \
		src/statistic.h \
		src/distribution.h \
		src/grid.h \
		src/coord.h \
		src/flatmcts.h \
		src/dot_graph.h \
		src/mcts.h \
		src/hierarchicalmcts.h \
		src/network.h \
		src/pocman.h \
		src/rocksample.h \
		src/rooms.h \
		src/continousrooms.h \
		src/tag.h \
		src/experiment.h \
		src/redundantobject.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o main.o src/main.cpp

flatmcts.o: src/flatmcts.cpp src/flatmcts.h \
		src/simulator.h \
		src/history.h \
		src/utils.h \
		src/memorypool.h \
		src/prettyprint.h \
		src/node.h \
		src/beliefstate.h \
		src/statistic.h \
		src/distribution.h \
		src/dot_graph.h \
		src/mcts.h \
		src/testsimulator.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o flatmcts.o src/flatmcts.cpp

network.o: src/network.cpp src/network.h \
		src/simulator.h \
		src/history.h \
		src/utils.h \
		src/memorypool.h \
		src/prettyprint.h \
		src/node.h \
		src/beliefstate.h \
		src/statistic.h \
		src/distribution.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o network.o src/network.cpp

node.o: src/node.cpp src/node.h \
		src/beliefstate.h \
		src/statistic.h \
		src/distribution.h \
		src/utils.h \
		src/memorypool.h \
		src/prettyprint.h \
		src/history.h \
		src/simulator.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o node.o src/node.cpp

pocman.o: src/pocman.cpp src/pocman.h \
		src/simulator.h \
		src/history.h \
		src/utils.h \
		src/memorypool.h \
		src/prettyprint.h \
		src/node.h \
		src/beliefstate.h \
		src/statistic.h \
		src/distribution.h \
		src/coord.h \
		src/grid.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o pocman.o src/pocman.cpp

rocksample.o: src/rocksample.cpp src/rocksample.h \
		src/simulator.h \
		src/history.h \
		src/utils.h \
		src/memorypool.h \
		src/prettyprint.h \
		src/node.h \
		src/beliefstate.h \
		src/statistic.h \
		src/distribution.h \
		src/coord.h \
		src/grid.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o rocksample.o src/rocksample.cpp

simulator.o: src/simulator.cpp src/simulator.h \
		src/history.h \
		src/utils.h \
		src/memorypool.h \
		src/prettyprint.h \
		src/node.h \
		src/beliefstate.h \
		src/statistic.h \
		src/distribution.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o simulator.o src/simulator.cpp

tag.o: src/tag.cpp src/tag.h \
		src/simulator.h \
		src/history.h \
		src/utils.h \
		src/memorypool.h \
		src/prettyprint.h \
		src/node.h \
		src/beliefstate.h \
		src/statistic.h \
		src/distribution.h \
		src/coord.h \
		src/grid.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o tag.o src/tag.cpp

testsimulator.o: src/testsimulator.cpp src/testsimulator.h \
		src/simulator.h \
		src/history.h \
		src/utils.h \
		src/memorypool.h \
		src/prettyprint.h \
		src/node.h \
		src/beliefstate.h \
		src/statistic.h \
		src/distribution.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o testsimulator.o src/testsimulator.cpp

utils.o: src/utils.cpp src/utils.h \
		src/memorypool.h \
		src/prettyprint.h \
		src/distribution.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o utils.o src/utils.cpp

distribution.o: src/distribution.cpp src/distribution.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o distribution.o src/distribution.cpp

rooms.o: src/rooms.cpp src/rooms.h \
		src/simulator.h \
		src/history.h \
		src/utils.h \
		src/memorypool.h \
		src/prettyprint.h \
		src/node.h \
		src/beliefstate.h \
		src/statistic.h \
		src/distribution.h \
		src/coord.h \
		src/grid.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o rooms.o src/rooms.cpp

hierarchicalmcts.o: src/hierarchicalmcts.cpp src/hierarchicalmcts.h \
		src/mcts.h \
		src/simulator.h \
		src/history.h \
		src/utils.h \
		src/memorypool.h \
		src/prettyprint.h \
		src/node.h \
		src/beliefstate.h \
		src/statistic.h \
		src/distribution.h \
		src/dot_graph.h \
		src/coord.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o hierarchicalmcts.o src/hierarchicalmcts.cpp

redundantobject.o: src/redundantobject.cpp src/redundantobject.h \
		src/simulator.h \
		src/history.h \
		src/utils.h \
		src/memorypool.h \
		src/prettyprint.h \
		src/node.h \
		src/beliefstate.h \
		src/statistic.h \
		src/distribution.h \
		src/coord.h \
		src/grid.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o redundantobject.o src/redundantobject.cpp

mcts.o: src/mcts.cpp src/mcts.h \
		src/simulator.h \
		src/history.h \
		src/utils.h \
		src/memorypool.h \
		src/prettyprint.h \
		src/node.h \
		src/beliefstate.h \
		src/statistic.h \
		src/distribution.h \
		src/dot_graph.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o mcts.o src/mcts.cpp

continousrooms.o: src/continousrooms.cpp src/continousrooms.h \
		src/simulator.h \
		src/history.h \
		src/utils.h \
		src/memorypool.h \
		src/prettyprint.h \
		src/node.h \
		src/beliefstate.h \
		src/statistic.h \
		src/distribution.h \
		src/coord.h \
		src/grid.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o continousrooms.o src/continousrooms.cpp

####### Install

install:   FORCE

uninstall:   FORCE

FORCE:

