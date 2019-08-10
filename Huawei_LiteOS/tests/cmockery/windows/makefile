#
# Copyright 2008 Google Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
# http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Microsoft Windows namke file which builds the cmockery library and example 
# applications.
#
# To use this makefile...
# Select Start->Run and run "cmd.exe" to open the command line.
# Run "vsvars.bat" located in Microsoft Visual Studio's directory.
# Run "nmake" to build the cmockery library and example applications.
# Run "nmake clean" to delete all files built by this makefile.

# Target directories.
!IFNDEF EXECUTABLE_DIR
EXECUTABLE_DIR=.
!ENDIF # EXECUTABLE_DIR

!IFNDEF LIBRARY_DIR
LIBRARY_DIR=.
!ENDIF # LIBRARY_DIR

!IFNDEF OBJECT_DIR
OBJECT_DIR=.
!ENDIF # OBJECT_DIR

# Tools
!IFNDEF LIBTOOL
LIBTOOL=lib.exe
!ENDIF # LIBTOOL

!IFNDEF LINK
LINK=link.exe
!ENDIF # LINK

# Source directories.
!IFNDEF LIBRARY_SOURCE_DIRECTORY
LIBRARY_SOURCE_DIRECTORY=..\src
!ENDIF # LIBRARY_SOURCE_DIRECTORY
!IFNDEF EXAMPLE_SOURCE_DIRECTORY
EXAMPLE_SOURCE_DIRECTORY=..\src\example
!ENDIF # EXAMPLE_SOURCE_DIRECTORY

# Compiler flags and commands.
CC_COMMAND=$(CC) /nologo /c /D_WIN32_WINNT=0x501 /DHAVE_MALLOC_H \
	/I$(LIBRARY_SOURCE_DIRECTORY)\google $(CFLAGS) $(**) /Fo$(@)
CC_COMMAND_UNIT_TEST=$(CC_COMMAND) /DUNIT_TESTING=1 \
	/I$(EXAMPLE_SOURCE_DIRECTORY)

# If debugging is enabled compile with symbols and stop on exceptions.
!IFDEF UNIT_TESTING_DEBUG
CC_COMMAND=$(CC_COMMAND) /DUNIT_TESTING_DEBUG=1 /Zi /Od
LFLAGS=$(LFLAGS) /DEBUG
!ENDIF # UNIT_TESTING_DEBUG

LIBLINKFLAGS=/NOLOGO

# Library archiver flags and command.
LIB_COMMAND=$(LIBTOOL) $(LIBLINKFLAGS) $(LIBFLAGS) $(**) /OUT:$(@)

# Linker flags and command. 
LINK_COMMAND=$(LINK) $(LIBLINKFLAGS) /SUBSYSTEM:console \
	libcmt.lib kernel32.lib /NODEFAULTLIB:libc.lib \
	$(LFLAGS) $(**) /OUT:$(@)

.SUFFIXES: .exe .lib .obj .c

all: $(EXECUTABLE_DIR) $(LIBRARY_DIR) $(OBJECT_DIR) \
	$(LIBRARY_DIR)\cmockery.lib examples

$(EXECUTABLE_DIR):
	mkdir $@

!IF "$(LIBRARY_DIR)" != "$(EXECUTABLE_DIR)"
$(LIBRARY_DIR):
	mkdir $*
!ENDIF

!IF "$(OBJECT_DIR)" != "$(LIBRARY_DIR)" && \
	"$(OBJECT_DIR)" != "$(EXECUTABLE_DIR)"
$(OBJECT_DIR):
	mkdir $@
!ENDIF

examples: \
	$(EXECUTABLE_DIR)\calculator.exe \
	$(EXECUTABLE_DIR)\calculator_test.exe \
	$(EXECUTABLE_DIR)\allocate_module_test.exe \
	$(EXECUTABLE_DIR)\assert_macro_test.exe \
	$(EXECUTABLE_DIR)\customer_database_test.exe \
	$(EXECUTABLE_DIR)\key_value_test.exe \
	$(EXECUTABLE_DIR)\product_database_test.exe \
	$(EXECUTABLE_DIR)\run_tests.exe

clean:
	-cmd /c "@for %A in (\
		$(LIBRARY_DIR)\cmockery.lib \
			$(OBJECT_DIR)\cmockery.obj \
		$(EXECUTABLE_DIR)\calculator.exe \
			$(OBJECT_DIR)\calculator.obj \
		$(EXECUTABLE_DIR)\calculator_test.exe \
			$(OBJECT_DIR)\calculator_test.obj \
			$(OBJECT_DIR)\calculator_test-calculator.obj \
		$(EXECUTABLE_DIR)\allocate_module_test.exe \
			$(OBJECT_DIR)\allocate_module_test.obj \
			$(OBJECT_DIR)\allocate_module.obj \
		$(EXECUTABLE_DIR)\assert_macro_test.exe \
			$(OBJECT_DIR)\assert_macro_test.obj \
			$(OBJECT_DIR)\assert_macro.obj \
		$(EXECUTABLE_DIR)\customer_database_test.exe \
			$(OBJECT_DIR)\customer_database_test.obj \
			$(OBJECT_DIR)\customer_database.obj \
		$(EXECUTABLE_DIR)\key_value_test.exe \
			$(OBJECT_DIR)\key_value_test.obj \
			$(OBJECT_DIR)\key_value.obj \
		$(EXECUTABLE_DIR)\product_database_test.exe \
			$(OBJECT_DIR)\product_database_test.obj \
			$(OBJECT_DIR)\product_database.obj \
		$(EXECUTABLE_DIR)\run_tests.exe \
			$(OBJECT_DIR)\run_tests.obj) do @del %A 2>NUL"
	-rmdir $(EXECUTABLE_DIR) $(OBJECT_DIR) $(LIBRARY_DIR) 2>NUL

# Rules for the cmockery library.
$(LIBRARY_DIR)\cmockery.lib: $(OBJECT_DIR)\cmockery.obj
$(OBJECT_DIR)\cmockery.obj: $(LIBRARY_SOURCE_DIRECTORY)\cmockery.c

# Rules for the calculator application.
$(EXECUTABLE_DIR)\calculator.exe: $(OBJECT_DIR)\calculator.obj

$(OBJECT_DIR)\calculator.obj: $(EXAMPLE_SOURCE_DIRECTORY)\calculator.c
	$(CC_COMMAND)

# Rules for the calculator test application.
$(EXECUTABLE_DIR)\calculator_test.exe: \
		$(OBJECT_DIR)\calculator_test.obj \
		$(OBJECT_DIR)\calculator_test-calculator.obj \
		$(LIBRARY_DIR)\cmockery.lib
	$(LINK_COMMAND)

$(OBJECT_DIR)\calculator_test.obj: \
	$(EXAMPLE_SOURCE_DIRECTORY)\calculator_test.c

$(OBJECT_DIR)\calculator_test-calculator.obj: \
	$(EXAMPLE_SOURCE_DIRECTORY)\calculator.c
	$(CC_COMMAND_UNIT_TEST)

# Sample code applications.
$(EXECUTABLE_DIR)\allocate_module_test.exe: \
		$(OBJECT_DIR)\allocate_module_test.obj \
		$(OBJECT_DIR)\allocate_module.obj \
		$(LIBRARY_DIR)\cmockery.lib
	$(LINK_COMMAND)

$(OBJECT_DIR)\allocate_module_test.obj: \
	$(EXAMPLE_SOURCE_DIRECTORY)\allocate_module_test.c
$(OBJECT_DIR)\allocate_module.obj: \
	$(EXAMPLE_SOURCE_DIRECTORY)\allocate_module.c

$(EXECUTABLE_DIR)\assert_macro_test.exe: \
		$(OBJECT_DIR)\assert_macro_test.obj \
		$(OBJECT_DIR)\assert_macro.obj \
		$(LIBRARY_DIR)\cmockery.lib
	$(LINK_COMMAND)

$(OBJECT_DIR)\assert_macro_test.obj: \
	$(EXAMPLE_SOURCE_DIRECTORY)\assert_macro_test.c
$(OBJECT_DIR)\assert_macro.obj: $(EXAMPLE_SOURCE_DIRECTORY)\assert_macro.c

$(EXECUTABLE_DIR)\customer_database_test.exe: \
		$(OBJECT_DIR)\customer_database_test.obj \
		$(OBJECT_DIR)\customer_database.obj \
		$(LIBRARY_DIR)\cmockery.lib
	$(LINK_COMMAND)

$(OBJECT_DIR)\customer_database_test.obj: \
	$(EXAMPLE_SOURCE_DIRECTORY)\customer_database_test.c
$(OBJECT_DIR)\customer_database.obj: \
	$(EXAMPLE_SOURCE_DIRECTORY)\customer_database.c

$(EXECUTABLE_DIR)\key_value_test.exe: \
		$(OBJECT_DIR)\key_value_test.obj \
		$(OBJECT_DIR)\key_value.obj \
		$(LIBRARY_DIR)\cmockery.lib
	$(LINK_COMMAND)

$(OBJECT_DIR)\key_value_test.obj: $(EXAMPLE_SOURCE_DIRECTORY)\key_value_test.c
$(OBJECT_DIR)\key_value.obj: $(EXAMPLE_SOURCE_DIRECTORY)\key_value.c

$(EXECUTABLE_DIR)\product_database_test.exe: \
		$(OBJECT_DIR)\product_database_test.obj \
		$(OBJECT_DIR)\product_database.obj \
		$(LIBRARY_DIR)\cmockery.lib
	$(LINK_COMMAND)

$(OBJECT_DIR)\product_database_test.obj: \
	$(EXAMPLE_SOURCE_DIRECTORY)\product_database_test.c
$(OBJECT_DIR)\product_database.obj: \
	$(EXAMPLE_SOURCE_DIRECTORY)\product_database.c

$(EXECUTABLE_DIR)\run_tests.exe: \
		$(OBJECT_DIR)\run_tests.obj $(LIBRARY_DIR)\cmockery.lib
	$(LINK_COMMAND)

$(OBJECT_DIR)\run_tests.obj: $(EXAMPLE_SOURCE_DIRECTORY)\run_tests.c

# Inference rules.
{$(OBJECT_DIR)\}.obj{$(EXECUTABLE_DIR)\}.exe:
	$(LINK_COMMAND)

{$(OBJECT_DIR)\}.obj{$(LIBRARY_DIR)\}.lib:
	$(LIB_COMMAND)

{$(LIBRARY_SOURCE_DIRECTORY)\}.c{$(OBJECT_DIR)\}.obj:
	$(CC_COMMAND)

{$(EXAMPLE_SOURCE_DIRECTORY)\}.c{$(OBJECT_DIR)\}.obj:
	$(CC_COMMAND_UNIT_TEST)
