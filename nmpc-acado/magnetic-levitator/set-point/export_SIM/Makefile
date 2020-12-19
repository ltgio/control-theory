#
#    This file was auto-generated using the ACADO Toolkit.
#    
#    While ACADO Toolkit is free software released under the terms of
#    the GNU Lesser General Public License (LGPL), the generated code
#    as such remains the property of the user who used ACADO Toolkit
#    to generate this code. In particular, user dependent data of the code
#    do not inherit the GNU LGPL license. On the other hand, parts of the
#    generated code that are a direct copy of source code from the
#    ACADO Toolkit or the software tools it is based on, remain, as derived
#    work, automatically covered by the LGPL license.
#    
#    ACADO Toolkit is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
#    


UNAME := $(shell uname)

LDLIBS = -lm -lstdc++
ifeq ($(UNAME), Linux)
	LDLIBS += -lrt
endif

CCACHE_APP := $(shell which ccache 2>/dev/null)

CFLAGS = -O3 -I.
CXXFLAGS = -O3 -finline-functions -I. 
CC     = $(CCACHE_APP) gcc
CXX    = $(CCACHE_APP) g++

OBJECTS = 	\
			acado_auxiliary_sim_functions.o \
			acado_integrator.o 

.PHONY: all
all: acado_test acado_compare 

acado_test: ${OBJECTS} acado_test.o

acado_compare: ${OBJECTS} acado_compare.o

acado_auxiliary_sim_functions.o          : 	acado_common.h \
											acado_auxiliary_sim_functions.h
acado_integrator.o          : 	acado_common.h \
								acado_auxiliary_sim_functions.h
acado_test.o                : 	acado_common.h \
								acado_auxiliary_sim_functions.h
acado_compare.o             : 	acado_common.h \
								acado_auxiliary_sim_functions.h

${OBJECTS} : acado_auxiliary_sim_functions.h

.PHONY : clean
clean :
	-rm -f *.o *.a acado_test acado_compare

