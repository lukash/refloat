VESC_TOOL ?= vesc_tool

TARGET = package_lib

REFLOAT_SOURCES = $(wildcard *.c)
CONF_GEN_HEADERS = conf/conf_default.h conf/confparser.h conf/confxml.h
CONF_GEN_SOURCES = conf/confparser.c conf/confxml.c
CONF_GEN_FILES = $(CONF_GEN_HEADERS) $(CONF_GEN_SOURCES)
SOURCES = $(REFLOAT_SOURCES) $(CONF_GEN_SOURCES) conf/buffer.c
DEPS = $(SOURCES:.c=.d)

ADD_TO_CLEAN = $(CONF_GEN_FILES) $(DEPS)

VESC_C_LIB_PATH = ../vesc_pkg_lib/
include $(VESC_C_LIB_PATH)rules.mk

CFLAGS += -MMD

$(REFLOAT_SOURCES): $(CONF_GEN_HEADERS)

$(CONF_GEN_FILES) &: conf/settings.xml
	$(VESC_TOOL) --xmlConfToCode conf/settings.xml

-include $(DEPS)
