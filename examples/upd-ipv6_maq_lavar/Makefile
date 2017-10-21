###CONTIKI_PROJECT = udp-server-atividade4
all: udp-server-atividade4

CONTIKI = ../..
CONTIKI_WITH_IPV6 = 1
CFLAGS += -DUIP_CONF_ND6_SEND_NS=1
CFLAGS += -g
DEBUG = 1
PROJECT_SOURCEFILES += protocol.c
include $(CONTIKI)/Makefile.include
