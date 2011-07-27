#
# Makefile for the Linux Ethernet layer.
#


obj-y					+= eth.o

CFLAGS_REMOVE_eth.o = -Werror

obj-$(subst m,y,$(CONFIG_IPX))		+= pe2.o
obj-$(subst m,y,$(CONFIG_ATALK))	+= pe2.o
