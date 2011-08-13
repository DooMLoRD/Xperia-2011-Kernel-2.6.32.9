#
# Makefile for the FUSE filesystem.
#

CFLAGS_REMOVE_dev.o = -Werror
obj-$(CONFIG_FUSE_FS) += fuse.o
obj-$(CONFIG_CUSE) += cuse.o

fuse-objs := dev.o dir.o file.o inode.o control.o
