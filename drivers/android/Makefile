ccflags-y += -I$(src)			# needed for trace events

obj-$(CONFIG_ANDROID_BINDER_IPC)	+= binder.o binder_alloc.o
obj-$(CONFIG_ANDROID_BINDER_IPC_SELFTEST) += binder_alloc_selftest.o

#ifdef OPLUS_FEATURE_HANS_FREEZE
obj-$(CONFIG_OPPO_HANS) += hans.o
obj-$(CONFIG_OPPO_HANS) += hans_netfilter.o
#endif /*OPLUS_FEATURE_HANS_FREEZE*/
