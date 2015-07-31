obj-m += kcontrol_gpu_tegra.o

kcontrol_gpu_tegra-y	:= kcontrol_module_tegra.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
	$(CROSS_COMPILE)strip --strip-debug kcontrol_gpu_tegra.ko

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
	rm -f modules.order *~ *.o
