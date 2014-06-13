android_kernel_Google_glass_1
=============================

The kernel sources for Google Glass. Each branch corresponds to a Glass release. For example, the kernel source for XE17.3 is "glass-omap-xrv60b"

Checkout the desired branch into {CM.ROOT}/kernel/Google/glass_1 and build the recovery image with the following in your BoardConfig.mk:

```
TARGET_KERNEL_SOURCE := kernel/Google/glass_1
TARGET_KERNEL_CONFIG := notle_defconfig
```

| Kernel          | Glass Version   |
| :-------------: | :-------------: |
| XRV60B          | XE17.3          |
| XRV67           | XE17.31         |
| XRV70D          | XE18.1          |
