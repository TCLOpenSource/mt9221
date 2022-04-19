# 0. download an generic arm architecture toolchains

# 1. build kernel

## generate configs
cp .config_mt5867_SMP_arm_android_emmc_nand_utopia2k_iommu .config
make menuconfig
## compile with prebuilts toolchains
make -jN ARCH=arm CROSS_COMPILE=${CROSS_COMPILE}
