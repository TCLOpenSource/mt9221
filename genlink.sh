#!/bin/sh

if [ $# -eq 1 ] && [ -d $1 ]
then
	KERN_LOC=`realpath $1`
else
	KERN_LOC=`pwd`
fi

echo "$KERN_LOC"
if [ -e ${KERN_LOC}/mstar2 ]; then
	MST_DRVROOT=${KERN_LOC}
elif [ -e ${KERN_LOC}/../mstar2 ]; then
	MST_DRVROOT=${KERN_LOC}/..
else
	echo -e "err: mstar2 not found\n"
fi

# Create symlinks
ls ${MST_DRVROOT}/mstar2/hal/ | xargs -I {} ln -sfn ${MST_DRVROOT}/mstar2/hal/{}/cpu/arm ${KERN_LOC}/arch/arm/mach-{}
ln -sfn ${MST_DRVROOT}/mstar2/drv/cpu/arm64 ./arch/arm64/arm-boards
ln -sfn ${MST_DRVROOT}/mstar2 ./drivers/mstar2
ln -sfn ${MST_DRVROOT}/mstar2/Kconfig ./arch/mips/Kconfig_kdrv
ln -sfn ${MST_DRVROOT}/mstar3party ./drivers/mstar3party
true
