TOPDIR=$1
rm ${TOPDIR}/build/output/bin/romfs.img
sh ${TOPDIR}/tools/fs/mkromfsimg.sh
cd ${TOPDIR}/build/output/bin
xxd -i romfs.img > stm32_romfs_img.c
cp stm32_romfs_img.c ../../../os/board/stm32l4xr/src
echo "stm32_romfs_img.c was made"

