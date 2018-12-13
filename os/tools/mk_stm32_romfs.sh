sh ../tools/fs/mkromfsimg.sh
cd ../build/output/bin
xxd -i romfs.img > stm32_romfs_img.c
cp stm32_romfs_img.c ../../../os/arch/arm/src/stm32l4xr/src
echo "stm32_romfs_img.c was made"

