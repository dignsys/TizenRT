# Assumed to run from tinyara/os/ directory
# TODO:
#   1. Get the code from the git repo
#   2. Patch the code using the additional changes
#   3. Build the code with the necessary flags

set -x

extract_flags() {
	if [ -e ${TOPDIR}/include/tinyara/config.h ]; then
		echo `cat ${TOPDIR}/include/tinyara/config.h | grep -w "$1"  | cut -d " " -f 3`
	fi
}

export IOTIVITY_BASE="${TOPDIR}/../external/iotivity-constrained"
CONFIG_IOTIVITY_RELEASE_VERSION=`extract_flags "CONFIG_ENABLE_IOTIVITY_CONSTRAINED"`
IOTIVITY_RELEASE_VERSION=$(echo "$CONFIG_IOTIVITY_RELEASE_VERSION" | sed 's/"//g')
export TINYARA_BUILD_DIR="${TOPDIR}"
# export IOTIVITY_BUILD_DIR="${IOTIVITY_BASE}/iotivity_${IOTIVITY_RELEASE_VERSION}"
export IOTIVITY_BUILD_DIR="${IOTIVITY_BASE}/port/tizenrt"
# export IOTIVITY_PATCH_DIR="${IOTIVITY_BASE}/patches/${IOTIVITY_RELEASE_VERSION}"
export IOTIVITY_PATCH_DIR="${IOTIVITY_BASE}/patches"

echo "iotivity contrained BUILD DIR : ${IOTIVITY_BUILD_DIR}"

OPTIONS="DYNAMIC=1"

CONFIG_ENABLE_IOTIVITY_CONSTRAINED=`extract_flags "CONFIG_ENABLE_IOTIVITY_CONSTRAINED"`
if [ -z ${CONFIG_ENABLE_IOTIVITY_CONSTRAINED} ]; then CONFIG_ENABLE_IOTIVITY_CONSTRAINED=0; fi
CONFIG_IOTIVITY_CONSTRAINED_DEBUG=`extract_flags "CONFIG_IOTIVITY_CONSTRAINED_DEBUG"`
if [ ! -z ${CONFIG_IOTIVITY_CONSTRAINED_DEBUG} ]; then OPTIONS="${OPTIONS} DEBUG=1"; fi
CONFIG_IOTIVITY_CONSTRAINED_SECURED=`extract_flags "CONFIG_IOTIVITY_CONSTRAINED_SECURED"`
if [ -z ${CONFIG_IOTIVITY_CONSTRAINED_SECURED} ]; then OPTIONS="${OPTIONS} SECURE=0"; fi
CONFIG_IOTIVITY_CONTRAINED_TCP=`extract_flags "CONFIG_IOTIVITY_CONTRAINED_TCP"`
if [ ! -z ${CONFIG_IOTIVITY_CONTRAINED_TCP} ]; then OPTIONS="${OPTIONS} TCP=1"; fi
CONFIG_IOTIVITY_CONTRAINED_IPV4=`extract_flags "CONFIG_IOTIVITY_CONTRAINED_IPV4"`
if [ ! -z ${CONFIG_IOTIVITY_CONTRAINED_IPV4} ]; then OPTIONS="${OPTIONS} IPV4=1"; fi

if [ ${CONFIG_ENABLE_IOTIVITY_CONSTRAINED} -eq 1 ]; then
	cd ${IOTIVITY_BUILD_DIR}

	echo "Cleaning Iotivity Constrained Old build with options = ${OPTIONS}"

	make clean

	cd ${TINYARA_BUILD_DIR}
fi
