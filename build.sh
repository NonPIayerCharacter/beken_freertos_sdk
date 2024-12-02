SOC=$1
VERSION=$2
export OTA_TOOL="./tools/rtt_ota/rt_ota_packaging_tool_cli"

[ -z $SOC ] && SOC=bk7238
[ -z $VERSION ] && VERSION=1.0.0

make $SOC -j $(nproc) USER_SW_VER=$VERSION

$OTA_TOOL -f ./out/bsp.bin -o ./out/app.rbl -p app -c gzip -s aes -k 0123456789ABCDEF0123456789ABCDEF -i 0123456789ABCDEF -v $VERSION
