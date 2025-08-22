#!/bin/bash

#The following two parameters can be modified according to the actual situation.
#mod_name: refers to the name of the dynamically loaded file.
#file_size: refers to the size of the filesystem you will generate.
mod_name="webclient_mod"
file_size=0x100000

echo "================ start $mod_name ====================="

current_dir=$(basename $(pwd))
echo "Current dir is: $current_dir"

#Obtain the relevant paths.
armino_path=$1
build_path=$2
lib_path=$3
source_path=$4
toolchain_path=$5
module_keep_api_path=$6

echo "source_path: $source_path"
echo "armino_path: $armino_path"
echo "build_path: $build_path"
echo "lib_path: $lib_path"
echo "toolchain_path: $toolchain_path"
echo "module_keep_api_path: $module_keep_api_path"

#Generate a dynamically loaded image based on the input file
mod_build_tool=$armino_path/components/udynlink/scripts/mkmodule
mod_build_opt="--no-prologue --build-c-module --gen-c-header --header-path ${source_path} --armino-path $armino_path --build-path $build_path --toolchain-path $toolchain_path --module-keep-api-path $module_keep_api_path --disasm"


# export_api_list = [
#     "webclient_session_create",
#     "bk_webclient_get",
#     "bk_webclient_post",
#     "webclient_get",
#     "webclient_get_position",
#     "webclient_post",
#     "webclient_close",
#     "webclient_read",
#     "webclient_write",
#     "webclient_header_fields_add",
#     "webclient_header_fields_get",
#     "webclient_response",
#     "webclient_request",
#     "webclient_request_header_add",
#     "webclient_resp_status_get",
#     "webclient_content_length_get"
# ]

# mod_export_api = f"--public-symbols {','.join(export_api_list)}"


cp -rf $lib_path/lib$mod_name.a $lib_path/$mod_name.a
python3 $mod_build_tool $mod_build_opt $lib_path/$mod_name.a

#Create a file system image
echo "======== gen littlefs image ========"
fs_tool_path=$armino_path/tools/build_tools/littlefs_tool/
fs_path=$build_path/littlefs_tool

if [ ! -d "$fs_path" ]; then   
    cp -rf $fs_tool_path $build_path
    mkdir -p $fs_path/littlefs
    mkdir -p $build_path/dynamic_app
fi

cp -r $lib_path/$mod_name.bin $fs_path/littlefs/
$fs_path/mklittlefs -c $fs_path/littlefs/ -b 4096 -p 256 -s $file_size $build_path/dynamic_app/littlefs_0x500000.bin

# Place all files related to dynamically loading images into the 'dynamic_app' directory.
cp -rf $lib_path/$mod_name.elf $build_path/dynamic_app/
cp -rf $lib_path/$mod_name.bin $build_path/dynamic_app/

echo "================= end $mod_name ========================="