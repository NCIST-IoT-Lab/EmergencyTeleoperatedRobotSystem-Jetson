# !/bin/bash 

# 该脚本用于编译protobuf文件并将生成的文件复制到正确的位置

# 将脚本路径赋值给变量
SCRIPT_PATH=$(cd "$(dirname "$0")"; pwd)

PROTO_PATH=$SCRIPT_PATH

CPP_PATH=$SCRIPT_PATH/../build
PYTHON_PATH=$SCRIPT_PATH/../src/object_detection/protobuf
CSHARP_PATH=$SCRIPT_PATH/csharp

protoc -I=$PROTO_PATH --cpp_out=$CPP_PATH --mypy_out=$PYTHON_PATH --python_out=$PYTHON_PATH --csharp_out=$CSHARP_PATH $PROTO_PATH/*.proto 

# 压缩 C# 文件
cd $CSHARP_PATH
zip -r ETRS_protobuf_csharp.zip ./*