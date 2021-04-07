#!/bin/bash
echo 'args:' $@


DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

source ${DIR}/../../install/local_setup.sh && \
ros2 pkg create_nodejs $@
