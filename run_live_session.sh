#!/bin/bash

set -e

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

export CARB_APP_PATH=${SCRIPT_DIR}/_build/linux-x86_64/release
export PYTHONHOME=${CARB_APP_PATH}/python-runtime

export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${PYTHONHOME}/lib:${CARB_APP_PATH}:/usr/local/google/home/shameek/omni-workspace/install/lib"

# Set this if running gzserver locally outside of Flowstate.
# export GZ_PARTITION=giza_ign

# export GZ_FILE_PATH=/google/obj/workspace/83528f8ee0e8d15942e6ef2eff8e437345df69ddf2ab25a20105a778c150f974/22fa6766-0958-4ab0-a5d3-5f2fb67bf2bf/blaze-out/k8-opt/bin/googlex/giza/simulation/gzserver/gzserver.runfiles/google3/
# export GZ_FILE_PATH=/usr/local/google/home/shameek/Desktop/test_bluebird_usd
export GZ_FILE_PATH=/var/apps/gzserver/meshes

echo Running script in ${SCRIPT_DIR}
pushd "$SCRIPT_DIR" > /dev/null
"${CARB_APP_PATH}/LiveSession" "$@"
popd > /dev/null
