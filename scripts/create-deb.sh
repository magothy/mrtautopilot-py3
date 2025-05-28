#!/bin/bash

set -e
set -u
set -x

THIS_DIR=$(cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd)

# download/install the correct dependencies
rm -fr env
python3 -m venv env
env/bin/pip install -e "$THIS_DIR/.."

# build python package
env/bin/pip install build
env/bin/python3 -m build --outdir "$THIS_DIR/dist" "$THIS_DIR/.."

PYTHON_VERSION=3.$(python3 -c 'import sys; print(sys.version_info.minor)')
MRTAUTOPILOT_VERSION=$(env/bin/python3 -c 'import mrtautopilot; print(mrtautopilot.__version__)')

SHORT_SHA=${GITHUB_SHA::7}
PACKAGE_VERSION=$MRTAUTOPILOT_VERSION.$GITHUB_RUN_NUMBER-$SHORT_SHA
PACKAGE_NAME=python3-magothy-autopilot_${PACKAGE_VERSION}_all
DIST_PKG_DIR=$THIS_DIR/$PACKAGE_NAME/usr/lib/python3/dist-packages

# prepare the deb package structure
rm -fr $PACKAGE_NAME
mkdir -p $PACKAGE_NAME/DEBIAN
mkdir -p $DIST_PKG_DIR

cp -r "$THIS_DIR/DEBIAN"  $PACKAGE_NAME
sed -i "s/VERSION_STR/$PACKAGE_VERSION/g" $PACKAGE_NAME/DEBIAN/control

cp -r env/lib/python$PYTHON_VERSION/site-packages/mrtproto* "$DIST_PKG_DIR"
cp -r env/lib/python$PYTHON_VERSION/site-packages/mrtmavlink* "$DIST_PKG_DIR"
unzip -o "$THIS_DIR/dist/mrtautopilot-$MRTAUTOPILOT_VERSION-py3-none-any.whl" -d "$DIST_PKG_DIR"

dpkg-deb --root-owner-group --build $PACKAGE_NAME
