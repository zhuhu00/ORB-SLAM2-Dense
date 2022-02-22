#!/bin/bash

set -e
# docker 的入口会运行的程序

echo "================ ORB-SLAM2 Ready =============="

cd /root/orbslam2

exec "$@"
