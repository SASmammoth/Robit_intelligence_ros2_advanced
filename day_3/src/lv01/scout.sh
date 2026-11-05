#!/bin/bash
set -eo pipefail

echo "hello"

for i in {10..95};
do
echo "domainID ${i}"
export ROS_DOMAIN_ID=$i
ros2 topic list
echo ""
done
