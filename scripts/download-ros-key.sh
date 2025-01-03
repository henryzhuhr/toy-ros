mkdir -p .cache
if [ -f .cache/ros.key ]; then
    echo "ros key already exists, skip"
    exit 0
fi
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o .cache/ros.key