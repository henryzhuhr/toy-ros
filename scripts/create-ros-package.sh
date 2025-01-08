package_group=${1:-"my_group"}
package_name=${2:-"my_package"}

package_dir=$(pwd)/modules/${package_group}
if [ ! -d $package_dir ]; then
    mkdir -p $package_dir
fi

echo "create package '${package_name}' in ${package_dir}"

cd $package_dir

ros2 pkg create "${package_name}" \
    --license Apache-2.0 \
    --build-type ament_cmake \
    --dependencies rclcpp

ros2 pkg create "${package_name}_py" \
    --license Apache-2.0 \
    --build-type ament_python \
    --dependencies rclpy

