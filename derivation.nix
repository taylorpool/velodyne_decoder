{ lib, stdenv, eigen, cmake, rosPackages, python311, llvmPackages_16 }:
stdenv.mkDerivation {
  pname = "velodyne_decoder";
  version = "0.1.0";

  src = ./.;

  nativeBuildInputs = [
    cmake 
    rosPackages.noetic.rosparam
    rosPackages.noetic.rosbag
    rosPackages.noetic.rostopic
    rosPackages.noetic.rosnode
    rosPackages.noetic.ros-core
    rosPackages.noetic.tf2
    llvmPackages_16.clang-unwrapped
  ];

  propagatedBuildInputs = [
    eigen
    rosPackages.noetic.sensor-msgs
    rosPackages.noetic.roscpp
    rosPackages.noetic.velodyne-msgs
  ];

  meta = with lib; {
    description = "A package to decode velodyne packets";
  };
}
