{ lib, stdenv, eigen, cmake, rosPackages }:
stdenv.mkDerivation {
  pname = "velodyne_decoder";
  version = "0.1.0";

  src = ./.;

  nativeBuildInputs = [
    cmake 
    rosPackages.noetic.rosparam
    rosPackages.noetic.rosbag
    rosPackages.noetic.rostopic
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
