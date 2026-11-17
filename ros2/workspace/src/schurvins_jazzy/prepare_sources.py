"""Generate narrowly adapted build inputs; never edit the upstream checkout."""
from pathlib import Path
import sys

up, out = map(Path, sys.argv[1:])
(out / 'include/svo_ros').mkdir(parents=True, exist_ok=True)
(out / 'include/svo').mkdir(parents=True, exist_ok=True)
for relative, target in [
    ('svo_ros/src/svo_factory.cpp', out / 'svo_factory.cpp'),
    ('svo_ros/include/svo_ros/svo_factory.h', out / 'include/svo_ros/svo_factory.h'),
]:
    source = (up / relative).read_text()
    source = source.replace('#include <ros/package.h>', '#include <hexbot_vio/compat.hpp>')
    source = source.replace('#include <ros/ros.h>', '#include <hexbot_vio/compat.hpp>')
    source = source.replace('#include <vikit/params_helper.h>', '')
    source = source.replace('ros::NodeHandle', 'YAML::Node')
    source = source.replace('vk::param<', 'hexbot_vio::param<')
    source = source.replace('ros::package::getPath("svo")', 'std::string(".")')
    target.write_text(source)

# This include is unused in the transport-independent core but pulls ROS 1.
source = (up / 'svo/include/svo/frame_handler_base.h').read_text()
source = source.replace('#include <vikit/params_helper.h>', '')
(out / 'include/svo/frame_handler_base.h').write_text(source)

# The upstream non-SIMD FAST-10 branch has an undefined pyramid index `L`.
# ARM64 uses this scalar FAST-10 path to preserve the detector type used on x86.
source = (up / 'svo_direct/src/feature_detection_utils.cpp').read_text()
source = source.replace('img_pyr[L]', 'img_pyr[level]')
(out / 'feature_detection_utils.cpp').write_text(source)
