add_library(rgbd_slam SHARED
Config.cpp
#example.cpp
#profiler.cpp
types/Camera.cpp 
types/Feature.cpp
types/Frame.cpp
types/Map.cpp
types/MapPoint.cpp
types/Object.cpp
utils/Viewer.cpp
core/Tracker.cpp
core/Matcher.cpp
core/Extractor.cpp
core/Optimizer.cpp
core/Detector.cpp
detect_3d_cuboid/box_proposal_detail.cpp
detect_3d_cuboid/matrix_utils.cpp
detect_3d_cuboid/object_3d_util.cpp
line_lbd/libs/binary_descriptor.cpp
line_lbd/libs/binary_descriptor_matcher.cpp
line_lbd/libs/LSDDetector.cpp
line_lbd/libs/lsd.cpp
line_lbd/libs/bitarray.hpp
line_lbd/libs/bitops.hpp
line_lbd/libs/draw.cpp
line_lbd/libs/precomp.hpp
line_lbd/libs/types.hpp
line_lbd/class/line_lbd_allclass.cpp
# yolov3/blas.c
# yolov3/box.c
# yolov3/cuda.c
# yolov3/darknet.c
# yolov3/data.c
# yolov3/detector.c
# yolov3/image.c
# yolov3/list.c
# yolov3/matrix.c
# yolov3/tree.c
# yolov3/utils.c
)


target_link_libraries(rgbd_slam ${THIRD_PARTY_LIBS})
