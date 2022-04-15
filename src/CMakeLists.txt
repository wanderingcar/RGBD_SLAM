add_library(rgbd_slam SHARED
Config.cpp
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
)

target_link_libraries(rgbd_slam ${THIRD_PARTY_LIBS})
