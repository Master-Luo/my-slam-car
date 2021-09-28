add_library(myslam SHARED
        frame.cpp
        feature.cpp
        frontend.cpp
        backend.cpp)

target_link_libraries(myslam
        ${THIRD_PARTY_LIBS})
