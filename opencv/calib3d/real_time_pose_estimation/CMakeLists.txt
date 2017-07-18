cmake_minimum_required(VERSION 2.8)
project( Test )
find_package( OpenCV REQUIRED )

set(sample_dir src/)

set(sample_pnplib
        ${sample_dir}CsvReader.cpp
        ${sample_dir}CsvWriter.cpp
        ${sample_dir}ModelRegistration.cpp
        ${sample_dir}Mesh.cpp
        ${sample_dir}Model.cpp
        ${sample_dir}PnPProblem.cpp
        ${sample_dir}Utils.cpp
        ${sample_dir}RobustMatcher.cpp
        main.cpp
)

add_executable( Test ${sample_pnplib} )

