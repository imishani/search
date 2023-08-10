# uninstall.cmake. A file for uninstalling the installed files. It is used with the command "make uninstall" and configured in the top-level CMakeLists.txt file.

# Print uninstalling...
message(STATUS "Uninstalling...")

# Now do this iteratively going through rows in install_manifest.txt
file(STRINGS install_manifest.txt files)
foreach(file ${files})
    message(STATUS "Removing ${file}")
    file(REMOVE ${file})

    # If the file still exists, try with sudo.
    if(EXISTS ${file})
        message(STATUS "    Removing ${file} with sudo")
        execute_process(COMMAND sudo rm ${file})
    endif()
endforeach()