For Ubuntu 14.04:

Install OGRE 1.8.1 from source:
   - Download source from http://www.ogre3d.org/download/source
   - Install Cg
   - Configure with cmake-gui. Enable OGRE_STATIC and fix FREETYPE path error.

Compile libhand statically:
   - Use cmake-gui to configure. Fix FREETYPE path.
   - Modify hand_cpp/source/CMakeLists.txt and add the following to TARGET_LINK_LIBRARIES(hand_render):
     boost_system dl Xt Xrandr Xaw

