if [ -z "$HERCULES_DESCRIPTION" ]; then
  export HERCULES_DESCRIPTION=$(catkin_find --share --first-only hercules_description urdf/description.xacro 2>/dev/null)
fi
