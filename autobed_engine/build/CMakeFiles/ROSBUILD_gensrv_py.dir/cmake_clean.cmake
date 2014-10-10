FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/hrl_autobed_dev/srv"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/hrl_autobed_dev/srv/__init__.py"
  "../src/hrl_autobed_dev/srv/_add_bed_config.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
