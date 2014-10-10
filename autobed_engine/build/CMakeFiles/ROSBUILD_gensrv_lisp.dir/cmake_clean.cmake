FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/hrl_autobed_dev/srv"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/add_bed_config.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_add_bed_config.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
