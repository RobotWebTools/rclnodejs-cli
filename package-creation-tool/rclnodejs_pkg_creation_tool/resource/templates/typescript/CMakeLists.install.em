# install rules added by ros2pkg_configure_nodejs
install(FILES 
  package.json
  DESTINATION share/${PROJECT_NAME}/
 )
 
install(DIRECTORY 
  config
  dist
  launch
  node_modules
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
 )
 