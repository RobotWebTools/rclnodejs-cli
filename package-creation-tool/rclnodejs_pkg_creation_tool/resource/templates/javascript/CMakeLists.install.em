# install rules added by ros2pkg_configure_nodejs
install(FILES 
  package.json
  DESTINATION share/${PROJECT_NAME}/
 )
 
 # only use with javascript projects (not typescript)
 install(DIRECTORY 
  src/
  DESTINATION share/${PROJECT_NAME}/dist
  OPTIONAL
 )
 
install(DIRECTORY 
  config
  dist
  launch
  node_modules
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
 )
 