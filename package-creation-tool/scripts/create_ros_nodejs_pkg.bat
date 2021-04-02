echo off
SET CUR_FILE=%~dp0
SET DIR=%CUR_FILE:~0%
echo on

%DIR%..\..\install\local_setup.bat && ros2 pkg create_nodejs %*
