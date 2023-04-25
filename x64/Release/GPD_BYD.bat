@echo off

set project_name=GPD_BYD
set sdk_version=gpd_1.0.0

cd %project_name%
dir

@echo *** update doc ***
xcopy ..\Doc\GPD_UserGuide.pdf  /d /y

@echo *** update .dll ***
xcopy ..\gpd_grasp_pose.dll      %sdk_version%\bin  /d /y
xcopy ..\gpd_camera_data.dll      %sdk_version%\bin  /d /y
xcopy ..\gpd_pose_estimation.dll  %sdk_version%\bin  /d /y
xcopy ..\gpd_recognition.dll      %sdk_version%\bin  /d /y
xcopy ..\gpd_registration_3d.dll  %sdk_version%\bin  /d /y
xcopy ..\gpd_segmentation.dll     %sdk_version%\bin  /d /y

@echo *** update .h ***
xcopy ..\..\..\grasp_pose\grasp_pose.h   %sdk_version%\include  /d /y

@echo *** update .lib ***
xcopy ..\gpd_grasp_pose.lib            %sdk_version%\lib  /d /y

@echo *** compress .rar ***
cd ..
"C:\Program Files\WinRAR\WinRAR.exe" a -ep1 %project_name%.rar %project_name%


pause

goto start
pause
:start


