@echo off

set project_name=Grasp_BYD
set sdk_version=Vision_1.0.0

cd %project_name%
dir

@echo *** update doc ***
xcopy ..\Doc\Vision_SolutionGuide.pdf  /d /y

@echo *** update .dll ***
xcopy ..\vision_grasp_pose.dll      %sdk_version%\bin  /d /y
xcopy ..\vision_camera_data.dll      %sdk_version%\bin  /d /y
xcopy ..\vision_pose_estimation.dll  %sdk_version%\bin  /d /y
xcopy ..\vision_recognition.dll      %sdk_version%\bin  /d /y
xcopy ..\vision_registration_3d.dll  %sdk_version%\bin  /d /y
xcopy ..\vision_segmentation.dll     %sdk_version%\bin  /d /y

@echo *** update .h ***
xcopy ..\..\..\grasp_pose\grasp_pose.h   %sdk_version%\include  /d /y

@echo *** update .lib ***
xcopy ..\vision_grasp_pose.lib            %sdk_version%\lib  /d /y

@echo *** compress .rar ***
cd ..
"C:\Program Files\WinRAR\WinRAR.exe" a -ep1 %project_name%.rar %project_name%


pause

goto start
pause
:start


