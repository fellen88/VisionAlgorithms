@echo off

set project_name=.\BinPicking\
set sdk_version=Vision_1.0.0

cd %project_name%
dir

@echo update UserManual
xcopy ..\Doc\Vision_UserManual.pdf  /d /y

@echo update .dll
xcopy ..\vision_bin_picking.dll      %sdk_version%\bin  /d /y
xcopy ..\vision_camera_data.dll      %sdk_version%\bin  /d /y
xcopy ..\vision_pose_estimation.dll  %sdk_version%\bin  /d /y
xcopy ..\vision_recognition.dll      %sdk_version%\bin  /d /y
xcopy ..\vision_registration_3d.dll  %sdk_version%\bin  /d /y
xcopy ..\vision_segmentation.dll     %sdk_version%\bin  /d /y

@echo update .h
xcopy ..\..\..\bin_picking\bin_picking.h   %sdk_version%\include  /d /y

@echo update .lib
xcopy ..\vision_bin_picking.lib            %sdk_version%\lib  /d /y


pause

goto start
pause
:start


