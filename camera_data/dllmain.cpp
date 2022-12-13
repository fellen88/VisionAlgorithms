// dllmain.cpp : 定义 DLL 应用程序的入口点。
#include "stdafx.h"

void FatalMessageDump(const char* data, int size)
{
	google::ShutdownGoogleLogging();
}

BOOL APIENTRY DllMain( HMODULE hModule,
                       DWORD  ul_reason_for_call,
                       LPVOID lpReserved
                     )
{
    switch (ul_reason_for_call)
    {
    case DLL_PROCESS_ATTACH:
			//FLAGS_logtostderr = false;            //是否将日志输出到stderr，而非文件
			//FLAGS_alsologtostderr = false;            //日志记录到文件的同时输出到stderr
			//FLAGS_colorlogtostderr = true;            //是否将彩色日志输出到stderr
			////FLAGS_drop_log_memory = true;            //日志写到文件的时候删除其在内存中的buf
			//FLAGS_log_prefix = true;            //每行log加前缀，其格式为Log line format: [IWEF]mmdd hh:mm:ss.uuuuuu threadid file:line] msg
			//FLAGS_minloglevel = google::GLOG_ERROR;            //日志最低记录等级
			//FLAGS_logbuflevel = google::GLOG_INFO;            //缓存日志的最低等级 , -1表示不缓存,0表示只缓存google::INFO
			//FLAGS_logbufsecs = 30;            //日志最多缓存的秒数，0表示实时输出
			//FLAGS_log_dir = "";               //设置日志文件输出目录
			//FLAGS_max_log_size = 1800;            //最大日志大小（MB）, 如果设置为0将默认为1
			//FLAGS_stop_logging_if_full_disk = false;            //磁盘满停止记录日志
			google::InitGoogleLogging("pose_estimation.dll");
			google::SetLogDestination(google::GLOG_INFO, "Log/PoseEstimation_");
			google::FlushLogFilesUnsafe(google::GLOG_INFO);
			google::SetStderrLogging(google::GLOG_INFO);
			google::SetLogFilenameExtension("log_");
			google::InstallFailureSignalHandler();
		  google::InstallFailureWriter(&FatalMessageDump);
			LOG(INFO) << "camera_data dll attach ";
			break;
    case DLL_THREAD_ATTACH:
		break;
    case DLL_THREAD_DETACH:
		break;
    case DLL_PROCESS_DETACH:
			LOG(INFO) << "camera_data dll detach ";
			//google::ShutdownGoogleLogging();
      break;
    }
    return TRUE;
}

