using System;
using System.Threading;

namespace HoloSuite
{
    class Application
    {
        static CloudCallbackDelegate cloudCallback;
        static void Main(string[] args)
        {
            cloudCallback = new CloudCallbackDelegate(CloudCallback);
           	
	        HoloSuiteInternal.initHoloSuite(CaptureType.OpenNI2, AudioCaptureType.None, CodecType.None, CodecType.None, RenderType.None, AudioRenderType.None);
	
            HoloSuiteInternal.initSession(SessionMode.Direct, "NOTHING");

            HoloSuiteInternal.setRemoteVisualCallback(cloudCallback);

            HoloSuiteInternal.startSession();

	        while (true)
	        {
                Thread.Sleep(1000);
	        }
        }

        static void CloudCallback(HoloPoint3D[] data, int size)
        {
            for(int i = 0; i < size; i++)
            {
                if(data[i].r > 0)
                {
                    Console.Write("WOW");
                }
            }
        }



    }
}
