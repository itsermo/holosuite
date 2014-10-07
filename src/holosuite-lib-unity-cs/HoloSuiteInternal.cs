using System;
using System.Runtime.InteropServices;

namespace HoloSuite
{
    public class HoloSuiteInternal
    {
        
        [DllImport("holosuite-lib-unity",CallingConvention=CallingConvention.Cdecl)]
        public extern static void setRemoteVisualCallback(CloudCallbackDelegate visualCallback);

        [DllImport("holosuite-lib-unity.dll",CallingConvention=CallingConvention.Cdecl)]
        public extern static void initHoloSuite(CaptureType capture,
            AudioCaptureType audioCapture,
            CodecType captureEncoder,
            CodecType audioCaptureEncoder,
            RenderType visOutput,
            AudioRenderType audioOutput);

        [DllImport("holosuite-lib-unity",CallingConvention=CallingConvention.Cdecl)]
        public extern static void deinitHoloSuite();

        [DllImport("holosuite-lib-unity",CallingConvention=CallingConvention.Cdecl)]
        public extern static void startServer();

        [DllImport("holosuite-lib-unity",CallingConvention=CallingConvention.Cdecl)]
        public extern static void connectToServer(string address);

        [DllImport("holosuite-lib-unity.dll",CallingConvention=CallingConvention.Cdecl)]
        public extern static void initSession(SessionMode sessionMode, string address);

        [DllImport("holosuite-lib-unity",CallingConvention=CallingConvention.Cdecl)]
        public extern static void deInitSession();

        [DllImport("holosuite-lib-unity",CallingConvention=CallingConvention.Cdecl)]
        public extern static void startSession();

        [DllImport("holosuite-lib-unity",CallingConvention=CallingConvention.Cdecl)]
        public extern static void stopSession();
    }
}
