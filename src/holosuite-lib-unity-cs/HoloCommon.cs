using System;
using System.Runtime.InteropServices;

namespace HoloSuite
{
    public delegate void CloudCallbackDelegate([MarshalAs(UnmanagedType.LPArray,SizeParamIndex=1)] HoloPoint3D[] data, int size);

    [StructLayout(LayoutKind.Sequential, Pack=16, Size=32)]
    public struct HoloPoint3D
    {
        public float x;
        public float y;
        public float z;
        public float w;
        public byte r;
        public byte g;
        public byte b;
        public byte a;
    }

    public enum CaptureType
    {
        None = -1,
        FilePLY = 0,
        FilePCD = 1,
        FileOBJ = 2,
        FileONI = 3,
        OpenNI2 = 4
    }

    public enum CodecType
    {
        None = -1,
        PassthroughCloud = 0,
        PassthroughRGBAZ = 1,
        Octree = 2,
        h264 = 3,
        Opus = 4
    }

    public enum AudioCaptureType
    {
        None = -1,
        PortAudio = 0
    }

    public enum AudioRenderType
    {
        None = -1,
        PortAudio = 0
    }

    public enum RenderType
    {
        None = -1,
        Vis2D = 0,
        Vis3D = 1,
        DSCP_MKII = 2,
        DSCP_MKIV = 3
    }

    public enum SessionMode
    {
        Server = 0,
        Client = 1,
        Loopback = 2,
        Direct = 3
    }

}
