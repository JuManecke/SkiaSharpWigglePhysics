using SkiaSharp;
using System;
using System.Collections.Generic;


namespace Wiggle.Core
{
    public class TriangleIndex{
        public int A, B, C;
        public SKColor Fill;
        public float DefaultHeight_C_AB;
        public float DefaultHeight_A_BC;
        public float DefaultHeight_B_CA;


        public float MinHeightFraction = 0.35f; // Minimum height to prevent collapse or flip of triangles
    }
}