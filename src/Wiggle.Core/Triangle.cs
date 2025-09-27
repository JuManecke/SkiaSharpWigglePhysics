using SkiaSharp;
using System;
using System.Collections.Generic;


namespace Wiggle.Core
{
    public class Triangle{
        public SKPoint A, B, C;
        public SKColor FillColor;


        public Triangle(SKPoint a, SKPoint b, SKPoint c, SKColor color) // constructor
        {
            A = a;
            B = b;
            C = c;
            FillColor = color;
        }
    }
}