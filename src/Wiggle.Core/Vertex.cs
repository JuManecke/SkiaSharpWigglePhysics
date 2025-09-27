using SkiaSharp;
using System;
using System.Collections.Generic;


namespace Wiggle.Core
{
    public class Vertex{
        public Vec2 CurrentPosition;
        public Vec2 PreviousPosition;
        public Vec2 DefaultPosition; // pulls back into this position when idle
        public float Mass = 1f;
        public bool IsPinned = false; // bottom row is pinned as an anchor
        public float Radius = 5f; // radius for vertex on vertex self-collision
    }
}