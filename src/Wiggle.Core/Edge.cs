using SkiaSharp;
using System;
using System.Collections.Generic;


namespace Wiggle.Core
{
    public class Edge{
        public int I, J; // vertex indices of the edge (endpoints A and B).
        public float DefaultLength; // length of the DefaultPosition
        public float Stiffness; // of the edges
    }
}