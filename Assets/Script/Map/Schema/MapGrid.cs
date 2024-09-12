using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Map
{
    public class MapGrid
    {
        public float width, height, length;

        public MapGrid(float width, float height, float length)
        {
            this.width = width;
            this.height = height;
            this.length = length;
        }
    }
}