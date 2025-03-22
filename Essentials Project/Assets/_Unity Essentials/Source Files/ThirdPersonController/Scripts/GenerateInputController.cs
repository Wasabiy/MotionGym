using System;
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using UnityEngine;
using Random = UnityEngine.Random;

namespace StarterAssets
{
    public class GenerateInputController 
    {
        [Tooltip("Collection of Vectors that will move the robot")]
        public Vector2[] MovementInputs = new Vector2[10000];

        [Tooltip("Collection of Speeds that will move the robot")]
        public float[] MovementSpeeds = new float[10000];

        [Tooltip("The current movement direction being applied")]
        public Vector2 currentVector;

        [Tooltip("The current speed being applied")]
        public float currentSpeed;

        [Tooltip("Current index in the array")]
        public int arrayCounter = 0; 

        public GenerateInputController()
        {
                
           
                
                MovementInputs[0] = new Vector2(1.0f, -1.0f);
                MovementInputs[1] =new Vector2(-1.0f, 0.0f);
                MovementInputs[2] = new Vector2(0.0f, -1.0f);
                MovementInputs[3] =  new Vector2(-1.0f, -1.0f);
                MovementInputs[4] =  new Vector2(1.0f, 1.0f);






        }

        public void Iterate()
        {
            if (arrayCounter < MovementInputs.Length)
            {
                currentVector = MovementInputs[arrayCounter];
                currentSpeed = MovementSpeeds[arrayCounter];
                arrayCounter++;
            }

        }
    }
}
