using UnityEngine;
using UnityEngine.PlayerLoop;

namespace _Unity_Essentials.Scripts
{
    public class EnvironmentRobotInterface : MonoBehaviour
    {
        private GameObject mainCamera;
        private int currentAction;
        private int action;
        private GameObject agent;
        private GameObject target;
        private int numberOfActions;
        private float previousDistanceToTarget;

        public void setPreviousDistanceToTarget(float distanceToTarget)
        {
            this.previousDistanceToTarget = distanceToTarget;
        }

        public float getPreviousDistanceTotarget()
        {
            return this.previousDistanceToTarget;
        }
        public int getAction(Vector2 vector) // doesnt consider the size of the vector 
        {
            if (vector.x == 1 && vector.y == 0)
            {
                return 1; //right 
            } else if (vector.x == -1 && vector.y == 0)

            {
                return 2; // left
            } else if (vector.x == 0 && vector.y == 1)
            {
                return 3; //forward 
            }else if (vector.x == 0 && vector.y == -1)
            {
                return 4; //back
            }
            else
            {
                return 0; //stay still
            }
        }

        public (Vector2 observation, float reward, bool done, string information) step(Vector2 vector)
        {
            int action = getAction(vector);
            
            //Get observation by the distance between agent and target
            Vector2 observation = agent.transform.position - target.transform.position;
            float distance = Mathf.Abs(observation.x) + Mathf.Abs(observation.y);
            
            //Get reward based on time and distance to the target 
            float reward = getReward(distance);
            
            //Get done if the distance is 0
            bool done = false;
            if (distance < 20) {done = true;}
            
            //information is not important right now 
            string information = "";
            
            setPreviousDistanceToTarget(distance);
            return (observation, reward, done, information);
        }

        public float getReward(float distance)
        {
            if (distance < previousDistanceToTarget)
            {
                return 1;
            }
            else if (distance == previousDistanceToTarget)
            {
                return 0;
            }
            else
            {
                return -1;
            }
        }
        

        private void Start()
        {
            this.numberOfActions = 0;
            
        }
        private void Update()
        {
            numberOfActions++;
        }
        
    }
}