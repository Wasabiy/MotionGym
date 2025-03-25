using UnityEngine;
using System.Collections;  
using System.Collections.Generic;
using System.Linq; 


namespace CustomNavMesh
{ 
    public class NavMesh{

        private List<Vector3> navMeshNodes; // List of walkable points
        private Transform agentTransform; // Reference to the agent

        public NavMesh(List<Vector3> nodes, Transform agent)
        {
            navMeshNodes = nodes; // Initialize the NavMesh with predefined nodes
            agentTransform = agent;
        }

        //Pathfinding:  find the shortest path between two points on the NavMesh. The A (A-star) algorithm* is commonly used for this.
        //FindPath(Vector3 start, Vector3 end) → List<Vector3>
        //Computes a path from start to end using a graph-based search algorithm.
        public List<Vector3> FindPath(Vector3 start, Vector3 end)
        {
            // Open and Closed Lists for A*
            List<Vector3> openSet = new List<Vector3> { start };
            HashSet<Vector3> closedSet = new HashSet<Vector3>();
            Dictionary<Vector3, Vector3> cameFrom = new Dictionary<Vector3, Vector3>();

            Dictionary<Vector3, float> gScore = new Dictionary<Vector3, float>();
            Dictionary<Vector3, float> fScore = new Dictionary<Vector3, float>();

            foreach (var node in navMeshNodes)
            {
                gScore[node] = float.MaxValue;
                fScore[node] = float.MaxValue;
            }

            gScore[start] = 0;
            fScore[start] = Vector3.Distance(start, end); // Heuristic estimate

            while (openSet.Count > 0)
            {
                // Get the node with the lowest fScore
                Vector3 current = GetLowestFScore(openSet, fScore);

                if (current == end) // Path found!
                {
                    return ReconstructPath(cameFrom, current);
                }

                openSet.Remove(current);
                closedSet.Add(current);

                foreach (Vector3 neighbor in GetNeighbors(current))
                {
                    if (closedSet.Contains(neighbor))
                        continue;

                    float tentativeGScore = gScore[current] + Vector3.Distance(current, neighbor);

                    if (!openSet.Contains(neighbor))
                        openSet.Add(neighbor);

                    if (tentativeGScore >= gScore[neighbor])
                        continue;

                    // Best path so far
                    cameFrom[neighbor] = current;
                    gScore[neighbor] = tentativeGScore;
                    fScore[neighbor] = gScore[neighbor] + Vector3.Distance(neighbor, end);
                }
            }

            return new List<Vector3>(); // No path found
        }

        private Vector3 GetLowestFScore(List<Vector3> openSet, Dictionary<Vector3, float> fScore)
        {
            Vector3 lowestNode = openSet[0];
            float lowestScore = fScore[lowestNode];

            foreach (Vector3 node in openSet)
            {
                if (fScore[node] < lowestScore)
                {
                    lowestNode = node;
                    lowestScore = fScore[node];
                }
            }
            return lowestNode;
        }

        private List<Vector3> ReconstructPath(Dictionary<Vector3, Vector3> cameFrom, Vector3 current)
        {
            List<Vector3> path = new List<Vector3> { current };

            while (cameFrom.ContainsKey(current))
            {
                current = cameFrom[current];
                path.Add(current);
            }

            path.Reverse();
            return path;
        }

        private List<Vector3> GetNeighbors(Vector3 node)
        {
            List<Vector3> neighbors = new List<Vector3>();

            foreach (Vector3 potentialNeighbor in navMeshNodes)
            {
                if (Vector3.Distance(node, potentialNeighbor) < 1.5f) // Adjust distance threshold as needed
                {
                    neighbors.Add(potentialNeighbor);
                }
            }

            return neighbors;
        }

        //Navigation Mesh Representation 
        //IsWalkable(Vector3 position) → bool
        //Checks if a given position is inside the NavMesh.
        public bool IsWalkable(Vector3 position)
        {
            return navMeshNodes.Contains(position);
        }

        // Finds the closest valid position on the NavMesh
        public Vector3 GetNearestNavMeshPoint(Vector3 position)
        {
            return navMeshNodes.OrderBy(node => Vector3.Distance(position, node)).FirstOrDefault();
        }

        // Moves the agent along the computed path
        public void MoveAgent(Vector3 targetPosition, float speed)
        {
            List<Vector3> path = FindPath(agentTransform.position, targetPosition);

            if (path.Count > 0)
            {
                agentTransform.position = Vector3.MoveTowards(agentTransform.position, path[0], speed * Time.deltaTime);
            }
        }

        // Adjusts the agent's rotation smoothly to follow the path
        public void SteerAgent(Vector3 direction, float turnSpeed)
        {
            if (direction != Vector3.zero)
            {
                Quaternion targetRotation = Quaternion.LookRotation(direction);
                agentTransform.rotation = Quaternion.Slerp(agentTransform.rotation, targetRotation, turnSpeed * Time.deltaTime);
            }
        }

        // Adjusts the agent's movement when an obstacle is detected
        public void AvoidObstacle(Vector3 obstaclePosition)
        {
            Vector3 avoidanceDirection = agentTransform.position - obstaclePosition;
            avoidanceDirection.y = 0; // Keep movement on the ground plane

            if (avoidanceDirection.magnitude < 1.5f) // Adjust threshold as needed
            {
                Vector3 newDirection = agentTransform.position + avoidanceDirection.normalized * 1.5f;
                MoveAgent(GetNearestNavMeshPoint(newDirection), 2f); // Move away from obstacle
            }
        }


    }

}