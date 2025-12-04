using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class Swarm : MonoBehaviour
{
    public struct BBoid
    {
        public Vector3 position;
        public Vector3 forward;
        public Vector3 velocity;
        public Vector3 alignment;
        public Vector3 cohesion;
        public Vector3 separation;
        public Vector3 obstacle;
        public Vector3 currentTotalForce;
        public int neighborCount; //taylor added this 
    }

    [Header("the BOID Settings")]
    public Transform boidPrefab;
    public int numberOfBoids = 200;
    public float boidForceScale = 20f;
    public float maxSpeed = 5.0f;
    public float rotationSpeed = 40.0f;

    [Header("Rule Weights")]
    public float obstacleCheckRadius = 1.0f;
    public float separationWeight = 1.1f;
    public float alignmentWeight = 0.5f;
    public float cohesionWeight = 1f;
    public float goalWeight = 1f;
    public float obstacleWeight = 0.9f;
    public float wanderWeight = 0.3f;

    [Header("the BOID Perception")]
    public float neighbourDistance = 2.0f;
    public float initializationRadius = 1.0f;
    public float initializationForwardRandomRange = 50f;

    [Header("Obstacles")]
    public float obstacleAvoidanceRadius = 1.0f;

    private BBoid[] boids;
    private Transform[] boidObjects;
    private float sqrNeighbourDistance;

    private Vector3 boidZeroGoal;
    private NavMeshPath boidZeroPath;
    private int currentCorner;
    private bool boidZeroNavigatingTowardGoal = false;

    //add the world bounds, as set out in the assignment instructions
    private const float worldMinX = -8f;
    private const float worldMaxX = 8f;
    private const float worldMinY = 1f;
    private const float worldMaxY = 4f;
    private const float worldMinZ = -8f;
    private const float worldMaxZ = 8f;

    //added by taylor 
    private const float EPSILON = 0.0001f; //frequently reused small threshold value to avoid zero-div
    private LayerMask obstacleLayer; //for getting the cylinder obstcles, which are already on a layer called Obstacles in the scene


    /// <summary>
    /// Start, this function is called before the first frame
    /// </summary>
    private void Start()
    {
        sqrNeighbourDistance = neighbourDistance * neighbourDistance;
        boidZeroPath = new NavMeshPath();
        obstacleLayer = LayerMask.GetMask("Obstacles");
        InitBoids();
    }

    /// <summary>
    /// ComputeObstacles, Initializes the obstacles via layermask - all the cylinder obstacles are on Layer: Obstacles in Scene, via Inspector
    /// Wall Boundaries are computed not that way, but by the present locations of the wall boundaries as definied in parameters above
    /// </summary>
    private void ComputeObstacles(int i, Vector3 posI)
    {
        Vector3 obsAcc = Vector3.zero;

        // find nearby colliders on obstacleLayer -- obstacle cylinders are on Layer: Obstacles in scene
        Collider[] hitCols = Physics.OverlapSphere(posI, obstacleAvoidanceRadius, obstacleLayer);
        foreach (Collider col in hitCols)
        {
            Vector3 toObs = posI - col.ClosestPoint(posI);
            float dist = toObs.magnitude;
            if (dist > EPSILON)
                obsAcc += toObs.normalized * (1f - (dist / obstacleAvoidanceRadius));
        }

        // world boundaries
        float buffer = 0.5f;
        Vector3 worldObs = Vector3.zero;
        if (posI.x > worldMaxX - buffer) worldObs.x = -1f;
        else if (posI.x < worldMinX + buffer) worldObs.x = 1f;
        if (posI.z > worldMaxZ - buffer) worldObs.z = -1f;
        else if (posI.z < worldMinZ + buffer) worldObs.z = 1f;
        if (posI.y > worldMaxY - buffer) worldObs.y = -1f;
        else if (posI.y < worldMinY + buffer) worldObs.y = 1f;

        if (worldObs.sqrMagnitude > EPSILON)
            obsAcc += worldObs.normalized;

        boids[i].obstacle = (obsAcc.sqrMagnitude > EPSILON) ? obsAcc.normalized : Vector3.zero;
    }

    /// <summary>
    /// Initialize the array of boids
    /// </summary>
    private void InitBoids()
    {
        boids = new BBoid[numberOfBoids];
        boidObjects = new Transform[numberOfBoids];

        for (int i = 0; i < numberOfBoids; i++)
        {
            //set initial positions and apparate the boid
            Vector3 pos = transform.position + Random.insideUnitSphere * initializationRadius;
            pos.y = Mathf.Clamp(pos.y, worldMinY, worldMaxY);

            Transform t = Instantiate(boidPrefab, pos, Quaternion.identity, transform);
            boidObjects[i] = t;

            //initial forward - go forward within range - no dest necessary
            Vector3 fwd;
            if (i == 0 && boidZeroNavigatingTowardGoal)
            {
                //boidZero points towards(ish) goal
                fwd = (boidZeroGoal - pos).normalized;
                if (fwd.sqrMagnitude < EPSILON) fwd = Vector3.forward;
            }
            else
            {
                //boidZero does not point towards(ish) goal so boids randomly spread and forward
                float angle = Random.Range(-initializationForwardRandomRange, initializationForwardRandomRange);
                fwd = Quaternion.Euler(0, angle, 0) * Vector3.forward;
            }
            fwd = fwd.normalized;

            //ensure this particular instance of a boid is actually initialized!
            boids[i] = new BBoid();
            //assign random init speed
            Vector3 vel = fwd * Random.Range(0.5f * maxSpeed, maxSpeed);
            boids[i].position = pos;
            boids[i].forward = fwd;
            boids[i].velocity = vel;
            boids[i].alignment = Vector3.zero;
            boids[i].cohesion = Vector3.zero;
            boids[i].separation = Vector3.zero;
            boids[i].obstacle = Vector3.zero;
            boids[i].currentTotalForce = Vector3.zero;
            boids[i].neighborCount = 0;

            //orient the object
            if (vel.sqrMagnitude > EPSILON) t.forward = vel.normalized;
        }
    }

    /// <summary>
    /// Reset the particle forces
    /// </summary>
    public void ResetBoidForces()
    {
        int n = boids.Length;
        for (int i = 0; i < n; i++)
        {
            boids[i].alignment = Vector3.zero;
            boids[i].cohesion = Vector3.zero;
            boids[i].separation = Vector3.zero;
            boids[i].obstacle = Vector3.zero;
            boids[i].currentTotalForce = Vector3.zero;
            boids[i].neighborCount = 0;
        }
    }
    
    /// <summary>
    /// Sim Loop
    /// </summary>
    private void FixedUpdate()
    {
        if (boids == null || boids.Length == 0) return;
        float dt = Time.fixedDeltaTime;
        ResetBoidForces();

        int count = boids.Length;

        //first pass deals with neighbours and obstacles computation
        for (int i = 0; i < count; i++)
            ComputeBoidsForces(i);

        for (int i = 0; i < count; i++)
            ApplySteering(i, dt);
    }

    private void ComputeBoidsForces(int i)
    {
        Vector3 posI = boids[i].position;
        Vector3 fwdI = boids[i].forward;

        Vector3 sepAcc = Vector3.zero;
        Vector3 alignAcc = Vector3.zero;
        Vector3 cohAcc = Vector3.zero;
        int neighborCount = 0;

        // neighbor detection (naive O(n^2) as instructed in assignment description)
        for (int j = 0; j < boids.Length; j++)
        {
            if (i == j) continue;
            Vector3 toNeighbor = boids[j].position - posI;
            float sqrDist = toNeighbor.sqrMagnitude;
            if (sqrDist > sqrNeighbourDistance) continue;
            if (Vector3.Dot(toNeighbor.normalized, fwdI) <= 0f) continue;

            neighborCount++;

            float dist = Mathf.Sqrt(sqrDist);
            if (dist > EPSILON)
                sepAcc += (posI - boids[j].position).normalized * (1f - (dist / neighbourDistance));

            alignAcc += boids[j].velocity;
            cohAcc += boids[j].position;
        }
        // finalize the rule vectors
        Vector3 separationVec = Vector3.zero;
        Vector3 alignmentVec = Vector3.zero;
        Vector3 cohesionVec = Vector3.zero;

        if (neighborCount > 0)
        {
            separationVec = sepAcc / neighborCount;
            if (separationVec.sqrMagnitude > EPSILON)
                separationVec = separationVec.normalized;

            Vector3 avgVel = alignAcc / neighborCount;
            if (avgVel.sqrMagnitude > EPSILON)
                alignmentVec = avgVel.normalized;

            //cohesion, here we are trying scale by distance to local center (inverse-square style), because it's too strong!
            Vector3 center = cohAcc / neighborCount;
            Vector3 toCenter = center - posI;
            float distanceFactor = Mathf.Clamp01(toCenter.magnitude / neighbourDistance);
            if (toCenter.sqrMagnitude > EPSILON)
                cohesionVec = toCenter.normalized * distanceFactor; // scale by distance factor

            /* Vector3 toCenter = (cohAcc / neighborCount) - posI;
             if (toCenter.sqrMagnitude > EPSILON)
                 cohesionVec = toCenter / neighbourDistance;*/

            //adding some randomness to the boids
            Vector3 randomWander = new Vector3(Random.Range(-0.2f, 0.2f), Random.Range(-0.2f, 0.2f), Random.Range(-0.2f, 0.2f));
            if (toCenter.sqrMagnitude > EPSILON)
                cohesionVec = (toCenter / neighbourDistance) * 0.7f + randomWander * 0.3f;
            else
                cohesionVec = randomWander * 0.3f;
        }
        else //added for a true randomWander when there are no boid neighbours
        {
            cohesionVec = new Vector3(Random.Range(-0.3f, 0.3f), Random.Range(-0.3f, 0.3f), Random.Range(-0.3f, 0.3f));
        }
        //update the rule vectors since we're past neighbourCount > 0
        boids[i].separation = separationVec;
        boids[i].alignment = alignmentVec;
        boids[i].cohesion = cohesionVec;
        boids[i].neighborCount = neighborCount; // stored for wander

        //call for the check on obstacle avoidance
        ComputeObstacles(i, posI);
    }

    /// <summary>
    /// Convert a rule vector into a steering force, scaled by weight and maxSpeed.
    /// </summary>
    private Vector3 RuleSteer(Vector3 ruleVec, Vector3 currentVelocity, float weight)
    {
        if (ruleVec.sqrMagnitude < EPSILON) return Vector3.zero;

        // desired velocity in direction of rule vector at max speed
        Vector3 desired = ruleVec.normalized * maxSpeed;

        // steering = desired - current velocity
        Vector3 steer = desired - currentVelocity;

        if (steer.sqrMagnitude > EPSILON)
            return steer.normalized * weight * boidForceScale;

        return Vector3.zero;
    }

    private void ApplySteering(int i, float dt)
    {
        Vector3 vel = boids[i].velocity;
        Vector3 totalForce = Vector3.zero;

        totalForce += RuleSteer(boids[i].separation, vel, separationWeight);
        totalForce += RuleSteer(boids[i].alignment, vel, alignmentWeight);
        totalForce += RuleSteer(boids[i].cohesion, vel, cohesionWeight);
        totalForce += RuleSteer(boids[i].obstacle, vel, obstacleWeight);

        // boid zero goal
        if (i == 0 && boidZeroNavigatingTowardGoal)
            totalForce += ComputeGoalForce(i, vel);

        // wander if that boid has no neighbours
        if (boids[i].neighborCount == 0)
        {
            Vector3 wanderVec = vel.sqrMagnitude > EPSILON ? vel.normalized : Vector3.forward;
            totalForce += RuleSteer(wanderVec, vel, wanderWeight);
        }

        boids[i].currentTotalForce = totalForce;

        // integrate ***this is the symplectic euler here
        vel += totalForce * dt;
        if (vel.magnitude > maxSpeed) vel = vel.normalized * maxSpeed;

        Vector3 pos = boids[i].position + vel * dt;
        boids[i].velocity = vel;
        if (vel.sqrMagnitude > EPSILON) boids[i].forward = vel.normalized;
        boids[i].position = pos;

        // update visual transform
        if (boidObjects[i] != null)
        {
            boidObjects[i].position = pos;
            if (vel.sqrMagnitude > EPSILON) boidObjects[i].forward = boids[i].forward;
        }
    }

    private Vector3 ComputeGoalForce(int i, Vector3 vel)
    {
        if (boidZeroPath == null || boidZeroPath.status != NavMeshPathStatus.PathComplete || boidZeroPath.corners.Length <= 1)
            return Vector3.zero;

        // checkout the current corner
        Vector3 targetCorner = boidZeroPath.corners[currentCorner];
        Vector3 toCorner = targetCorner - boids[i].position;
        Vector3 goalVec = (toCorner.sqrMagnitude > EPSILON) ? toCorner.normalized : Vector3.zero;

        // check on the progress of the path
        NavMeshHit hit;
        if (NavMesh.SamplePosition(boids[i].position, out hit, 1.0f, NavMesh.AllAreas))
        {
            if ((hit.position - targetCorner).magnitude <= 1.0f)
            {
                if (currentCorner < boidZeroPath.corners.Length - 1)
                    currentCorner++;
                else
                {
                    boidZeroPath.ClearCorners();
                    boidZeroNavigatingTowardGoal = false;
                    currentCorner = 0;
                }
            }
        }

        return RuleSteer(goalVec, vel, goalWeight);
    }

    private void Update()
    {
        // Render information for boidzero, useful for debugging forces and path planning
        int boidCount = boids.Length;
        for (int i = 1; i < boidCount; i++)
        {
            Vector3 boidNeighbourVec = boids[i].position - boids[0].position;
            if (boidNeighbourVec.sqrMagnitude < sqrNeighbourDistance && Vector3.Dot(boidNeighbourVec, boids[0].forward) > 0f)
            { 
                Debug.DrawLine(boids[0].position, boids[i].position, Color.blue);
            }
        }
        
        Debug.DrawLine(boids[0].position, boids[0].position + boids[0].alignment, Color.green);
        Debug.DrawLine(boids[0].position, boids[0].position + boids[0].separation, Color.magenta);
        Debug.DrawLine(boids[0].position, boids[0].position + boids[0].cohesion, Color.yellow);
        Debug.DrawLine(boids[0].position, boids[0].position + boids[0].obstacle, Color.red);

        if (boidZeroPath != null)
        {
            int cornersLength = boidZeroPath.corners.Length;
            for (int i = 0; i < cornersLength - 1; i++)
                Debug.DrawLine(boidZeroPath.corners[i], boidZeroPath.corners[i + 1], Color.black);
        }    
    }

    public void SetGoal(Vector3 goal)
    {
        //if boid is already on a path ie. navigating, ignore new requests
        if (boidZeroNavigatingTowardGoal)
            return;
        //otherwise, accept the new goal
        boidZeroGoal = goal;
        if (boids == null || boids.Length == 0)
            return;

        //nearest navmesh point to boid zero ie boidZero start point on the navmesh
        Vector3 boidZeroPos = boids[0].position;
        NavMeshHit startHit;
        bool startFound = NavMesh.SamplePosition(boidZeroPos, out startHit, 5.0f, NavMesh.AllAreas);

        //get closest navmesh point to the GOAL, but we are NOT replacing our goal with this position
        NavMeshHit targetHit;
        bool targetFound = NavMesh.SamplePosition(boidZeroGoal, out targetHit, 5.0f, NavMesh.AllAreas);

        if (!startFound || !targetFound)
        {
            Debug.LogWarning("Swarm.SetGoal: either the Start or Target is not on Navmesh. Ignoring SetGoal.");
            return;
        }

        //calculate the path to target, using the sampled positions (ie not directly using the GOAL but using the closest navmesh position to the goal)
        NavMeshPath path = new NavMeshPath();
        bool pathFound = NavMesh.CalculatePath(startHit.position, targetHit.position, NavMesh.AllAreas, path);

        if (pathFound && path.status == NavMeshPathStatus.PathComplete && path.corners != null && path.corners.Length > 0)
        {
            //store the path for the boidZero to follow
            boidZeroPath = path;
            boidZeroNavigatingTowardGoal = true;
            currentCorner = 0;
        }
        else
        {
            Debug.LogWarning("Swarm.SetGoal: Path not found or is incomplete.");
        }
    }
}

