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
    }

    public Transform boidPrefab;
    public int numberOfBoids = 200;
    public float boidForceScale = 20f;
    public float maxSpeed = 5.0f;
    public float rotationSpeed = 40.0f;
    public float obstacleCheckRadius = 1.0f;
    public float separationWeight = 1.1f;
    public float alignmentWeight = 0.5f;
    public float cohesionWeight = 1f;
    public float goalWeight = 1f;
    public float obstacleWeight = 0.9f;
    public float wanderWeight = 0.3f;
    public float neighbourDistance = 2.0f;
    public float initializationRadius = 1.0f;
    public float initializationForwardRandomRange = 50f;

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

    /// <summary>
    /// Start, this function is called before the first frame
    /// </summary>
    private void Start()
    {
        sqrNeighbourDistance = neighbourDistance * neighbourDistance;
        boidZeroPath = new NavMeshPath();
        InitBoids();
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
            //apparate the object
            Vector3 pos = transform.position + Random.insideUnitSphere * initializationRadius;
            pos.y = Mathf.Clamp(pos.y, worldMinY, worldMaxY);

            Transform t = Instantiate(boidPrefab, pos, Quaternion.identity, transform);
            boidObjects[i] = t;

            //go forward within range - no dest necessary
            Vector2 fwd = Random.insideUnitSphere;
            if (fwd.sqrMagnitude < 0.0001f)
            {
                fwd = Vector3.forward;
            }
            fwd = fwd.normalized;

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

            //orient the object
            if (vel.sqrMagnitude > 0.0001f)
            {
                t.forward = vel.normalized;
            }
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
        }
    }
    
    // NEED TO GO OVER FIXEDUPDATE() AGAIN --- i AM NOT SURE ABOUT THIS PART --- when played, cohesion is too low and separation is too high NEED TO FIX THIS

    /// <summary>
    /// Sim Loop
    /// </summary>
    private void FixedUpdate()
    {
        if (boids == null || boids.Length == 0) return;

        float dt = Time.fixedDeltaTime;

        ResetBoidForces();

        int count = boids.Length;

        // First pass: compute neighbour-based accumulators and obstacles
        for (int i = 0; i < count; i++)
        {
            Vector3 posI = boids[i].position;
            Vector3 fwdI = boids[i].forward;
            Vector3 sepAcc = Vector3.zero;
            Vector3 alignAcc = Vector3.zero;
            Vector3 cohAcc = Vector3.zero;
            int neighborCount = 0;

            // neighbour detection (naive O(n^2) as required)
            for (int j = 0; j < count; j++)
            {
                if (i == j) continue;
                Vector3 posJ = boids[j].position;
                Vector3 toNeighbor = posJ - posI;

                float sqrDist = toNeighbor.sqrMagnitude;
                if (sqrDist > sqrNeighbourDistance) continue;

                // 180 degree FOV check -> dot > 0
                if (Vector3.Dot(toNeighbor.normalized, fwdI) <= 0f) continue;

                // accepted neighbour
                neighborCount++;

                float dist = Mathf.Sqrt(sqrDist);
                // separation: away from neighbor, stronger when closer (use inverse distance weighting)
                if (dist > 0.0001f)
                    sepAcc += (posI - posJ).normalized * (1f - (dist / neighbourDistance));

                // alignment: sum neighbour velocities
                alignAcc += boids[j].velocity;

                // cohesion: sum neighbour positions (center of mass later)
                cohAcc += posJ;
            }

            // finalize alignment/cohesion/separation vectors (normalize as required)
            Vector3 separationVec = Vector3.zero;
            Vector3 alignmentVec = Vector3.zero;
            Vector3 cohesionVec = Vector3.zero;

            if (neighborCount > 0)
            {
                // Separation: average and normalize
                separationVec = (sepAcc / neighborCount);
                if (separationVec.sqrMagnitude > 0.0001f)
                    separationVec = separationVec.normalized;

                // Alignment: average velocity -> normalized
                Vector3 avgVel = alignAcc / neighborCount;
                if (avgVel.sqrMagnitude > 0.0001f)
                    alignmentVec = avgVel.normalized;

                // Cohesion: center of mass - position
                Vector3 center = (cohAcc / neighborCount);
                Vector3 toCenter = center - posI;
                if (toCenter.sqrMagnitude > 0.0001f)
                    cohesionVec = toCenter / neighbourDistance;
                    //cohesionVec = toCenter.normalized;
            }
            else
            {
                //no neighbours but assigning wander to separation/alignment/cohesion did not function as expected. Try zeroing them for second pass prep to "wander"
                separationVec = Vector3.zero;
                alignmentVec = Vector3.zero;
                cohesionVec = Vector3.zero;
                /*
                // No neighbours: wander rule is boid's current velocity (normalized)
                Vector3 vel = boids[i].velocity;
                if (vel.sqrMagnitude > 0.0001f)
                    separationVec = alignmentVec = cohesionVec = vel.normalized;
                else
                    separationVec = alignmentVec = cohesionVec = Vector3.forward; // fallback*/
            }

            boids[i].separation = separationVec;
            boids[i].alignment = alignmentVec;
            boids[i].cohesion = cohesionVec;

            // Obstacles: sum of normals of nearby colliders found with OverlapSphere
            Vector3 obsAcc = Vector3.zero;
            Collider[] hitCols = Physics.OverlapSphere(posI, obstacleCheckRadius);
            if (hitCols != null && hitCols.Length > 0)
            {
                for (int h = 0; h < hitCols.Length; h++)
                {
                    Collider col = hitCols[h];
                    // Ignore own collider if boid has one; but likely boids have no colliders
                    Vector3 closest = col.ClosestPoint(posI);
                    Vector3 fromClosest = posI - closest;
                    float mag = fromClosest.magnitude;
                    if (mag > 0.0001f)
                    {
                        obsAcc += fromClosest.normalized; // pushing away
                    }
                }
            }

            // World bounds as obstacles with a buffer zone added

            float buffer = 0.5f;
            Vector3 worldObs = Vector3.zero;

            if (posI.x > worldMaxX - buffer) worldObs += new Vector3(-1f, 0f, 0f);
            else if (posI.x < worldMinX + buffer) worldObs += new Vector3(1f, 0f, 0f);

            if (posI.z > worldMaxZ - buffer) worldObs += new Vector3(0f, 0f, -1f);
            else if (posI.z < worldMinZ + buffer) worldObs += new Vector3(0f, 0f, 1f);

            if (posI.y > worldMaxY - buffer) worldObs += new Vector3(0f, -1f, 0f);
            else if (posI.y < worldMinY + buffer) worldObs += new Vector3(0f, 1f, 0f);

            if (worldObs.sqrMagnitude > 0.0001f) obsAcc += worldObs.normalized;

            if (obsAcc.sqrMagnitude > 0.0001f) boids[i].obstacle = obsAcc.normalized;
            else boids[i].obstacle = Vector3.zero;
        }

        // Second pass: compute steering, apply forces and integrate (symplectic Euler)
        for (int i = 0; i < count; i++)
        {
            Vector3 totalForce = Vector3.zero;
            Vector3 vel = boids[i].velocity;

            // helper to compute steering contribution for a rule vector
            System.Func<Vector3, float, Vector3> RuleSteer = (ruleVec, weight) =>
            {
                if (ruleVec.sqrMagnitude < 0.000001f) return Vector3.zero;
                // desired movement in direction of ruleVec at maxSpeed
                Vector3 desired = ruleVec.normalized * maxSpeed;
                Vector3 steer = desired - vel; // steering force
                if (steer.sqrMagnitude > 0.000001f)
                {
                    Vector3 s = steer.normalized; // normalize steering so weights scale correctly
                    return s * weight * boidForceScale;
                }
                return Vector3.zero;
            };

            // separation
            totalForce += RuleSteer(boids[i].separation, separationWeight);

            // alignment
            totalForce += RuleSteer(boids[i].alignment, alignmentWeight);

            // cohesion
            totalForce += RuleSteer(boids[i].cohesion, cohesionWeight);

            // obstacle
            totalForce += RuleSteer(boids[i].obstacle, obstacleWeight);

            // goal following for boid 0 (if navigating)
            if (i == 0 && boidZeroNavigatingTowardGoal && boidZeroPath != null && boidZeroPath.status == NavMeshPathStatus.PathComplete && boidZeroPath.corners.Length > 1)
            {
                // ensure currentCorner in range
                Vector3 targetCorner = boidZeroPath.corners[currentCorner];
                Vector3 toCorner = (targetCorner - boids[i].position);
                Vector3 goalVec = Vector3.zero;
                if (toCorner.sqrMagnitude > 0.0001f)
                    goalVec = toCorner.normalized;

                totalForce += RuleSteer(goalVec, goalWeight);

                // check progress to corner: sample nearest point on navmesh to boid position and compare distance to corner
                NavMeshHit hit;
                if (NavMesh.SamplePosition(boids[i].position, out hit, 1.0f, NavMesh.AllAreas))
                {
                    float distToCorner = (hit.position - targetCorner).magnitude;
                    if (distToCorner <= 1.0f)
                    {
                        // go to next corner (if any)
                        if (currentCorner < boidZeroPath.corners.Length - 1)
                            currentCorner++;
                        else
                        {
                            // reached final corner
                            boidZeroPath.ClearCorners();
                            boidZeroNavigatingTowardGoal = false;
                            currentCorner = 0;
                        }
                    }
                }
            }

            // wander when neighborCount == 0 ie we have no neighbours
            if (boids[i].alignment == Vector3.zero && boids[i].cohesion == Vector3.zero && boids[i].separation == Vector3.zero)
            {
                Vector3 wanderVec = boids[i].velocity.normalized;
                totalForce += RuleSteer(wanderVec, wanderWeight);
            }

            // store current total force (for debugging/visualization)
            boids[i].currentTotalForce = totalForce;

            // Integrate using Symplectic Euler:
            // a = totalForce (already scaled by boidForceScale inside RuleSteer)
            Vector3 acceleration = totalForce;
            vel += acceleration * dt;

            // clamp speed
            if (vel.magnitude > maxSpeed)
                vel = vel.normalized * maxSpeed;

            // position update
            Vector3 pos = boids[i].position + vel * dt;

            // write back
            boids[i].velocity = vel;
            if (vel.sqrMagnitude > 0.0001f)
                boids[i].forward = vel.normalized;
            boids[i].position = pos;

            // update visual transform
            if (boidObjects[i] != null)
            {
                boidObjects[i].position = boids[i].position;
                if (boids[i].forward.sqrMagnitude > 0.0001f)
                    boidObjects[i].forward = boids[i].forward;
            }
        }
    }


    private void Update()
    {
        /* Render information for boidzero, useful for debugging forces and path planning
        int boidCount = boids.Length;
        for (int i = 1; i < boidCount; i++)
        {
            Vector3 boidNeighbourVec = boids[i].position - boids[0].position;
            if (boidNeighbourVec.sqrMagnitude < sqrNeighbourDistance &&
                    Vector3.Dot(boidNeighbourVec, boids[0].forward) > 0f)
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
        */
    }


    public void SetGoal(Vector3 goal)
    {
        //if boid is already on a path ie. navigating, ignore new requests
        if (boidZeroNavigatingTowardGoal)
        {
            return;
        }
        //otherwise, accept the new goal
        boidZeroGoal = goal;

        //nearest navmesh point to boid zero
        if (boids == null || boids.Length == 0)
        {
            return;
        }

        Vector3 boidZeroPos = boids[0].position;
        NavMeshHit startHit;
        bool startFound = NavMesh.SamplePosition(boidZeroPos, out startHit, 5.0f, NavMesh.AllAreas);
        NavMeshHit targetHit;
        bool targetFound = NavMesh.SamplePosition(boidZeroGoal, out targetHit, 5.0f, NavMesh.AllAreas);

        if (!startFound || !targetFound)
        {
            Debug.LogWarning("Swarm.SetGoal: either the Start or Target is not on Navmesh. Ignoring SetGoal.");
            return;
        }

        //calculate the path to target
        NavMeshPath path = new NavMeshPath();
        bool pathFound = NavMesh.CalculatePath(startHit.position, targetHit.position, NavMesh.AllAreas, path);

        if (pathFound && path.status == NavMeshPathStatus.PathComplete && path.corners != null && path.corners.Length > 0)
        {
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

