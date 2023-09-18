using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using PathCreation;
using Random=UnityEngine.Random;

public class RacetrackGenerator : MonoBehaviour
{
    public PathCreator pathCreator;
    public GameObject roadMeshHolder;
    public PathCreation.Examples.RoadMeshCreator roadMeshCreator;
    public PathCreator[] racetracks;

    private VertexPath path;
    private float lastTime;
    private Mesh pathMesh;
    private MeshFilter roadMeshFilter;
    private MeshCollider pathCollider;

    void Start(){
        roadMeshFilter = roadMeshHolder.GetComponent<MeshFilter>();
        pathCollider = roadMeshHolder.GetComponent<MeshCollider>();
    }

    public void pickRandom(){
        int index = Random.Range(0, racetracks.Length);

        BezierPath bezierPath = racetracks[index].bezierPath;
        pathCreator.bezierPath = bezierPath;
        pathCreator.TriggerPathUpdate();

        //get mesh collider
        pathCollider.sharedMesh = CreateRoadMesh(new VertexPath(bezierPath, transform));

        //get mesh filter
        roadMeshFilter.mesh = pathCollider.sharedMesh;
    }

// public List<Vector3> GenerateWavySection(float length, int pointCount, float amplitude = 0.5f, float frequency = 1.0f, Vector3 direction = default, Vector3 startAt = default)
// {
//     if(direction == default)
//         direction = Vector3.right; 

//     List<Vector3> wavySection = new List<Vector3>();
//     float dx = length / pointCount;
//     for (int i = 0; i <= pointCount; i++)
//     {
//         float x = i * dx;
//         float z = amplitude * Mathf.Sin(frequency * x);
//         Vector3 point = new Vector3(x, 0.0f, z);

//         // Rotate point to match direction
//         Quaternion rotation = Quaternion.FromToRotation(Vector3.right, direction);
//         point = rotation * point;
        
//         // Offset the point to start at the correct position
//         point += startAt;

//         wavySection.Add(point);
//     }
//     return wavySection;
// }

    public void generateRandomTrack(){
        // Clear shared mesh and mesh filter from memory
        if (pathCollider.sharedMesh != null)
        {
            Destroy(pathCollider.sharedMesh);
        }
        if (roadMeshFilter.mesh != null)
        {
            Destroy(roadMeshFilter.mesh);
        }
        bool trackFound = false;
        while(!trackFound){

            List<Vector3> racetrack = new List<Vector3>();

            Vector3 currentPoint = new Vector3(0.0f, 0.0f, 0.0f);
            Vector3 currentDirection = Vector3.right;
            int direction = Random.Range(0, 2) * 2 - 1;
            int thisDirCount = 0;

            int curveCount = 2;

            for(int i = 0; i < curveCount; i++)
            {
                //direction = 1;
                List<Vector3> curve = generateCurve(currentPoint, direction);

                float directionAngle = Mathf.Atan2(currentDirection.z, currentDirection.x) + Mathf.PI;
                // if(direction == -1){
                //     directionAngle += Mathf.PI;
                // }

                //rotate all points on curve around currentPoint by directionAngle
                for(int j = 0; j < curve.Count; j++)
                {
                    Vector3 point = curve[j];
                    Vector3 rotatedPoint = new Vector3();
                    rotatedPoint.x = Mathf.Cos(directionAngle) * (point.x - currentPoint.x) - Mathf.Sin(directionAngle) * (point.z - currentPoint.z) + currentPoint.x;
                    rotatedPoint.z = Mathf.Sin(directionAngle) * (point.x - currentPoint.x) + Mathf.Cos(directionAngle) * (point.z - currentPoint.z) + currentPoint.z;
                    rotatedPoint.y = point.y;
                    curve[j] = rotatedPoint;
                }
                currentPoint = curve[curve.Count - 1];
                Vector3 currDirForw = curve[curve.Count - 1] - curve[curve.Count - 2];
                //currDir perpendicular to currDirForw
                currentDirection = new Vector3(currDirForw.z, 0.0f, -currDirForw.x);

                curve.RemoveAt(curve.Count - 1);

                // currentDirection = -currentDirection;
                if(i != 0){
                    racetrack.AddRange(curve);
                }

                // //generate connector
                float length = -1.0f;
                if(i == curveCount - 1){
                    length = 30.0f;
                }
                List<Vector3> connector = generateCurveConnector(currentPoint, currDirForw, length);
                currentPoint = connector[connector.Count - 1];

                if(i != curveCount - 1){
                    connector.RemoveAt(connector.Count - 1);
                }

                racetrack.AddRange(connector);
                direction *= -1;
            }

            BezierPath bezierPath = new BezierPath(racetrack, false, PathSpace.xyz);
            try{
                pathCollider.sharedMesh = CreateRoadMesh(new VertexPath(bezierPath, transform));
                trackFound = true;

                pathCreator.bezierPath = bezierPath;
                pathCreator.TriggerPathUpdate();

                //get mesh collider

                //get mesh filter
                roadMeshFilter.mesh = pathCollider.sharedMesh;
            }
            catch{
                Debug.Log("Track not found");
            }
        }
    }

    public List<Vector3> generateCurveConnector(Vector3 startPos, Vector3 direction, float lengthOverride = -1.0f){
        direction = direction.normalized;
        float length = Random.Range(40.0f, 50.0f);
        if(lengthOverride != -1.0f){
            length = lengthOverride;
        }
        float perpOffsetCenterPoint = Random.Range(-length/22.0f, length/22.0f);

        List<Vector3> connector = new List<Vector3>();

        Vector3 centerPoint = startPos + direction * length / 2.0f;
        centerPoint += new Vector3(-direction.z, 0.0f, direction.x) * perpOffsetCenterPoint;
        Vector3 lastPoint = startPos + direction * length;

        connector.Add(startPos);
        connector.Add(centerPoint);
        connector.Add(lastPoint);

        return connector;
    }

    public List<Vector3> generateCurve(Vector3 startPos, int direction = 1)
    {
        // Generate a random angle between 65 and 110 degrees which determines the angle between the curve start tangent and the curve end tangent
        float curveRadius = Random.Range(7.0f, 13.0f);
        float curveAngle = Random.Range(80f, 130f) * Mathf.Deg2Rad; // Convert to radians
        // float angle = Mathf.PI - curveAngle; // Angle between the curve start tangent and the curve end tangent
        // angle = 160.0f * Mathf.Deg2Rad;
        float angle = curveAngle;

        int curvePointCount = 20; // Number of points in the curve
        float curvePointAngle = angle / curvePointCount; // Angle between each point in the curve
        List<Vector2> curve = new List<Vector2>(); // List of points in the curve

       // Compute the center using the perpendicular direction.
        Vector2 center = new Vector2(startPos.x, startPos.z) + curveRadius * direction * new Vector2(1.0f, 0.0f);

         float initialAngle = (direction == 1) ? Mathf.PI : 0;

        // Generate the curve points
        for (int i = 0; i <= curvePointCount; i++)
        {
            float x = center.x + curveRadius * Mathf.Cos(initialAngle + direction * i * curvePointAngle);
            float y = center.y + curveRadius * Mathf.Sin(initialAngle + direction * i * curvePointAngle);
            curve.Add(new Vector2(x, y));
        }

        // Convert the curve's points to 3D space
        List<Vector3> curve3d = new List<Vector3>();
        for (int i = 0; i < curve.Count; i++)
        {
            curve3d.Add(new Vector3(curve[i].x, 0.0f, curve[i].y));
        }

        return curve3d;
    }

    public void generateRandomCircle()
    {
        //clear shared mesh and mesh filter from mem
        if (pathCollider.sharedMesh != null)
        {
            Destroy(pathCollider.sharedMesh);
        }
        if (roadMeshFilter.mesh != null)
        {
            Destroy(roadMeshFilter.mesh);
        }

        Random.InitState(System.DateTime.Now.Millisecond);

        float radius = Random.Range(7.0f, 13.0f);
        List<Vector2> racetrack = GenerateCircle(radius, 20);

        //convert to 3d
        List<Vector3> racetrack3d = new List<Vector3>();
        for (int i = 0; i < racetrack.Count; i++)
        {
            racetrack3d.Add(new Vector3(racetrack[i].x, 0.0f, racetrack[i].y));
        }

        BezierPath bezierPath = new BezierPath(racetrack3d, true, PathSpace.xyz);
        pathCreator.bezierPath = bezierPath;
        pathCreator.TriggerPathUpdate();

        //get mesh collider
        pathCollider.sharedMesh = CreateRoadMesh(new VertexPath(bezierPath, transform));

        //get mesh filter
        roadMeshFilter.mesh = pathCollider.sharedMesh;
    }

    // Start is called before the first frame update
    public void generateNew()
    {
        //clear shared mesh and mesh filter from mem
        if (pathCollider.sharedMesh != null)
        {
            Destroy(pathCollider.sharedMesh);
        }
        if (roadMeshFilter.mesh != null)
        {
            Destroy(roadMeshFilter.mesh);
        }

        Random.InitState(System.DateTime.Now.Millisecond);
        List<Vector2> racetrack = GenerateTrack(70.0f, 50.0f, 20, 70.0f, 360.0f, 5.0f);

        //convert to 3d
        List<Vector3> racetrack3d = new List<Vector3>();
        for (int i = 0; i < racetrack.Count; i++)
        {
            racetrack3d.Add(new Vector3(racetrack[i].x, 0.0f, racetrack[i].y));
        }

        BezierPath bezierPath = new BezierPath(racetrack3d, true, PathSpace.xyz);
        pathCreator.bezierPath = bezierPath;
        pathCreator.TriggerPathUpdate();

        //get mesh collider
        pathCollider.sharedMesh = CreateRoadMesh(new VertexPath(bezierPath, transform));

        //get mesh filter
        roadMeshFilter.mesh = pathCollider.sharedMesh;
    }

    public List<Vector2> GenerateCircle(float radius, int numPoints)
    {
        Vector2 center = new Vector2(33.0f, 26.0f);
        List<Vector2> points = new List<Vector2>();

        for (int i = 0; i < numPoints; i++)
        {
            float angle = i * Mathf.PI * 2.0f / numPoints;
            float x = center.x + radius * Mathf.Cos(angle);
            float y = center.y + radius * Mathf.Sin(angle);
            points.Add(new Vector2(x, y));
        }

        return points;
    }

    public List<Vector2> GenerateTrack(float width, float height, int numPoints, float minAngle, float maxAngle, float minDistance)
    {
        List<Vector2> randomPoints = GenerateRandomPoints(width, height, numPoints);
        //log points
        // for (int i = 0; i < randomPoints.Count; i++)
        // {
        //     Debug.Log(randomPoints[i]);
        // }
        List<Vector2> convexHull = GetConvexHull(randomPoints);
        //log points
        // for (int i = 0; i < convexHull.Count; i++)
        // {
        //     Debug.Log(convexHull[i]);
        // }
        List<Vector2> displacedPoints = DisplaceMidpoints(convexHull);
        List<Vector2> separatedPoints = PushApartPoints(displacedPoints, minAngle, maxAngle, minDistance);

        return separatedPoints;
    }

    private List<Vector2> GenerateRandomPoints(float width, float height, int numPoints)
    {
        List<Vector2> points = new List<Vector2>();

        for (int i = 0; i < numPoints; i++)
        {
            float x = UnityEngine.Random.Range(0f, width);
            float y = UnityEngine.Random.Range(0f, height);
            points.Add(new Vector2(x, y));
        }

        return points;
    }

    private List<Vector2> GetConvexHull(List<Vector2> points)
    {
        // Implementation of the Graham's scan algorithm for computing the convex hull

        if (points.Count < 3)
            return new List<Vector2>(points);

        List<Vector2> hull = new List<Vector2>();

        // Find the point with the lowest y-coordinate (leftmost point)
        Vector2 startPoint = points[0];
        foreach (var point in points)
        {
            if (point.y < startPoint.y || (point.y == startPoint.y && point.x < startPoint.x))
                startPoint = point;
        }

        // Sort the points by their polar angle with respect to the start point
        points.Sort((a, b) =>
        {
            float angleA = GetPolarAngle(startPoint, a);
            float angleB = GetPolarAngle(startPoint, b);
            if (angleA < angleB)
                return -1;
            if (angleA > angleB)
                return 1;
            return Distance(startPoint, a).CompareTo(Distance(startPoint, b));
        });

        hull.Add(points[0]);
        hull.Add(points[1]);

        for (int i = 2; i < points.Count; i++)
        {
            int m = hull.Count;
            while (m >= 2 && CrossProduct(hull[m - 2], hull[m - 1], points[i]) <= 0)
            {
                hull.RemoveAt(m - 1);
                m--;
            }
            hull.Add(points[i]);
        }

        return hull;
    }

    private float GetPolarAngle(Vector2 startPoint, Vector2 point)
    {
        return Mathf.Atan2(point.y - startPoint.y, point.x - startPoint.x);
    }

    private float CrossProduct(Vector2 a, Vector2 b, Vector2 c)
    {
        return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    }

    private float Distance(Vector2 a, Vector2 b)
    {
        float dx = b.x - a.x;
        float dy = b.y - a.y;
        return Mathf.Sqrt(dx * dx + dy * dy);
    }

    private List<Vector2> DisplaceMidpoints(List<Vector2> points)
    {
        List<Vector2> displacedPoints = new List<Vector2>();

        for (int i = 0; i < points.Count; i++)
        {
            Vector2 curr = points[i];
            Vector2 next = points[(i + 1) % points.Count];
            Vector2 midpoint = Vector2.Lerp(curr, next, 0.5f);

            float displacementX = UnityEngine.Random.Range(-1f, 1f);
            float displacementY = UnityEngine.Random.Range(-1f, 1f);

            Vector2 displacedPoint = new Vector2(midpoint.x + displacementX, midpoint.y + displacementY);
            displacedPoints.Add(displacedPoint);
        }

        return displacedPoints;
    }

    private List<Vector2> PushApartPoints(List<Vector2> points, float minAngle, float maxAngle, float minDistance)
    {
        List<Vector2> separatedPoints = new List<Vector2>(points);

        for (int i = 0; i < separatedPoints.Count; i++)
        {
            Vector2 prev = separatedPoints[i];
            Vector2 curr = separatedPoints[(i + 1) % separatedPoints.Count];
            Vector2 next = separatedPoints[(i + 2) % separatedPoints.Count];

            float angle = AngleBetween(prev, curr, next);
            float distance = Distance(curr, next);

            if (angle < minAngle && angle > maxAngle)
            {
                Vector2 newNext = DisplacePoint(next, minAngle - angle, minDistance);
                separatedPoints[(i + 2) % separatedPoints.Count] = newNext;
            }

            if (distance < minDistance)
            {
                Vector2 newCurr = DisplacePoint(curr, minAngle, minDistance - distance);
                separatedPoints[(i + 1) % separatedPoints.Count] = newCurr;
            }
        }

        return separatedPoints;
    }

    private Vector2 DisplacePoint(Vector2 point, float angle, float distance)
    {
        float dx = Mathf.Cos(angle) * distance;
        float dy = Mathf.Sin(angle) * distance;
        return new Vector2(point.x + dx, point.y + dy);
    }

    private float AngleBetween(Vector2 a, Vector2 b, Vector2 c)
    {
        Vector2 ab = b - a;
        Vector2 ac = c - a;
        return Vector2.Angle(ab, ac) * Mathf.Deg2Rad;
    }

    Mesh CreateRoadMesh (VertexPath path) {
            float roadWidth = roadMeshCreator.roadWidth;
            bool flattenSurface = roadMeshCreator.flattenSurface;
            float thickness = roadMeshCreator.thickness;

            Vector3[] verts = new Vector3[path.NumPoints * 8];
            Vector2[] uvs = new Vector2[verts.Length];
            Vector3[] normals = new Vector3[verts.Length];


            int numTris = 2 * (path.NumPoints - 1) + ((path.isClosedLoop) ? 2 : 0);
            int[] roadTriangles = new int[numTris * 3];
            int[] underRoadTriangles = new int[numTris * 3];
            int[] sideOfRoadTriangles = new int[numTris * 2 * 3];

            int vertIndex = 0;
            int triIndex = 0;

            // Vertices for the top of the road are layed out:
            // 0  1
            // 8  9
            // and so on... So the triangle map 0,8,1 for example, defines a triangle from top left to bottom left to bottom right.
            int[] triangleMap = { 0, 8, 1, 1, 8, 9 };
            int[] sidesTriangleMap = { 4, 6, 14, 12, 4, 14, 5, 15, 7, 13, 15, 5 };

            bool usePathNormals = !(path.space == PathSpace.xyz && flattenSurface);

            for (int i = 0; i < path.NumPoints; i++) {
                Vector3 localUp = (usePathNormals) ? Vector3.Cross (path.GetTangent (i), path.GetNormal (i)) : path.up;
                Vector3 localRight = (usePathNormals) ? path.GetNormal (i) : Vector3.Cross (localUp, path.GetTangent (i));

                // Find position to left and right of current path vertex
                Vector3 vertSideA = path.GetPoint (i) - localRight * Mathf.Abs (roadWidth);
                Vector3 vertSideB = path.GetPoint (i) + localRight * Mathf.Abs (roadWidth);


                // Add top of road vertices
                verts[vertIndex + 0] = vertSideA;
                verts[vertIndex + 1] = vertSideB;

                // Add bottom of road vertices
                verts[vertIndex + 2] = vertSideA - localUp * thickness;
                verts[vertIndex + 3] = vertSideB - localUp * thickness;

                // Duplicate vertices to get flat shading for sides of road
                verts[vertIndex + 4] = verts[vertIndex + 0];
                verts[vertIndex + 5] = verts[vertIndex + 1];
                verts[vertIndex + 6] = verts[vertIndex + 2];
                verts[vertIndex + 7] = verts[vertIndex + 3];

                // Set uv on y axis to path time (0 at start of path, up to 1 at end of path)
                uvs[vertIndex + 0] = new Vector2 (0, path.times[i]);
                uvs[vertIndex + 1] = new Vector2 (1, path.times[i]);

                // Top of road normals
                normals[vertIndex + 0] = localUp;
                normals[vertIndex + 1] = localUp;
                // Bottom of road normals
                normals[vertIndex + 2] = -localUp;
                normals[vertIndex + 3] = -localUp;
                // Sides of road normals
                normals[vertIndex + 4] = -localRight;
                normals[vertIndex + 5] = localRight;
                normals[vertIndex + 6] = -localRight;
                normals[vertIndex + 7] = localRight;

                // Set triangle indices
                if (i < path.NumPoints - 1 || path.isClosedLoop) {
                    for (int j = 0; j < triangleMap.Length; j++) {
                        roadTriangles[triIndex + j] = (vertIndex + triangleMap[j]) % verts.Length;
                        // reverse triangle map for under road so that triangles wind the other way and are visible from underneath
                        underRoadTriangles[triIndex + j] = (vertIndex + triangleMap[triangleMap.Length - 1 - j] + 2) % verts.Length;
                    }
                    for (int j = 0; j < sidesTriangleMap.Length; j++) {
                        sideOfRoadTriangles[triIndex * 2 + j] = (vertIndex + sidesTriangleMap[j]) % verts.Length;
                    }

                }

                vertIndex += 8;
                triIndex += 6;
            }
            foreach (var v in verts) {
                if (!float.IsFinite(v.x) || !float.IsFinite(v.y) || !float.IsFinite(v.z)) {
                    throw new System.Exception("Non-finite vertex detected");
                }
            }

            Mesh mesh = new Mesh ();
            mesh.Clear();
            mesh.vertices = verts;
            mesh.uv = uvs;
            mesh.normals = normals;
            mesh.subMeshCount = 3;
            mesh.SetTriangles (roadTriangles, 0);
            mesh.SetTriangles (underRoadTriangles, 1);
            mesh.SetTriangles (sideOfRoadTriangles, 2);
            mesh.RecalculateBounds ();
            return mesh;
        }
}
