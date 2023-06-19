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

    private VertexPath path;
    private float lastTime;


    // Start is called before the first frame update
    public void generateNew()
    {
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

        Mesh PathMesh = CreateRoadMesh(new VertexPath(bezierPath, transform));

        //get mesh collider
        MeshCollider meshCollider = roadMeshHolder.GetComponent<MeshCollider>();
        meshCollider.sharedMesh = CreateRoadMesh(new VertexPath(bezierPath, transform));

        //get mesh filter
        MeshFilter meshFilter = roadMeshHolder.GetComponent<MeshFilter>();
        meshFilter.mesh = meshCollider.sharedMesh;
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

            Mesh mesh = new Mesh ();
            mesh.Clear ();
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
