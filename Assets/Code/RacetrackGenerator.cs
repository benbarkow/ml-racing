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

        float radius = Random.Range(10.0f, 10.0f);
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
        pathCreator.TriggerPathUpdate();
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
