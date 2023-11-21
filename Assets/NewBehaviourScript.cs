using UnityEngine;

public class Example : MonoBehaviour
{
    public Vector3[] vertices;
    public int[] triangles;

   void Start()
    {
        // triangles[0] = 0; //BL
        // triangles[1] = 2; //TL
        // triangles[2] = 1; //BR
        // vertices[0] = new Vector3(0, 0, 0); //BL
        // vertices[1] = new Vector3(1, 0, 0); //BR
        // vertices[2] = new Vector3(0, 0, 1); //TL
        // Mesh mesh = new Mesh();
        // GetComponent<MeshFilter>().mesh = mesh;
        // mesh.Clear();
        // mesh.vertices = vertices;
        // mesh.triangles = triangles;
    }

    void Update()
    {
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        Vector3[] vertices = mesh.vertices;
        Vector3[] normals = mesh.normals;

       for (var i = 0; i < vertices.Length; i++)
        {
            vertices[i] += normals[i] * Mathf.Sin(Time.time);
        }

       mesh.vertices = vertices;
    }

}