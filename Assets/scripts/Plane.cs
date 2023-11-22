using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra.Double;

public class Plane : MonoBehaviour
{
    // Start is called before the first frame update
    static int n_grid = 11;
    public int n_vertices = n_grid * n_grid;
    public int n_edges = 220;
    public Vector3[] vertices;
    public Vector3[] velocity;

    public List<int> constrained_vertices;
    public float[] edge_rest_lengths;
    void Start()
    {
        /////////////////////////////////
        ///    Mesh Initialization    ///
        /////////////////////////////////
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        vertices = mesh.vertices;
        velocity = new Vector3[vertices.Length];
        for (var i = 0; i < vertices.Length; i++)
        {
            velocity[i] = new Vector3(0, 0, 0);
        }

        /////////////////////////////////
        ///      Precomputations      ///
        SparseMatrix[] S_edge_length = build_S_edge_length();
        SparseMatrix St_S = precompute_w_St_S(S_edge_length);
        edge_rest_lengths = new float[n_edges];
        for (var i = 0; i < n_edges; i++)
        {
            edge_rest_lengths[i] = get_edge_length(i, vertices);
        }
    }

    SparseMatrix precompute_w_St_S(SparseMatrix[] S, float[] weights = null)
    {
        if (weights == null)
        {
            weights = new float[S.Length];
            for (var i = 0; i < S.Length; i++)
            {
                weights[i] = 1;
            }
        }

        SparseMatrix St_S = SparseMatrix.OfMatrix(weights[0] * S[0].Transpose() * S[0]);
        for (var i = 1; i < S.Length; i++)
        {
            St_S += SparseMatrix.OfMatrix(weights[i] * S[i].Transpose() * S[i]);
        }
        return St_S;
    }

    // Update is called once per frame
    void Update()
    {
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        Vector3[] vertices = mesh.vertices;
        constrained_vertices = new List<int>();
        for (var i = 0; i < n_grid; i++)
        {
            constrained_vertices.Add(i);
        }

        /////////////////////////////////
        ///      Gravity Example      ///
        /////////////////////////////////
        for (var i = 0; i < vertices.Length; i++)
        {
            if (constrained_vertices.Contains(i))
            {
                velocity[i] = new Vector3(0, 0, 0);
                vertices[i][1] = 0;
            }
            velocity[i] += new Vector3(0, -9.8f, 0) * Time.deltaTime;
            vertices[i] += velocity[i] * Time.deltaTime;
        }

        mesh.vertices = vertices;
    }
}
