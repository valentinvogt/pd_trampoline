using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

public class Plane : MonoBehaviour
{
    static readonly int n_grid = 11;
    public int n_vertices = n_grid * n_grid;
    public int n_edges = 220;
    public Vector3[] vertices;
    public Vector<double> velocity;
    public Vector<double> q;

    // for constraints
    public List<int> constrained_vertices;
    public double[] edge_rest_lengths;
    // Start is called before the first frame update
    void Start()
    {
        /////////////////////////////////
        ///    Mesh Initialization    ///
        /////////////////////////////////
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        vertices = mesh.vertices;
        q = concatenate_vector_array(vertices);

        velocity = Vector<double>.Build.Dense(3 * n_vertices);

        /////////////////////////////////
        ///      Precomputations      ///
        /////////////////////////////////
        
        Matrix<double>[] S_edge_length = build_S_edge_length();
        Matrix<double> St_S = precompute_w_St_S(S_edge_length);
        edge_rest_lengths = new double[n_edges];
        for (var i = 0; i < n_edges; i++)
        {
            edge_rest_lengths[i] = get_edge_length(i, vertices);
        }
    }

    Vector<double> concatenate_vector_array(Vector3[] vectors)
    {
        int totalComponents = vectors.Length * 3;
        Vector<double> res = new DenseVector(totalComponents);
        for (var i = 0; i < vectors.Length; i++)
        {
            res[3 * i] = vectors[i][0];
            res[3 * i + 1] = vectors[i][1];
            res[3 * i + 2] = vectors[i][2];
        }
        return res;
    }

    Vector3[] to_vector3_array(Vector<double> q)
    {
        Vector3[] res = new Vector3[q.Count / 3];
        for (var i = 0; i < res.Length; i++)
        {
            res[i] = new Vector3((float)q[3 * i], (float)q[3 * i + 1], (float)q[3 * i + 2]);
        }
        return res;
    }
    
    int get_vertex_index(int i, int j)
    {
        return i * n_grid + j;
    }

    double get_edge_length(int edge_idx, Vector3[] vertices)
    {
        int[] vertices_of_edge = get_vertices_of_edge(edge_idx);
        int v1 = vertices_of_edge[0];
        int v2 = vertices_of_edge[1];
        return (vertices[v1] - vertices[v2]).magnitude;
    }

    int[] get_vertices_of_edge(int edge_idx)
    {
        int n = n_grid;
        int edgesPerGridRow = (n - 1) * 2 + n;
        int horizontalEdges = (n - 1);
        int verticalEdges = n;
        int diagonalEdges = (n - 1);

        int i = edge_idx;
        int row = i / edgesPerGridRow;
        int col = i % edgesPerGridRow;

        int[] res = new int[2];

        if (col < horizontalEdges)
        {
            int startingIdx = row * n + col; // TO THE RIGHT
            res = new int[] { startingIdx, startingIdx + 1 };
        }
        else if (col < horizontalEdges + verticalEdges)
        {
            int startingIdx = row * n + col - horizontalEdges; // DOWNWARDS
            res = new int[] { startingIdx, startingIdx + n };
        }
        else
        {
            int startingIdx = (row + 1) * n + col - horizontalEdges - verticalEdges; // DIAGONAL
            res = new int[] { startingIdx, startingIdx - n + 1 };
        }
        return res;
    }

    SparseMatrix[] build_S_edge_length()
    {
        // var M = SparseMatrix.Build;
        SparseMatrix[] S_edge_length = new SparseMatrix[n_edges];
        for (var i = 0; i < n_edges; i++)
        {
            Tuple<int, int, double>[] entries = new Tuple<int, int, double>[6];
            int[] vertices = get_vertices_of_edge(i);
            int v1 = vertices[0];
            int v2 = vertices[1];
            entries[0] = new Tuple<int, int, double>(0, 3 * v1, 1);
            entries[1] = new Tuple<int, int, double>(1, 3 * v1 + 1, 1);
            entries[2] = new Tuple<int, int, double>(2, 3 * v1 + 2, 1);

            entries[3] = new Tuple<int, int, double>(0, 3 * v2, -1);
            entries[4] = new Tuple<int, int, double>(1, 3 * v2 + 1, -1);
            entries[5] = new Tuple<int, int, double>(2, 3 * v2 + 2, -1);

            SparseMatrix S_i = SparseMatrix.OfIndexed(3, n_vertices * 3, entries);

            S_edge_length[i] = S_i;
        }
        return S_edge_length;
    }

    Vector<double> project_edge_length_constraint(Vector<double> q, Matrix<double> Si, double Li)
    {
        // var V = Vector<double>.Build;
        Vector<double> pi = Si * q;
        Vector<double> pi_hat = Li * pi / pi.Norm(2);
        return pi_hat;
    }

    Matrix<double> precompute_w_St_S( Matrix<double>[] S, double[] weights = null)
    {
        var M = Matrix<double>.Build;

        if (weights == null)
        {
            weights = new double[S.Length];
            for (var i = 0; i < S.Length; i++)
            {
                weights[i] = 1;
            }
        }

        Matrix<double> St_S = M.SparseOfMatrix(weights[0] * S[0].Transpose() * S[0]);
        for (var i = 1; i < S.Length; i++)
        {
            St_S += M.SparseOfMatrix(weights[i] * S[i].Transpose() * S[i]);
        }
        return St_S;
    }

    // Update is called once per frame
    void Update()
    {
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        constrained_vertices = new List<int>();
        for (var i = 0; i < n_grid; i++)
        {
            constrained_vertices.Add(i);
        }

        /////////////////////////////////
        ///      Gravity Example      ///
        /////////////////////////////////
        double h = Time.deltaTime;
        Vector<double> g = Vector.Build.Dense(3 * n_vertices);
        for (var i = 0; i < n_vertices; i++)
        {
            g[3 * i + 1] = -9.8;
        }
        h = Time.deltaTime;
        velocity += h * g;
        q += h * velocity;
        mesh.vertices = to_vector3_array(q);
    }
}
