using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using System.Linq; // for Select

public class Plane : MonoBehaviour
{
    //-----------------------------//
    //      Mesh parameters        //
    //-----------------------------//
    public static readonly int n_grid = 11;
    public static readonly int n_vertices = n_grid * n_grid;
    public static readonly int n_edges = 2*(n_grid-1)*n_grid + (n_grid-1)*(n_grid-1);
    public Vector3[] vertices;

    //-----------------------------//
    //     For calculations        //
    //-----------------------------//
    public Vector<double> velocity;
    static public Vector<double> q;
    public Vector<double> M;
    public Vector<double> M_inv;
    Matrix<double> global_LHS; // LHS for global step, can be precomputed
    public double h; // time step

    // for constraints
    public List<int> constrained_vertices;

    // Start is called before the first frame update
    void Start()
    {
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        vertices = mesh.vertices;
        h =  Time.fixedDeltaTime;
        q = concatenate_vector_array(vertices);
        velocity = Vector<double>.Build.Dense(3 * n_vertices);

        M = Vector<double>.Build.Dense(3 * n_vertices, 1.0);
        M_inv = Vector<double>.Build.Dense(3 * n_vertices, 1.0);
        
        /////////////////////////////////
        ///      Precomputations      ///
        /////////////////////////////////
        edge_length_constraint[] E = new edge_length_constraint[n_edges];
        for (var i = 0; i < n_edges; i++)
        {
            E[i] = new edge_length_constraint(i);
        }
        Matrix<double> St_S = precompute_w_St_S(E.Select(e => e.Si).ToArray());
        global_LHS = St_S + Matrix<double>.Build.SparseOfDiagonalVector(M_inv) / (h * h);
    }

    Vector<double> concatenate_vector_array(Vector3[] vectors)
    {
        int totalComponents = vectors.Length * 3;
        Vector<double> res = Vector<double>.Build.Dense(totalComponents);
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

    Matrix<double> precompute_w_St_S(Matrix<double>[] S, double[] weights = null)
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
        Vector<double> g = Vector<double>.Build.Dense(3 * n_vertices);
        for (var i = 0; i < n_vertices; i++)
        {
            g[3 * i + 1] = -9.8;
        }
        velocity += h * g;
        q += h * velocity;
        mesh.vertices = to_vector3_array(q);
    }
}
