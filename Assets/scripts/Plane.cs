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
    public static readonly int n_edges = 2 * (n_grid - 1) * n_grid + (n_grid - 1) * (n_grid - 1);
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

    //-----------------------------//
    //      Constraints            //
    //-----------------------------//
    public int n_constraints;
    public List<int> constrained_vertices;
    public List<int> constrained_edges;

    public edge_length_constraint[] edge_constraints;
    public position_constraint[] boundary;
    public int counter = 0;
    // Start is called before the first frame update
    void Start()
    {
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        vertices = mesh.vertices;
        h = Time.fixedDeltaTime;
        q = concatenate_vector_array(vertices);
        velocity = Vector<double>.Build.Dense(3 * n_vertices);

        M = Vector<double>.Build.Dense(3 * n_vertices, 1.0);
        M_inv = Vector<double>.Build.Dense(3 * n_vertices, 1.0);

        /////////////////////////////////
        ///      Precomputations      ///
        /////////////////////////////////
        global_LHS = Matrix<double>.Build.SparseOfDiagonalVector(M_inv) / (h * h);
        edge_constraints = new edge_length_constraint[n_edges];
        n_constraints = n_edges;
        for (var i = 0; i < n_edges; i++)
        {
            edge_constraints[i] = new edge_length_constraint(i);
        }
        Matrix<double> St_S_edge_length = precompute_w_St_S(edge_constraints.Select(e => e.Si).ToArray());
        global_LHS += St_S_edge_length;

        boundary = new position_constraint[n_grid];
        n_constraints += n_grid;
        for (var i = 0; i < n_grid; i++)
        {
            boundary[i] = new position_constraint(i);
        }
        Matrix<double> St_S_boundary = precompute_w_St_S(boundary.Select(b => b.Si).ToArray());
        global_LHS += St_S_boundary;
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
        print(h);
        Mesh mesh = GetComponent<MeshFilter>().mesh;

        //-----------------------------//
        //          hot start          //
        //-----------------------------//
        Vector<double> g = Vector<double>.Build.Dense(3 * n_vertices);
        for (var i = 0; i < n_vertices; i++)
        {
            g[3 * i + 1] = -9.8;
        }
        Vector<double> hot_start = q + h * velocity + h * h * M_inv.PointwiseMultiply(g);

        for (var j = 0; j < 2; j++)
        {

            //-----------------------------//
            //         Local steps         //
            //-----------------------------//
            Vector<double> rhs = Vector<double>.Build.Dense(3 * n_vertices);
            rhs += M.PointwiseMultiply(hot_start) / (h * h);

            for (var i = 0; i < n_edges; i++)
            {
                Vector<double> p = edge_constraints[i].project_onto_constraint(q);
                rhs += edge_constraints[i].w * edge_constraints[i].Si.Transpose() * p;
            }
            for (var i = 0; i < n_grid; i++)
            {
                Vector<double> p = boundary[i].project_onto_constraint(q);
                rhs += boundary[i].w * boundary[i].Si.Transpose() * p;
            }

            //-----------------------------//
            //         Global step         /
            //-----------------------------//

            Vector<double> q_new = global_LHS.Solve(rhs);
            Vector<double> velocity_new = (q_new - q) / h;
            velocity = velocity_new;
            q = q_new;
        }

        if (counter % 10 == 0)
        {
            // mesh update
            print(q);
            mesh.vertices = to_vector3_array(q);
        }
        counter++;
    }
}
