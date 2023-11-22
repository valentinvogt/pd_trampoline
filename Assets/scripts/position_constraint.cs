using System;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;

public class position_constraint
{
    public position_constraint(int i, double w = 1.0)
    {
        this.rest_position = get_vertex_position(i);
        this.Si = build_S_position(i);
        this.w = w;
    }
    public Vector<double> rest_position;
    public Matrix<double> Si;
    public double w = 1.0;
    //-----------------------------//
    //      Mesh parameters        //
    //-----------------------------//
    Vector<double> q = Plane.q;
    public static readonly int n_grid = Plane.n_grid;
    public readonly int n_vertices = Plane.n_vertices;

    Vector<double> get_vertex_position(int vertex_index)
    {
        return Vector<double>.Build.DenseOfArray(
            new double[] { q[3 * vertex_index], q[3 * vertex_index + 1], q[3 * vertex_index + 2] });
    }

    public Vector<double> project_onto_constraint(Vector<double> q)
    {
        Vector<double> p = Vector<double>.Build.Dense(3);
        p = Vector<double>.Build.DenseOfVector(rest_position);
        return p;
    }

    public Matrix<double> build_S_position(int vertex_index)
    {
        var M = Matrix<double>.Build;

        Tuple<int, int, double>[] entries = new Tuple<int, int, double>[3];
        entries[0] = new Tuple<int, int, double>(0, 3 * vertex_index + 0, 1);
        entries[1] = new Tuple<int, int, double>(1, 3 * vertex_index + 1, 1);
        entries[2] = new Tuple<int, int, double>(2, 3 * vertex_index + 2, 1);

        Matrix<double> S = M.SparseOfIndexed(3, 3 * n_vertices, entries);

        return S;
    }
}