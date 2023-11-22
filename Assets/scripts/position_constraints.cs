using System;


// class for position constraints
using MathNet.Numerics.LinearAlgebra;

public class position_constraints
{
    public int n_vertices = Plane.n_vertices;

    Matrix<double> build_S_matrix(int vertex_index)
    {


        Tuple<int, int, double>[] entries = new Tuple<int, int, double>[3];
        entries[0] = new Tuple<int, int, double>(0, 3 * vertex_index + 0, 1);
        entries[1] = new Tuple<int, int, double>(1, 3 * vertex_index + 1, 1);
        entries[2] = new Tuple<int, int, double>(2, 3 * vertex_index + 2, 1);

        Matrix<double> S = Matrix<double>.Build.SparseOfIndexed(3, 3 * n_vertices, entries);

        return S;
    }
}