using System;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;

public class edge_length_constraint
{
    public double rest_length;
    public Matrix<double> Si;
    public edge_length_constraint(int i) {
        this.rest_length = get_edge_length(i,q);
        this.Si = build_S_edge_length(i);
    }

    /////////////////////////////////
    ///        Grid parameters    ///
    /////////////////////////////////
    Vector<double> q = Plane.q;
    public static readonly int n_grid = Plane.n_grid;
    public int n_vertices = Plane.n_vertices;
    public int n_edges = Plane.n_edges;

    /////////////////////////////////
    ///    Helper functions       ///
    /////////////////////////////////
    static Vector3 segment3(Vector<double> q, int n)
    {
        return new Vector3((float)q[3 * n], (float)q[3 * n + 1], (float)q[3 * n + 2]);   
    }

    int get_vertex_index(int i, int j)
    {
        return i * n_grid + j;
    }

    static public float get_edge_length(int edge_idx, Vector<double> q)
    {
        int[] vertices_of_edge = get_vertices_of_edge(edge_idx);
        int v1 = vertices_of_edge[0];
        int v2 = vertices_of_edge[1];
        return (segment3(q, v1) - segment3(q, v2)).magnitude;
    }

    static int[] get_vertices_of_edge(int edge_idx)
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

    Vector<double> project_onto_constraint(Vector<double> q)
    {
        Vector<double> pi = Si * q;
        Vector<double> pi_hat = rest_length * pi / pi.Norm(2);
        return pi_hat;
    }

    public Matrix<double> build_S_edge_length(int i)
    {
        var M = Matrix<double>.Build;

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

        Matrix<double> S_i = M.SparseOfIndexed(3, n_vertices * 3, entries);

        return S_i;
    }
}
