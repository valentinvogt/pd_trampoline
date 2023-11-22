

class public: edge_length_constraints
{
    public int n_vertices = Plane.n_vertices;

int get_vertex_index(int i, int j)
{
    return i * n_grid + j;
}

float get_edge_length(int edge_idx, Vector3[] vertices)
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

Vector project_edge_length_constraint(Vector q, Matrix Si, float Li)
{
    // var V = Vector<double>.Build;
    Vector pi = (Vector)(Si * q);
    Vector pi_hat = (Vector)(Li * pi / pi.Norm(2));
    return pi_hat;
}

SparseMatrix[] build_S_edge_length(edge_index) // edge_index = tuple<int,int>
{
    Tuple<int, int, double>[] entries = new Tuple<int, int, double>[6];
    int[] vertices = get_vertices_of_edge(edge_index);

    entries[0] = new Tuple<int, int, double>(0, 3 * vertices[0] + 0, 1);
    entries[1] = new Tuple<int, int, double>(1, 3 * vertices[0] + 1, 1);
    entries[2] = new Tuple<int, int, double>(2, 3 * vertices[0] + 2, 1);
    entries[3] = new Tuple<int, int, double>(0, 3 * vertices[1] + 0, -1);
    entries[4] = new Tuple<int, int, double>(1, 3 * vertices[1] + 1, -1);
    entries[5] = new Tuple<int, int, double>(2, 3 * vertices[1] + 2, -1);

    SparseMatrix S = SparseMatrix.Build.DenseOfIndexed(3, 3 * n_vertices, entries);

    return S;
}
}
