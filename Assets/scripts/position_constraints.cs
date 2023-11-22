


// class for position constraints
public class position_constraints
{
    public int n_vertices = Plane.n_vertices;

    SparseMatrix[] build_S_matrix(vertex_index) // vertex_index = int
    {


        Tuple<int, int, double>[] entries = new Tuple<int, int, double>[3];
        entries[0] = new Tuple<int, int, double>(0, 3 * vertex_index + 0, 1);
        entries[1] = new Tuple<int, int, double>(1, 3 * vertex_index + 1, 1);
        entries[2] = new Tuple<int, int, double>(2, 3 * vertex_index + 2, 1);

        SparseMatrix S = SparseMatrix.Build.DenseOfIndexed(3, 3 * n_vertices, entries);

        return S;
    }
}