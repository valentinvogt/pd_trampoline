using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;

public class MyMesh
{
    static public readonly int n_grid = 5;
    static public readonly int n_vertices = n_grid * n_grid;
    public GameObject[] spheres;
    public void Init()
    {
        for (int i = 0; i < n_grid; i++) {
            for (int j = 0; j < n_grid; j++)
            {
                GameObject g = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                g.transform.localScale = new Vector3(0.1f, 0.1f, 0.1f);
                g.transform.position = new Vector3(i, 0, j);
                g.tag = "Player";
            }
        }

        spheres = GameObject.FindGameObjectsWithTag("Player");
    }

    public void set_position(Vector<double> q)
    {
        for (int i = 0; i < n_grid; i++) {
            for (int j = 0; j < n_grid; j++)
            {
                spheres[i * n_grid + j].transform.position = new Vector3((float)q[3 * (i * n_grid + j)], (float)q[3 * (i * n_grid + j) + 1], (float)q[3 * (i * n_grid + j) + 2]);
            }
        }
    }

    public Vector<double> get_q()
    {
        Vector<double> q = Vector<double>.Build.Dense(3 * n_vertices);
        for (int i = 0; i < n_grid; i++) {
            for (int j = 0; j < n_grid; j++)
            {
                q[3 * (i * n_grid + j)] = spheres[i * n_grid + j].transform.position.x;
                q[3 * (i * n_grid + j) + 1] = spheres[i * n_grid + j].transform.position.y;
                q[3 * (i * n_grid + j) + 2] = spheres[i * n_grid + j].transform.position.z;
            }
        }
        return q;
    }
}
