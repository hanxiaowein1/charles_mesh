#include "charles_mesh.h"

namespace
{

class Point3D
{
public:
    double x, y, z;
};

class HalfEdge;

class Vertex
{
public:
    Point3D position;
    HalfEdge* half_edge;
};

class Face
{
public:
    HalfEdge* half_edge;
};

class HalfEdge
{
public:
    Vertex* vertex;
    Face* face;
    HalfEdge* next;
    HalfEdge* prev;
    HalfEdge* opposite;
};

};
