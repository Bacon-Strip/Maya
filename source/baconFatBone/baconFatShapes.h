////////////////////////////////////////////////////////////////////////
// DESCRIPTION:
// Produces the data needed to create the many Bacon Fat Shapes.
//
////////////////////////////////////////////////////////////////////////



#ifndef __baconFatShapes_h 
#define __baconFatShapes_h 

#include <maya/MTypes.h> 



// *********************************************************************
// Box Shape
// *********************************************************************
static double box_VertexList[][3] =
{
	{ 0.0,  0.5, -0.5 },	{ 0.0,  0.5,  0.5 },	{ 0.0, -0.5, -0.5 },	{ 0.0, -0.5,  0.5 },
	{ 1.0,  0.5, -0.5 },	{ 1.0,  0.5,  0.5 },	{ 1.0, -0.5, -0.5 },	{ 1.0, -0.5,  0.5 }
};


int3 box_FaceList[] =
{
	{ 0, 2, 1 },	{ 1, 2, 3 },	{ 0, 1, 5 },	{ 0, 5, 4 },
	{ 1, 3, 7 },	{ 7, 5, 1 },	{ 2, 0, 4 },	{ 2, 4, 6 },
	{ 3, 2, 6 },	{ 3, 6, 7 },	{ 6, 4, 5 },	{ 6, 5, 7 }

};

double box_NormalList[][3] =
{
	{ -1, 0, 0 },	{ -1, 0, 0 },	{ 0, 1, 0 },	{ 0, 1, 0 },
	{ 0, 0, 1 },	{ 0, 0, 1 },	{ 0, 0, -1 },	{ 0, 0, -1 },
	{ 0, -1, 0 },	{ 0, -1, 0 },	{ 1, 0, 0 },	{ 1, 0, 0 }
};


int2 box_EdgeList[] =
{
	{ 0 , 1 },	{ 1 , 3 },	{ 3 , 2 },	{ 2 , 0 },	{ 4 , 5 },
	{ 5 , 7 },	{ 7 , 6 },	{ 6 , 4 },	{ 0 , 4 },	{ 1 , 5 },
	{ 2 , 6 },	{ 3 , 7 }
};


// *********************************************************************
// Loop Ball
// *********************************************************************

static double ball_VertexList[][3] =
{
	{ 0, 0, 1 },{ 0, 0.5, 0.866 },{ -0.25, 0.433, 0.866 },{ -0.433, 0.25, 0.866 },{ -0.5, 0, 0.866 },
	{ -0.433, -0.25, 0.866 },{ -0.25, -0.433, 0.866 },{ 0, -0.5, 0.866 },{ 0.25, -0.433, 0.866 },{ 0.433, -0.25, 0.866 },
	{ 0.5, 0, 0.866 },{ 0.433, 0.25, 0.866 },{ 0.25, 0.433, 0.866 },{ 0, 0.866, 0.5 },{ -0.433, 0.75, 0.5 },
	{ -0.75, 0.433, 0.5 },{ -0.866, 0, 0.5 },{ -0.75, -0.433, 0.5 },{ -0.433, -0.75, 0.5 },{ 0, -0.866, 0.5 },
	{ 0.433, -0.75, 0.5 },{ 0.75, -0.433, 0.5 },{ 0.866, 0, 0.5 },{ 0.75, 0.433, 0.5 },{ 0.433, 0.75, 0.5 },
	{ 0, 1, 0 },{ -0.5, 0.866, 0 },{ -0.866, 0.5, 0 },{ -1, 0, 0 },{ -0.866, -0.5, 0 },
	{ -0.5, -0.866, 0 },{ 0, -1, 0 },{ 0.5, -0.866, 0 },{ 0.866, -0.5, 0 },{ 1, 0, 0 },
	{ 0.866, 0.5, 0 },{ 0.5, 0.866, 0 },{ 0, 0.866, -0.5 },{ -0.433, 0.75, -0.5 },{ -0.75, 0.433, -0.5 },
	{ -0.866, 0, -0.5 },{ -0.75, -0.433, -0.5 },{ -0.433, -0.75, -0.5 },{ 0, -0.866, -0.5 },{ 0.433, -0.75, -0.5 },
	{ 0.75, -0.433, -0.5 },{ 0.866, 0, -0.5 },{ 0.75, 0.433, -0.5 },{ 0.433, 0.75, -0.5 },{ 0, 0.5, -0.866 },
	{ -0.25, 0.433, -0.866 },{ -0.433, 0.25, -0.866 },{ -0.5, 0, -0.866 },{ -0.433, -0.25, -0.866 },{ -0.25, -0.433, -0.866 },
	{ 0, -0.5, -0.866 },{ 0.25, -0.433, -0.866 },{ 0.433, -0.25, -0.866 },{ 0.5, 0, -0.866 },{ 0.433, 0.25, -0.866 },
	{ 0.25, 0.433, -0.866 },{ 0, 0, -1 }
};

int3 ball_FaceList[] =
{
	{ 0, 1, 2 },{ 0, 2, 3 },{ 0, 3, 4 },{ 0, 4, 5 },{ 0, 5, 6 },
	{ 0, 6, 7 },{ 0, 7, 8 },{ 0, 8, 9 },{ 0, 9, 10 },{ 0, 10, 11 },
	{ 0, 11, 12 },{ 0, 12, 1 },{ 1, 13, 14 },{ 16, 4, 3 },{ 15, 16, 3 },
	{ 15, 3, 2 },{ 14, 15, 2 },{ 1, 14, 2 },{ 4, 16, 17 },{ 19, 7, 6 },
	{ 18, 19, 6 },{ 18, 6, 5 },{ 17, 18, 5 },{ 4, 17, 5 },{ 7, 19, 20 },
	{ 22, 10, 9 },{ 21, 22, 9 },{ 21, 9, 8 },{ 20, 21, 8 },{ 7, 20, 8 },
	{ 10, 22, 23 },{ 13, 1, 12 },{ 24, 13, 12 },{ 24, 12, 11 },{ 23, 24, 11 },
	{ 10, 23, 11 },{ 13, 25, 26 },{ 28, 16, 15 },{ 27, 28, 15 },{ 27, 15, 14 },
	{ 26, 27, 14 },{ 13, 26, 14 },{ 16, 28, 29 },{ 31, 19, 18 },{ 30, 31, 18 },
	{ 30, 18, 17 },{ 29, 30, 17 },{ 16, 29, 17 },{ 19, 31, 32 },{ 34, 22, 21 },
	{ 33, 34, 21 },{ 33, 21, 20 },{ 32, 33, 20 },{ 19, 32, 20 },{ 22, 34, 35 },
	{ 25, 13, 24 },{ 36, 25, 24 },{ 36, 24, 23 },{ 35, 36, 23 },{ 22, 35, 23 },
	{ 25, 37, 38 },{ 40, 28, 27 },{ 39, 40, 27 },{ 39, 27, 26 },{ 38, 39, 26 },
	{ 25, 38, 26 },{ 28, 40, 41 },{ 43, 31, 30 },{ 42, 43, 30 },{ 42, 30, 29 },
	{ 41, 42, 29 },{ 28, 41, 29 },{ 31, 43, 44 },{ 46, 34, 33 },{ 45, 46, 33 },
	{ 45, 33, 32 },{ 44, 45, 32 },{ 31, 44, 32 },{ 34, 46, 47 },{ 37, 25, 36 },
	{ 48, 37, 36 },{ 48, 36, 35 },{ 47, 48, 35 },{ 34, 47, 35 },{ 37, 49, 50 },
	{ 52, 40, 39 },{ 51, 52, 39 },{ 51, 39, 38 },{ 50, 51, 38 },{ 37, 50, 38 },
	{ 40, 52, 53 },{ 55, 43, 42 },{ 54, 55, 42 },{ 54, 42, 41 },{ 53, 54, 41 },
	{ 40, 53, 41 },{ 43, 55, 56 },{ 58, 46, 45 },{ 57, 58, 45 },{ 57, 45, 44 },
	{ 56, 57, 44 },{ 43, 56, 44 },{ 46, 58, 59 },{ 49, 37, 48 },{ 60, 49, 48 },
	{ 60, 48, 47 },{ 59, 60, 47 },{ 46, 59, 47 },{ 61, 52, 51 },{ 61, 51, 50 },
	{ 61, 50, 49 },{ 61, 55, 54 },{ 61, 54, 53 },{ 61, 53, 52 },{ 61, 58, 57 },
	{ 61, 57, 56 },{ 61, 56, 55 },{ 61, 49, 60 },{ 61, 60, 59 },{ 61, 59, 58 }
};


double ball_NormalList[][3] =
{
	{ -0.0692, 0.258, 0.964 },{ -0.189, 0.189, 0.964 },{ -0.258, 0.0692, 0.964 },{ -0.258, -0.0692, 0.964 },{ -0.189, -0.189, 0.964 },
	{ -0.0692, -0.258, 0.964 },{ 0.0692, -0.258, 0.964 },{ 0.189, -0.189, 0.964 },{ 0.258, -0.0692, 0.964 },{ 0.258, 0.0692, 0.964 },
	{ 0.189, 0.189, 0.964 },{ 0.0692, 0.258, 0.964 },{ -0.186, 0.695, 0.695 },{ -0.695, 0.186, 0.695 },{ -0.695, 0.186, 0.695 },
	{ -0.509, 0.509, 0.695 },{ -0.509, 0.509, 0.695 },{ -0.186, 0.695, 0.695 },{ -0.695, -0.186, 0.695 },{ -0.186, -0.695, 0.695 },
	{ -0.186, -0.695, 0.695 },{ -0.509, -0.509, 0.695 },{ -0.509, -0.509, 0.695 },{ -0.695, -0.186, 0.695 },{ 0.186, -0.695, 0.695 },
	{ 0.695, -0.186, 0.695 },{ 0.695, -0.186, 0.695 },{ 0.509, -0.509, 0.695 },{ 0.509, -0.509, 0.695 },{ 0.186, -0.695, 0.695 },
	{ 0.695, 0.186, 0.695 },{ 0.186, 0.695, 0.695 },{ 0.186, 0.695, 0.695 },{ 0.509, 0.509, 0.695 },{ 0.509, 0.509, 0.695 },
	{ 0.695, 0.186, 0.695 },{ -0.251, 0.935, 0.251 },{ -0.935, 0.251, 0.251 },{ -0.935, 0.251, 0.251 },{ -0.685, 0.685, 0.251 },
	{ -0.685, 0.685, 0.251 },{ -0.251, 0.935, 0.251 },{ -0.935, -0.251, 0.251 },{ -0.251, -0.935, 0.251 },{ -0.251, -0.935, 0.251 },
	{ -0.685, -0.685, 0.251 },{ -0.685, -0.685, 0.251 },{ -0.935, -0.251, 0.251 },{ 0.251, -0.935, 0.251 },{ 0.935, -0.251, 0.251 },
	{ 0.935, -0.251, 0.251 },{ 0.685, -0.685, 0.251 },{ 0.685, -0.685, 0.251 },{ 0.251, -0.935, 0.251 },{ 0.935, 0.251, 0.251 },
	{ 0.251, 0.935, 0.251 },{ 0.251, 0.935, 0.251 },{ 0.685, 0.685, 0.251 },{ 0.685, 0.685, 0.251 },{ 0.935, 0.251, 0.251 },
	{ -0.251, 0.935, -0.251 },{ -0.935, 0.251, -0.251 },{ -0.935, 0.251, -0.251 },{ -0.685, 0.685, -0.251 },{ -0.685, 0.685, -0.251 },
	{ -0.251, 0.935, -0.251 },{ -0.935, -0.251, -0.251 },{ -0.251, -0.935, -0.251 },{ -0.251, -0.935, -0.251 },{ -0.685, -0.685, -0.251 },
	{ -0.685, -0.685, -0.251 },{ -0.935, -0.251, -0.251 },{ 0.251, -0.935, -0.251 },{ 0.935, -0.251, -0.251 },{ 0.935, -0.251, -0.251 },
	{ 0.685, -0.685, -0.251 },{ 0.685, -0.685, -0.251 },{ 0.251, -0.935, -0.251 },{ 0.935, 0.251, -0.251 },{ 0.251, 0.935, -0.251 },
	{ 0.251, 0.935, -0.251 },{ 0.685, 0.685, -0.251 },{ 0.685, 0.685, -0.251 },{ 0.935, 0.251, -0.251 },{ -0.186, 0.695, -0.695 },
	{ -0.695, 0.186, -0.695 },{ -0.695, 0.186, -0.695 },{ -0.509, 0.509, -0.695 },{ -0.509, 0.509, -0.695 },{ -0.186, 0.695, -0.695 },
	{ -0.695, -0.186, -0.695 },{ -0.186, -0.695, -0.695 },{ -0.186, -0.695, -0.695 },{ -0.509, -0.509, -0.695 },{ -0.509, -0.509, -0.695 },
	{ -0.695, -0.186, -0.695 },{ 0.186, -0.695, -0.695 },{ 0.695, -0.186, -0.695 },{ 0.695, -0.186, -0.695 },{ 0.509, -0.509, -0.695 },
	{ 0.509, -0.509, -0.695 },{ 0.186, -0.695, -0.695 },{ 0.695, 0.186, -0.695 },{ 0.186, 0.695, -0.695 },{ 0.186, 0.695, -0.695 },
	{ 0.509, 0.509, -0.695 },{ 0.509, 0.509, -0.695 },{ 0.695, 0.186, -0.695 },{ -0.258, 0.0692, -0.964 },{ -0.189, 0.189, -0.964 },
	{ -0.0692, 0.258, -0.964 },{ -0.0692, -0.258, -0.964 },{ -0.189, -0.189, -0.964 },{ -0.258, -0.0692, -0.964 },{ 0.258, -0.0692, -0.964 },
	{ 0.189, -0.189, -0.964 },{ 0.0692, -0.258, -0.964 },{ 0.0692, 0.258, -0.964 },{ 0.189, 0.189, -0.964 },{ 0.258, 0.0692, -0.964 }
};


int2 ball_EdgeList[] =
{
	{ 1, 2 },{ 2, 3 },{ 3, 4 },{ 4, 5 },{ 5, 6 },
	{ 6, 7 },{ 7, 8 },{ 8, 9 },{ 9, 10 },{ 10, 11 },
	{ 11, 12 },{ 12, 1 },{ 1, 13 },{ 16, 4 },{ 19, 7 },
	{ 22, 10 },{ 13, 25 },{ 25, 26 },{ 28, 16 },{ 27, 28 },
	{ 26, 27 },{ 28, 29 },{ 31, 19 },{ 30, 31 },{ 29, 30 },
	{ 31, 32 },{ 34, 22 },{ 33, 34 },{ 32, 33 },{ 34, 35 },
	{ 36, 25 },{ 35, 36 },{ 25, 37 },{ 40, 28 },{ 43, 31 },
	{ 46, 34 },{ 37, 49 },{ 49, 50 },{ 52, 40 },{ 51, 52 },
	{ 50, 51 },{ 52, 53 },{ 55, 43 },{ 54, 55 },{ 53, 54 },
	{ 55, 56 },{ 58, 46 },{ 57, 58 },{ 56, 57 },{ 58, 59 },
	{ 60, 49 },{ 59, 60 }
};




// *********************************************************************
// Loop Shape
// *********************************************************************

static double loop_VertexList[][3] =
{
	{ 1, 0, 0 },{ 0.866, 0.5, 0 },{ 0.5, 0.866, 0 },{ 0, 1, 0 },{ -0.5, 0.866, 0 },
	{ -0.866, 0.5, 0 },{ -1, 0, 0 },{ -0.866, -0.5, 0 },{ -0.5, -0.866, 0 },{ 0, -1, 0 },
	{ 0.5, -0.866, 0 },{ 0.866, -0.5, 0 },{ 1, 0, 1 },{ 0.866, 0.5, 1 },{ 0.5, 0.866, 1 },
	{ 0, 1, 1 },{ -0.5, 0.866, 1 },{ -0.866, 0.5, 1 },{ -1, 0, 1 },{ -0.866, -0.5, 1 },
	{ -0.5, -0.866, 1 },{ 0, -1, 1 },{ 0.5, -0.866, 1 },{ 0.866, -0.5, 1 },{ 0.111, -0.994, 0 },
	{ 0.111, -0.994, 1 },{ 0.0553, -0.997, 0.5 }
};

int3 loop_FaceList[] =
{
	{ 2, 3, 15 },{ 2, 15, 14 },{ 1, 2, 14 },{ 1, 14, 13 },{ 0, 1, 13 },
	{ 0, 13, 12 },{ 5, 6, 18 },{ 5, 18, 17 },{ 4, 5, 17 },{ 4, 17, 16 },
	{ 3, 4, 16 },{ 3, 16, 15 },{ 8, 9, 21 },{ 8, 21, 20 },{ 7, 8, 20 },
	{ 7, 20, 19 },{ 6, 7, 19 },{ 6, 19, 18 },{ 11, 0, 12 },{ 11, 12, 23 },
	{ 10, 11, 23 },{ 10, 23, 22 },{ 10, 22, 25 },{ 24, 10, 25 },{ 26, 24, 25 }
};


double loop_NormalList[][3] =
{
	{ 0.259, 0.966, 0 },{ 0.259, 0.966, 0 },{ 0.707, 0.707, 0 },{ 0.707, 0.707, 0 },{ 0.966, 0.259, 0 },
	{ 0.966, 0.259, 0 },{ -0.966, 0.259, 0 },{ -0.966, 0.259, 0 },{ -0.707, 0.707, 0 },{ -0.707, 0.707, 0 },
	{ -0.259, 0.966, 0 },{ -0.259, 0.966, 0 },{ -0.259, -0.966, 0 },{ -0.259, -0.966, 0 },{ -0.707, -0.707, 0 },
	{ -0.707, -0.707, 0 },{ -0.966, -0.259, 0 },{ -0.966, -0.259, 0 },{ 0.966, -0.259, 0 },{ 0.966, -0.259, 0 },
	{ 0.707, -0.707, 0 },{ 0.707, -0.707, 0 },{ 0.312, -0.95, 0 },{ 0.312, -0.95, 0 },{ 0.0556, -0.998, 0 }
};


int2 loop_EdgeList[] =
{
	{ 2, 3 },{ 3, 15 },{ 15, 14 },{ 1, 2 },{ 14, 13 },
	{ 0, 1 },{ 13, 12 },{ 12, 0 },{ 5, 6 },{ 6, 18 },
	{ 18, 17 },{ 4, 5 },{ 17, 16 },{ 3, 4 },{ 16, 15 },
	{ 8, 9 },{ 9, 21 },{ 21, 20 },{ 7, 8 },{ 20, 19 },
	{ 6, 7 },{ 19, 18 },{ 11, 0 },{ 12, 23 },{ 10, 11 },
	{ 23, 22 },{ 22, 25 },{ 24, 10 },{ 26, 24 },{ 25, 26 }
};




// *********************************************************************
// Rig Base
// *********************************************************************

static double rigBase_VertexList[][3] =
{
	{ 1, 0, 0 },{ 0.974, 0.229, 0 },{ 0.898, 0.44, 0 },{ 0.78, 0.625, 0 },{ 0.625, 0.78, 0 },
	{ 0.44, 0.898, 0 },{ 0.229, 0.974, 0 },{ 0, 1, 0 },{ -0.229, 0.974, 0 },{ -0.44, 0.898, 0 },
	{ -0.625, 0.78, 0 },{ -0.78, 0.625, 0 },{ -0.898, 0.44, 0 },{ -0.974, 0.229, 0 },{ -1, 0, 0 },
	{ -0.974, -0.229, 0 },{ -0.898, -0.44, 0 },{ -0.78, -0.625, 0 },{ -0.625, -0.78, 0 },{ -0.44, -0.898, 0 },
	{ -0.229, -0.974, 0 },{ 0, -1, 0 },{ 0.229, -0.974, 0 },{ 0.44, -0.898, 0 },{ 0.625, -0.78, 0 },
	{ 0.78, -0.625, 0 },{ 0.898, -0.44, 0 },{ 0.974, -0.229, 0 },{ -0.609, 0.0894, 0 },{ -0.63, 0.13, 0 },
	{ -0.637, 0.167, 0 },{ -0.632, 0.199, 0 },{ -0.614, 0.226, 0 },{ -0.585, 0.246, 0 },{ -0.545, 0.259, 0 },
	{ -0.496, 0.264, 0 },{ 0, 0.26, 0 },{ 0.496, 0.264, 0 },{ 0.545, 0.259, 0 },{ 0.585, 0.246, 0 },
	{ 0.614, 0.226, 0 },{ 0.632, 0.199, 0 },{ 0.637, 0.167, 0 },{ 0.63, 0.13, 0 },{ 0.609, 0.0894, 0 },
	{ 0.358, -0.289, 0 },{ 0.113, -0.671, 0 },{ 0.0845, -0.707, 0 },{ 0.052, -0.731, 0 },{ 0.0176, -0.743, 0 },
	{ -0.0176, -0.743, 0 },{ -0.0521, -0.731, 0 },{ -0.0845, -0.707, 0 },{ -0.113, -0.671, 0 },{ -0.358, -0.289, 0 }
};


int3 rigBase_FaceList[] =
{
	{ 44, 0, 1 },{ 43, 44, 1 },{ 42, 43, 1 },{ 42, 1, 2 },{ 41, 42, 2 },
	{ 40, 41, 2 },{ 40, 2, 3 },{ 39, 40, 3 },{ 27, 0, 44 },{ 27, 44, 45 },
	{ 26, 27, 45 },{ 25, 26, 45 },{ 24, 25, 45 },{ 24, 45, 46 },{ 23, 24, 46 },
	{ 22, 23, 46 },{ 22, 46, 47 },{ 21, 22, 47 },{ 21, 47, 48 },{ 21, 48, 49 },
	{ 21, 49, 50 },{ 20, 21, 50 },{ 20, 50, 51 },{ 20, 51, 52 },{ 19, 20, 52 },
	{ 19, 52, 53 },{ 19, 53, 54 },{ 18, 19, 54 },{ 17, 18, 54 },{ 16, 17, 54 },
	{ 16, 54, 28 },{ 15, 16, 28 },{ 14, 15, 28 },{ 13, 14, 28 },{ 13, 28, 29 },
	{ 13, 29, 30 },{ 12, 13, 30 },{ 12, 30, 31 },{ 12, 31, 32 },{ 11, 12, 32 },
	{ 11, 32, 33 },{ 38, 39, 3 },{ 11, 33, 34 },{ 10, 11, 34 },{ 10, 34, 35 },
	{ 38, 3, 4 },{ 37, 38, 4 },{ 36, 37, 4 },{ 36, 4, 5 },{ 36, 5, 6 },
	{ 36, 6, 7 },{ 36, 7, 8 },{ 35, 36, 8 },{ 10, 35, 8 },{ 10, 8, 9 }
};


double rigBase_NormalList[][3] =
{
	{ 0, 0, 1 },{ 0, 0, 1 },{ 0, 0, 1 },{ 0, 0, 1 },{ 0, 0, 1 },
	{ 0, 0, 1 },{ 0, 0, 1 },{ 0, 0, 1 },{ 0, 0, 1 },{ 0, 0, 1 },
	{ 0, 0, 1 },{ 0, 0, 1 },{ 0, 0, 1 },{ 0, 0, 1 },{ 0, 0, 1 },
	{ 0, 0, 1 },{ 0, 0, 1 },{ 0, 0, 1 },{ 0, 0, 1 },{ 0, 0, 1 },
	{ 0, 0, 1 },{ 0, 0, 1 },{ 0, 0, 1 },{ 0, 0, 1 },{ 0, 0, 1 },
	{ 0, 0, 1 },{ 0, 0, 1 },{ 0, 0, 1 },{ 0, 0, 1 },{ 0, 0, 1 },
	{ 0, 0, 1 },{ 0, 0, 1 },{ 0, 0, 1 },{ 0, 0, 1 },{ 0, 0, 1 },
	{ 0, 0, 1 },{ 0, 0, 1 },{ 0, 0, 1 },{ 0, 0, 1 },{ 0, 0, 1 },
	{ 0, 0, 1 },{ 0, 0, 1 },{ 0, 0, 1 },{ 0, 0, 1 },{ 0, 0, 1 },
	{ 0, 0, 1 },{ 0, 0, 1 },{ 0, 0, 1 },{ 0, 0, 1 },{ 0, 0, 1 },
	{ 0, 0, 1 },{ 0, 0, 1 },{ 0, 0, 1 },{ 0, 0, 1 },{ 0, 0, 1 }
};


int2 rigBase_EdgeList[] =
{
	{ 0, 1 },{ 43, 44 },{ 42, 43 },{ 1, 2 },{ 41, 42 },
	{ 40, 41 },{ 2, 3 },{ 39, 40 },{ 27, 0 },{ 44, 45 },
	{ 26, 27 },{ 25, 26 },{ 24, 25 },{ 45, 46 },{ 23, 24 },
	{ 22, 23 },{ 46, 47 },{ 21, 22 },{ 47, 48 },{ 48, 49 },
	{ 49, 50 },{ 20, 21 },{ 50, 51 },{ 51, 52 },{ 19, 20 },
	{ 52, 53 },{ 53, 54 },{ 18, 19 },{ 17, 18 },{ 16, 17 },
	{ 54, 28 },{ 15, 16 },{ 14, 15 },{ 13, 14 },{ 28, 29 },
	{ 29, 30 },{ 12, 13 },{ 30, 31 },{ 31, 32 },{ 11, 12 },
	{ 32, 33 },{ 38, 39 },{ 33, 34 },{ 10, 11 },{ 34, 35 },
	{ 3, 4 },{ 37, 38 },{ 36, 37 },{ 4, 5 },{ 5, 6 },
	{ 6, 7 },{ 7, 8 },{ 35, 36 },{ 8, 9 },{ 9, 10 }
};






#endif 



