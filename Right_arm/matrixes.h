#ifndef __MATRIXES__
#define __MATRIXES__

typedef struct {
        float *vector;
        int n_rows;
        int n_columns;
        } matrix;

int initialize_matrix (matrix *M, int n_rows, int n_columns);
void print_matrix (matrix M);
int identify_element (matrix *M, int row, int column);
int matrix_multiply(matrix *a, matrix *b, matrix *res);
void matrix_vector_multiply(matrix *a, float *b, float *res);
void create_diagonal_matrix (matrix *M, float value);
void vector_subtraction (float *a, float *b, float *res, int dim);
void vector_sum (float *a, float *b, float *res, int dim);
void evaluate_minor (matrix *M, matrix *C, int row, int column);
float evaluate_cofactor (matrix *M, int row, int column);
float evaluate_determinant (matrix *M);
int matrix_inverse (matrix *M, matrix *Inv);
void matrix_transpose (matrix *M, matrix *T);
int matrix_diff(matrix *a, matrix *b, matrix *res);

#endif
