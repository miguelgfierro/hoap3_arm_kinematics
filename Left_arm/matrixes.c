#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "matrixes.h"

int initialize_matrix (matrix *M, int n_rows, int n_columns)
{
int i;
M->vector = (float *)malloc(sizeof(float) *n_rows*n_columns);
M->n_rows=n_rows;
M->n_columns=n_columns;
if(M->vector == NULL){
			 printf("\nOut of memory");
			 exit(1);
             }
for(i=0; i<n_rows*n_columns; i++) {
		 M->vector[i] = 0;
		}
}


void print_matrix (matrix M)
{
 int i, j;
 for(i=0;i<M.n_rows;i++) {
 						 printf("\nRow %d: ", i+1);
						 for(j=0;j<M.n_columns;j++) {
                                                  	printf("%f,",M.vector[identify_element (&M, i, j)]);
                                                	}
                         }
 }

int identify_element (matrix *M, int row, int column)
{
    return M->n_columns*row+column;
}

int matrix_multiply(matrix *a, matrix *b, matrix *res)
{
if (a->n_columns!=b->n_rows) return -1;
int i, j, k, element;
for(i=0; i<a->n_rows; i++) 
		 for(j=0; j<b->n_columns; j++){
		 		  element = identify_element (res, i, j);
		 		  res->vector[element]=0;
         		  for(k=0; k<a->n_columns; k++)
				  		   res->vector[element] +=  a->vector[identify_element (a, i, k)] * b->vector[identify_element (b, k, j)];
				   }
}

int matrix_diff(matrix *a, matrix *b, matrix *res)
{
if (a->n_rows!=b->n_rows) return -1;
if (a->n_columns!=b->n_columns) return -1;
int i, j, element;
for(i=0; i<a->n_rows; i++) 
		 for(j=0; j<a->n_columns; j++){
		 		  element = identify_element (res, i, j);
		 		  res->vector[element]=a->vector[element]-b->vector[element];
				   }
}

void matrix_vector_multiply(matrix *a, float *b, float *res)
{
int i, j;
for(i=0; i<a->n_rows; i++) {
		 res[i]=0;
		 for(j=0; j<a->n_columns; j++)
		 		  res[i] +=  a->vector[identify_element (a, i, j)] * b[j];
	     }
}

void create_diagonal_matrix (matrix *M, float value)
{
 	 int i;
	 for(i=0;i<M->n_rows;i++)
	 						M->vector[identify_element (M, i, i)]=value;
}

void vector_subtraction (float *a, float *b, float *res, int dim)
{
 	 int i;
 	 for (i=0; i<dim; i++)
 	 	 res[i]=a[i]-b[i];
}

void vector_sum (float *a, float *b, float *res, int dim)
{
 	 int i;
 	 for (i=0; i<dim; i++)
 	 	 res[i]=a[i]+b[i];
}

void evaluate_minor (matrix *M, matrix *C, int row, int column)
{
int i, j, k;
k=0;
for(i=0;i<row-1;i++) {
					   for(j=0;j<column-1;j++)
					   						{
					   						C->vector[k]=M->vector[identify_element(M,i,j)];
					   						k++;
											}
                       for(j=column;j<M->n_columns;j++)
					                        {
					   						C->vector[k]=M->vector[identify_element(M,i,j)];
					   						k++;
											}
											}
for(i=row;i<M->n_rows;i++) {
					   for(j=0;j<column-1;j++)
					   						{
					   						C->vector[k]=M->vector[identify_element(M,i,j)];
					   						k++;
											}
                       for(j=column;j<M->n_columns;j++)
					                        {
					   						C->vector[k]=M->vector[identify_element(M,i,j)];
					   						k++;
											}
											}
}

float evaluate_determinant (matrix *M)
{
 	  float res;
 	  int j;
 	  matrix C;
 	  
  	 if (M->n_rows==2)
	   res=M->vector[0]*M->vector[3]-M->vector[1]*M->vector[2];
	   else {
	   		res=0;
			for(j=0;j<M->n_columns;j++)
									   res+=M->vector[j]*evaluate_cofactor (M, 1, j+1);
			}
return res;
}

float evaluate_cofactor (matrix *M, int row, int column)
{
	matrix A;
	float c;
	float a;
	if (((row+column) % 2) == 0) a=1;
    else a=-1;
    initialize_matrix (&A, M->n_rows-1,M->n_rows-1);
	evaluate_minor (M, &A, row, column);
	c=a*evaluate_determinant(&A);
	free(A.vector);
	return c;
}

int matrix_inverse (matrix *M, matrix *Inv)
{
float det, det_inv;
int i, j;
det=evaluate_determinant (M);
if (det==0)
   {
   printf("Determinant zero...exiting");
   return(-1);
   }
else
	det_inv = 1/det;

for(i=0;i<M->n_rows;i++) 					   			
					   for(j=0;j<M->n_columns;j++) {
					   							   Inv->vector[identify_element(M,i,j)]=det_inv*evaluate_cofactor (M, j+1, i+1);
                 								   }
}

void matrix_transpose (matrix *M, matrix *T)
{
int i, j;
for(i=0;i<M->n_rows;i++) 					   			
					   for(j=0;j<M->n_columns;j++)
					   							   T->vector[identify_element(T,j,i)]=M->vector[identify_element(M,i,j)];
}
