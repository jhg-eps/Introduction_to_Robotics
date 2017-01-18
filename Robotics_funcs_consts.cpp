#include "robotics_funcs_consts.h"
#include "header.h"
#include <iostream>
#include <math.h>

robotics_funcs_consts::robotics_funcs_consts()      // constructor function
{
}

// function declarations go here
void robotics_funcs_consts::matrix_multiply(double A[4][4],double B[4][4],double C[4][4])
{
  std::cout << "\n" << std::endl;
    const int size = 4;         // stylistic choice to use a constant equal to 4 as the order of the homogeneous matrix
    int i = 0,j = 0,k = 0;         // initialize matrix looping variables
    double sum = 0.0;               // initialize the sum which represents C_ij (C[i-1][j-1])

    for( i = 0; i < size; i++)           // pick the ith row of matrix A (A[i][0,1,2,3])
    {
        for(k = 0; k < size; k++)
        {
            sum = 0;                                 // reinitialize the sum representing C_ik for every new combination of i and k
            for(j = 0; j < size; j++)
            {
                sum = sum + A[i][j] * B[j][k];        // sum the product of the ith row of A times the kth column of B to compute C[i][k]
            }
            C[i][k] = sum;
        }
    }
//    print_matrix(C);     // print out the product C = A*B
 //   std::cout << "\n" << std::endl;
}

//this function will transform the translation/position vector point using the selected homogeneous matrix R, and the transformed values will be stored into point_tfmd.
void robotics_funcs_consts::transform_point(double R[4][4], double point[4], double point_tfmd[4])
{
    int j=0, k= 0;                  // initialize matrix looping variables
       double row_sum = 0;               // initialize the variable which the hold the sum for an individual matrix element M_ij
       std::cout << "Your transformation matrix is:\n\n" << std::endl;
       print_matrix(R);                         // print out the transformation matrix to the user just in case they want to make sure that all is as it should be
   for(int i = 0; i < 4; i++)
    {
        row_sum = 0;                                // reinitialize the sum representing point_tfmd[i] for every new i value
       for (j =0; j < 4; j++)
       {
            row_sum = row_sum + R[i][j]*point[j];     // sum the product of the ith row of R times the single column of point to compute point_tfmd[i]
       }
      point_tfmd[i] = row_sum;
     }

std::cout << "\nYour transformed point is:\n\n" << std::endl;
for(k=0; k < 4; k++)  std::cout << point_tfmd[k] << std::endl;            // print out the transformed vector point_tfmd in homogeneous form.
std::cout<<"\n\n" << std::endl;
}

//function through which point's coordinates will be stored in memory (in the scope of main)
void robotics_funcs_consts::input_point(double point[5])
{
    std::cout << "Enter the 3D coordinates of the point in the order x,y,z or U,V,W\n\n" << std::endl;
    int k = 0;                      // initialize counting variable to go through the array indices representing the point.
    double coordinate = 0;             // initialize the variable which will temporarily hold the user's input for a particular coordinate (x,y,z, etc.)
        for(k = 0; k < 3; k++)
        {
            std::cin >> coordinate;        // read the user's input
            point[k] = coordinate;           // assign the given input into memory
        }
            point[3] = 1;                      // make the position vector homogeneous for the user.
            std::cout << "\nYour Homogeneous Position Vector is:\n\n" << std::endl;
        for(k = 0; k < 4; k++)
            std::cout << point[k]  << std::endl;                      // print out the points of the homogeneous matri
        std::cout << "\n" << std::endl;
}

// function through which matrix M's components will be stored in memory (in the scope of main)
void robotics_funcs_consts::input_matrix(double M[4][4])
{
int i = 0;
int j = 0;               // initialize matrix looping variables
double matrix_value = 0;
    for(i = 0; i < 3; i++)              // enter the loop which will allow the user to assign values to the subsidiary indices of the ith row of Matrix M
    {
        for(j = 0; j < 4; j++)          // enter the loop to assign values to the subsidiary indices (columns) of the ith row of matrix M
        {
        std::cin >> matrix_value;         // read user input
        M[i][j] = matrix_value;             // assign the value into the corresponding Matrix index
        }
    }
      M[3][0] = 0;
      M[3][1] = 0;             //make the matrix homogeneous (4th row)
      M[3][2] = 0;
      M[3][3] = 1;
}

//Function to invert any given homogeneous matrix
void robotics_funcs_consts::invert_matrix(double M[4][4])
{
   int i =0, j =0, b = 0;
   double temp = 0;          // variable used to hold values in the swapping process.
   double row_sum = 0;
   double row_sum_[3];       // used as the holding variable for sums created due to a row of Matrix A being multiplied by a column of Matrix B (assuming A*B)
   double R[3][3];          // the rotational section of the segmented transformation matrix
   const int neg1 = -1;  // constant by which we multiply the transposed rotational matrix

 //  transpose the rotational part of the homogeneous transformation matrix
         for(i = 0; i < 3; i++)             // go through the rows of the matrix M
         {
              for(j = 0; j < 3; j++)              // go through the columns of matrix M
              {
                if(i < j)                                // isolate M_12, M_13, and M_23 for swapping (the lower triangular part of the rotational matrix
                {
                    temp = M[i][j];                     // transpose the rotational part of the matrix
                    M[i][j] = M[j][i];
                    M[j][i] = temp;
                }
               }
         }
// multiply the transposed rotational matrix by (-1) to become -R_T
         for(i = 0; i < 3; i++)          // go through the rows of the rotational part R_T of the homogeneous matrix M
         {
              for(j = 0; j < 3; j++)     // go through the columns of the rotational part R_T of the homogeneous matrix M
              {
                     R[i][j] = M[i][j]*neg1;   //   R_T = -(R_T);
              }
         }
// multiply the negative transposed matrix -R_T by translational vector so that the point can be transformed
   for(i = 0; i < 3; i++)       // go through the rows of -R_T
    {
        row_sum = 0;
       for (j =0; j < 3; j++)     // go through the columns of -R_T
       {
        row_sum = row_sum + R[i][j]*M[j][3]; // sum the product of the ith row of -R_T times the single column of t to compute the new translation vector (t', for example)
       // std::cout"at row %d and column %d, row_sum is currently %f\n", i,j,row_sum);
       }
       row_sum_[i] = row_sum;
     }

    for(b = 0; b < 3; b++)
        M[b][3] = row_sum_[b];

M[3][0] = 0;
M[3][1] = 0;                   // autofill the 4th row of the matrix so that the user does not have to do so.
M[3][2] = 0;
M[3][3] = 1;
print_matrix(M);
}
void robotics_funcs_consts::print_matrix(double M[4][4])  // print out a general 4x4 matrix, in traditional Matrix format
{
    int i =0, j =0;    // initialize matrix counting variables
    for(i = 0; i < 4; i++)
    {
        for(j =0; j < 4; j++)
        {
            std::cout << M[i][j] << " ";   // print out M[i][j]
        }
        std::cout << "\n" << std::endl;
    }
}
void robotics_funcs_consts::display_matrices(double M1[4][4], double M2[4][4], double M3[4][4], double M4[4][4], double M5[4][4], double M6[4][4])
{
    robotics_funcs_consts robomath;
    std::cout << "Here are the five transformation matrices.\n" << std::endl;

    std::cout << "0T1" << std::endl;
    robomath.robotics_funcs_consts::print_matrix(M1);
    std::cout << "\n" << std::endl;

    std::cout << "1T2\n" << std::endl;
    robomath.robotics_funcs_consts::print_matrix(M2);
     std::cout << "\n" << std::endl;

     std::cout << "2T3\n" << std::endl;
    robomath.robotics_funcs_consts::print_matrix(M3);
     std::cout << "\n" << std::endl;

     std::cout << "3T4\n" << std::endl;
    robomath.robotics_funcs_consts::print_matrix(M4);
     std::cout << "\n" << std::endl;

     std::cout << "4T5\n" << std::endl;
    robomath.robotics_funcs_consts::print_matrix(M5);
     std::cout << "\n" << std::endl;

     std::cout << "5T0\n" << std::endl;
    robomath.robotics_funcs_consts::print_matrix(M6);
    std::cout << "\n" << std::endl;
}
void robotics_funcs_consts::create_general_matrix(double general_matrix[4][4], double d_i, double a_i, double alpha_i, double theta_i)
{
    general_matrix[0][0] = cos(theta_i);
    general_matrix[0][1] = -cos(alpha_i)*sin(theta_i);
    general_matrix[0][2] = sin(alpha_i)*sin(theta_i);
    general_matrix[0][3] = a_i*cos(theta_i);
    general_matrix[1][0] = sin(theta_i);
    general_matrix[1][1] = cos(alpha_i)*cos(theta_i);
    general_matrix[1][2] = -sin(alpha_i)*cos(theta_i);
    general_matrix[1][3] = a_i*sin(theta_i);
    general_matrix[2][0] = 0;
    general_matrix[2][1] = sin(alpha_i);
    general_matrix[2][2] = cos(alpha_i);
    general_matrix[2][3] = d_i;
    general_matrix[3][0] = 0;
    general_matrix[3][1] = 0;
    general_matrix[3][2] = 0;
    general_matrix[3][3] = 1;
}

void robotics_funcs_consts::create_zero_wrt_three_mtx(double M[4][4],double t1, double t2, double t3)
{
    // all input arguments are in radians.
    M[0][0] = cos(t1)*cos(t3 - t2);
    M[0][1] = sin(t1)*cos(t3 - t2);
    M[0][2] = -sin(t3 - t2);
    M[0][3] = -a3 - a2*cos(t3) + d1*sin(t3 - t2);
    M[1][0] = -cos(t1)*sin(t3 - t2);
    M[1][1] = -sin(t1)*sin(t3 - t2);
    M[1][2] = -cos(t3 - t2);
    M[1][3] = a2*sin(t3) + d1*cos(t3 - t2);
    M[2][0] = -sin(t1);
    M[2][1] = cos(t1);
    M[2][2] = 0;
    M[2][3] = 0;
    M[3][0] = 0;
    M[3][1] = 0;
    M[3][2] = 0;
    M[3][3] = 1;
}
void robotics_funcs_consts::convert_input_to_rad(double * theta_1,double * theta_2,double * theta_3,double * theta_4,double * theta_5)
{
    const double PI = 3.14159;
    *theta_1 = *theta_1*PI/180;
    *theta_2 = *theta_2*PI/180;
    *theta_3 = *theta_3*PI/180;
    *theta_4 = *theta_4*PI/180;
    *theta_5 = *theta_5*PI/180;
}

void robotics_funcs_consts::find_all_thetas(double four_wrt_three[4][4], double five_wrt_four[4][4], double five_wrt_zero[4][4])
{
    double five_wrt_three [4][4];   // 3T5
    double three_wrt_zero[4][4];     //0T3
    double zero_wrt_three[4][4];     //3T0
    double theta_1 = 0,theta_2 = 0,theta_3 = 0,theta_4 = 0,theta_5 = 0;
    robotics_funcs_consts robo_math;

    robo_math.matrix_multiply(four_wrt_three,five_wrt_four,five_wrt_three); // created 3T4*4T5 = 3T5
    robo_math.invert_matrix(five_wrt_three);      // invert the 3T5 matrix to become 5T3 matrix
    robo_math.matrix_multiply(five_wrt_zero,five_wrt_three,three_wrt_zero);       // 0T3 = 0T5*5T3
    theta_1 = atan2(three_wrt_zero[1][3],three_wrt_zero[0][3]);     // theta_1 = tan(R_24/R_14)   // MAY HAVE TO ADD IF LOGIC HERE!!!                                                              // this returns the value in radians....
    // finding theta_3
    double a = sqrt((three_wrt_zero[1][3]*three_wrt_zero[1][3]) + (three_wrt_zero[0][3]*three_wrt_zero[0][3]));                       // cos(theta_1) accepts a value in radians
    double z = three_wrt_zero[2][3] - d1;

    double theta_3_arg = (a*a+ z*z - a2*a2 - a3*a3)/(2*a2*a3);
    std::cout << theta_3_arg << std::endl;
    if((theta_3_arg > 0.9999) && (theta_3_arg < 1.0001)) theta_3 = 0;
    else theta_3 = acos(theta_3_arg);

    // finding theta_2 now...
    theta_2 = atan2(z,a) + atan2(a3*sin(theta_3),a2+ a3*cos(theta_3));
    //finding thetas 4 and 5....
    robo_math.invert_matrix(five_wrt_three);     // 5T3 becomes 3T5 again.

    robo_math.create_zero_wrt_three_mtx(zero_wrt_three,theta_1, theta_2, theta_3);  // instantiate the 3T0 matrix
    robo_math.matrix_multiply(zero_wrt_three,five_wrt_zero,five_wrt_three);         //3T5 = 3T0*0T5
    theta_4 = atan2(five_wrt_three[1][2],five_wrt_three[0][2]);
    theta_5 = atan2(five_wrt_three[2][0],five_wrt_three[2][1]);

    std::cout << theta_1*rad_to_deg << " " << theta_2*rad_to_deg << " " << theta_3*rad_to_deg << " " << theta_4*rad_to_deg << " " << theta_5*rad_to_deg << std::endl;
}

robotics_funcs_consts::~robotics_funcs_consts()        // destructor function
{
}
